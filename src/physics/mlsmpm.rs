//! MLS-MPM (Hu et al., SIGGRAPH 2018) as a `FluidSolver` strategy.
//!
//! A **hybrid** Eulerian–Lagrangian method, structurally unlike every other
//! solver here: particles carry the state (position, velocity, an APIC affine
//! velocity matrix C, and — for solids — a deformation gradient F), while a
//! background grid does the momentum solve each step. Transfers use the
//! Moving-Least-Squares / APIC weighting, which conserves both linear and
//! angular momentum.
//!
//! What it buys us that nothing else here can: swapping the *constitutive
//! model* turns the identical solver from a weakly-compressible fluid into an
//! elastic **jelly** (and, with a plasticity return map, sand or snow). The
//! default material is `Liquid` so it slots into the water comparison; `Jelly`
//! shows the elastic-solid differentiator.
//!
//! Per substep (Hu's "MLS-MPM" / the taichi 88-line formulation, in 2D):
//!   1. **P2G** — scatter mass and APIC momentum to the 3×3 grid nodes around
//!      each particle, folding in the internal stress via the MLS affine form
//!      `affine = m·C − Δt·V·(4/Δx²)·σ`.
//!   2. **grid update** — momentum→velocity, add gravity, apply wall BCs.
//!   3. **G2P** — gather velocity (and reconstruct C) back to particles,
//!      advect, and update J (liquid) or F (jelly).

use glam::{Mat2, Vec2};

use super::{clamp_wall, FluidSolver, ShareData, PHYS_TIME_STEP};
use crate::constants::{BALL_SIZE, HEIGHT, WIDTH};

// Grid spacing: two particle spacings, so a rest-packed fluid seeds ~4
// particles per cell (the MPM sweet spot). Support is the 3×3 block of nodes.
const MPM_DX: f32 = 4.0 * BALL_SIZE; // = 2 × rest spacing (2·BALL_SIZE)
const MPM_INV_DX: f32 = 1.0 / MPM_DX;
// One-cell halo (`HALO`) on every side: a particle at the wall has base cell
// −1, so node indices are shifted by +HALO and the grid is padded by 2·HALO+1
// cells, keeping the whole 3×3 stencil in bounds without desyncing weights.
const HALO: isize = 1;
const GW: usize = (WIDTH / MPM_DX) as usize + 2 * HALO as usize + 1;
const GH: usize = (HEIGHT / MPM_DX) as usize + 2 * HALO as usize + 1;
// Volume/mass one particle represents (rest spacing², unit density).
const P_VOL: f32 = (2.0 * BALL_SIZE) * (2.0 * BALL_SIZE);
const P_MASS: f32 = P_VOL; // unit rest density
                           // Grid nodes within this many cells of a wall get their into-wall velocity
                           // component zeroed (sticky floor / slip walls).
const MPM_BND: usize = 3;

/// Which constitutive model the MPM particles obey.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum MpmMaterial {
    /// Weakly-compressible fluid (equation of state on the volume ratio J).
    #[default]
    Liquid,
    /// Fixed-corotated elastic solid — wobbles and holds its shape (jelly).
    Jelly,
}

/// Tunable MLS-MPM coefficients.
#[derive(Clone, Copy, Debug)]
pub struct MpmParams {
    pub material: MpmMaterial,
    /// Bulk modulus λ for the volumetric (pressure) response. Higher = stiffer
    /// / less compressible, but the CFL limit tightens.
    pub bulk: f32,
    /// Shear modulus μ (jelly only; 0 for a liquid, which sustains no shear).
    pub shear: f32,
}

impl Default for MpmParams {
    fn default() -> Self {
        // Swept against the drop-and-settle test (mpm_bulk_sweep): the stiffness
        // must balance this codebase's large effective gravity (accel ≈ 4700).
        // At bulk ≥ 5e4 a dropped blob spreads into a thin floor puddle with the
        // interior volume ratio J ≈ 0.97 (incompressible); 2e5 sits comfortably
        // below the CFL limit (stable even at 1e6). Shear (jelly only) is set an
        // order below so a jelly wobbles and holds shape without locking up.
        Self {
            material: MpmMaterial::Liquid,
            bulk: 2.0e5,
            shear: 4.0e4,
        }
    }
}

/// MLS-MPM solver state. Per-particle Lagrangian state (v, C, and J or F) lives
/// here; the Eulerian grid is scratch rebuilt every substep.
pub struct Mlsmpm {
    params: MpmParams,
    vel: Vec<Vec2>,
    cmat: Vec<Mat2>, // APIC affine velocity C
    jdet: Vec<f32>,  // volume ratio J = det(F) (liquid)
    fmat: Vec<Mat2>, // deformation gradient F (jelly)
    grid_v: Vec<Vec2>,
    grid_m: Vec<f32>,
}

impl Default for Mlsmpm {
    fn default() -> Self {
        Self::new()
    }
}

impl Mlsmpm {
    pub fn new() -> Self {
        Self {
            params: MpmParams::default(),
            vel: Vec::new(),
            cmat: Vec::new(),
            jdet: Vec::new(),
            fmat: Vec::new(),
            grid_v: vec![Vec2::ZERO; GW * GH],
            grid_m: vec![0.0; GW * GH],
        }
    }

    pub fn set_params(&mut self, params: MpmParams) {
        self.params = params;
    }

    fn ensure_sized(&mut self, n: usize) {
        if self.vel.len() != n {
            // Grow (cannon) / init: newcomers start at rest, undeformed.
            self.vel.resize(n, Vec2::ZERO);
            self.cmat.resize(n, Mat2::ZERO);
            self.jdet.resize(n, 1.0);
            self.fmat.resize(n, Mat2::IDENTITY);
        }
    }

    /// Quadratic B-spline weights for the 3 nodes along each axis, plus the
    /// integer base node (unclamped; may be −1 at a wall — the HALO absorbs it)
    /// and the fractional offset `fx ∈ [0.5, 1.5)`.
    #[inline(always)]
    fn weights(xp: Vec2) -> ([isize; 2], Vec2, [Vec2; 3]) {
        let cell = xp * MPM_INV_DX;
        let base_f = (cell - Vec2::splat(0.5)).floor();
        let fx = cell - base_f;
        let w = [
            (Vec2::splat(1.5) - fx) * (Vec2::splat(1.5) - fx) * 0.5,
            Vec2::splat(0.75) - (fx - Vec2::splat(1.0)) * (fx - Vec2::splat(1.0)),
            (fx - Vec2::splat(0.5)) * (fx - Vec2::splat(0.5)) * 0.5,
        ];
        ([base_f.x as isize, base_f.y as isize], fx, w)
    }

    /// Flat node index for stencil offset (i, j) around `base`, shifted by the
    /// halo and clamped to the padded grid (the clamp only ever bites if a
    /// particle reaches the very corner, which the wall projection prevents).
    #[inline(always)]
    fn node(base: [isize; 2], i: usize, j: usize) -> usize {
        let nx = (base[0] + i as isize + HALO).clamp(0, GW as isize - 1) as usize;
        let ny = (base[1] + j as isize + HALO).clamp(0, GH as isize - 1) as usize;
        ny * GW + nx
    }

    /// Cauchy stress × the MLS-MPM prefactor `−Δt·V·(4/Δx²)`, plus the APIC
    /// `m·C` term — i.e. the "affine" matrix scattered in P2G.
    #[inline(always)]
    fn affine(&self, p: usize, dt: f32) -> Mat2 {
        let pre = -dt * P_VOL * 4.0 * MPM_INV_DX * MPM_INV_DX;
        let stress = match self.params.material {
            MpmMaterial::Liquid => {
                // Equation of state on J: pressure ∝ λ·J·(J−1) (isotropic).
                let j = self.jdet[p];
                Mat2::IDENTITY * (self.params.bulk * j * (j - 1.0))
            }
            MpmMaterial::Jelly => {
                // Fixed-corotated: σ = 2μ(F−R)Fᵀ + λ·J·(J−1)·I, R from the 2D
                // polar decomposition of F (closed form, no SVD).
                let f = self.fmat[p];
                let j = f.determinant();
                let r = polar_rotation(f);
                let mu = self.params.shear;
                let la = self.params.bulk;
                (f - r) * f.transpose() * (2.0 * mu) + Mat2::IDENTITY * (la * j * (j - 1.0))
            }
        };
        stress * pre + self.cmat[p] * P_MASS
    }

    fn p2g(&mut self, x: &[Vec2], dt: f32) {
        self.grid_v.iter_mut().for_each(|v| *v = Vec2::ZERO);
        self.grid_m.iter_mut().for_each(|m| *m = 0.0);

        for p in 0..x.len() {
            let (base, fx, w) = Self::weights(x[p]);
            let affine = self.affine(p, dt);
            let mv = self.vel[p] * P_MASS;
            for j in 0..3 {
                for i in 0..3 {
                    let weight = w[i].x * w[j].y;
                    let dpos = (Vec2::new(i as f32, j as f32) - fx) * MPM_DX;
                    let node = Self::node(base, i, j);
                    self.grid_v[node] += (mv + affine * dpos) * weight;
                    self.grid_m[node] += weight * P_MASS;
                }
            }
        }
    }

    fn grid_update(&mut self, gravity: Vec2, dt: f32) {
        // Match the other models' effective acceleration so all fall the same.
        let dv = gravity * (dt / PHYS_TIME_STEP);
        for node in 0..GW * GH {
            let m = self.grid_m[node];
            if m > 0.0 {
                let mut v = self.grid_v[node] / m + dv;
                let (gx, gy) = (node % GW, node / GW);
                if gx < MPM_BND && v.x < 0.0 {
                    v.x = 0.0;
                }
                if gx >= GW - MPM_BND && v.x > 0.0 {
                    v.x = 0.0;
                }
                if gy < MPM_BND && v.y < 0.0 {
                    v.y = 0.0;
                }
                if gy >= GH - MPM_BND && v.y > 0.0 {
                    v.y = 0.0;
                }
                self.grid_v[node] = v;
            }
        }
    }

    fn g2p(&mut self, x: &mut [Vec2], dt: f32) {
        for p in 0..x.len() {
            let (base, fx, w) = Self::weights(x[p]);
            let mut new_v = Vec2::ZERO;
            let mut new_c = Mat2::ZERO;
            for j in 0..3 {
                for i in 0..3 {
                    let weight = w[i].x * w[j].y;
                    let dpos = Vec2::new(i as f32, j as f32) - fx;
                    let gv = self.grid_v[Self::node(base, i, j)];
                    new_v += gv * weight;
                    // C = (4/Δx²) Σ w·gv⊗dpos ; dpos is in cell units so one
                    // 1/Δx is folded and the other stays explicit below.
                    new_c += Mat2::from_cols(gv * (dpos.x * weight), gv * (dpos.y * weight));
                }
            }
            new_c *= 4.0 * MPM_INV_DX;
            self.vel[p] = new_v;
            self.cmat[p] = new_c;

            // Advect, then wall-clamp the position (belt-and-suspenders with the
            // grid BC).
            let mut xp = x[p] + new_v * dt;
            clamp_wall(&mut xp);
            x[p] = xp;

            // Evolve the material state by F ← (I + Δt·C)·F.
            let grad = Mat2::IDENTITY + new_c * dt;
            match self.params.material {
                MpmMaterial::Liquid => {
                    // Only J = det(F) matters for a liquid; accumulate it, then
                    // clamp. Without the clamp a violent compression (e.g. the
                    // floor impact) drives J→0, where the J·(J−1) EOS gives ~0
                    // restoring force and J can never recover — the fluid goes
                    // permanently "pressureless". Clamping keeps the pressure
                    // strong enough at the extremes to push J back toward 1.
                    self.jdet[p] = (self.jdet[p] * grad.determinant()).clamp(0.6, 1.4);
                }
                MpmMaterial::Jelly => {
                    self.fmat[p] = grad * self.fmat[p];
                }
            }
        }
    }
}

impl FluidSolver for Mlsmpm {
    fn name(&self) -> &'static str {
        "MLS-MPM"
    }

    fn set_mpm_params(&mut self, params: MpmParams) {
        self.set_params(params);
    }

    fn substep(&mut self, dt: f32, gravity: Vec2, share: &mut ShareData, c_opos: &mut Vec<Vec2>) {
        let n = share.c_pos.len();
        if n == 0 {
            return;
        }
        self.ensure_sized(n);
        c_opos.resize(n, Vec2::ZERO);

        self.p2g(&share.c_pos, dt);
        self.grid_update(gravity, dt);
        self.g2p(&mut share.c_pos, dt);

        // Shared/rendered bookkeeping, matching the granular/PBF convention.
        let x = &share.c_pos;
        let mut speed_sum = 0.0f32;
        let mut max_speed = 0.0f32;
        let mut jsum = 0.0f64;
        for i in 0..n {
            c_opos[i] = x[i] - self.vel[i] * dt;
            let speed = (self.vel[i] * dt * 20.0).length();
            share.c_color[i] = (speed + 198.0) % 360.0;
            speed_sum += speed;
            max_speed = max_speed.max(speed);
            jsum += self.jdet[i] as f64;
        }

        let ps = &mut share.perf_stats;
        ps.mean_speed = speed_sum / n as f32;
        ps.max_speed = max_speed;
        // Report mean J (volume ratio) in the density-ratio slot: 1.0 = at rest
        // volume, <1 compressed, >1 expanded. (For jelly J tracks det F.)
        ps.pbf_density_ratio = if self.params.material == MpmMaterial::Liquid {
            (jsum / n as f64) as f32
        } else {
            let mut js = 0.0f64;
            for f in &self.fmat[..n] {
                js += f.determinant() as f64;
            }
            (js / n as f64) as f32
        };
    }
}

/// Rotation R from the 2D polar decomposition F = R·S. Closed form: the angle
/// is atan2(F₁₀ − F₀₁, F₀₀ + F₁₁). Returns identity for a degenerate F.
#[inline(always)]
fn polar_rotation(f: Mat2) -> Mat2 {
    // glam Mat2 is column-major: col0 = (F00, F10), col1 = (F01, F11).
    let (f00, f10) = (f.col(0).x, f.col(0).y);
    let (f01, f11) = (f.col(1).x, f.col(1).y);
    let c = f00 + f11;
    let s = f10 - f01;
    let r = (c * c + s * s).sqrt();
    if r < 1e-9 {
        return Mat2::IDENTITY;
    }
    let (cos, sin) = (c / r, s / r);
    // Rotation [[cos, -sin], [sin, cos]] in column-major cols.
    Mat2::from_cols(Vec2::new(cos, sin), Vec2::new(-sin, cos))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{Physics, ShareData, Strategy, PHYS_TIME_STEP};
    use std::sync::mpsc::channel;

    fn blob() -> Vec<Vec2> {
        let s = 2.0 * BALL_SIZE;
        let mut v = Vec::new();
        for gy in 0..30 {
            for gx in 0..30 {
                v.push(Vec2::new(500.0 + gx as f32 * s, 300.0 + gy as f32 * s));
            }
        }
        v
    }

    fn run_blob(material: MpmMaterial, steps: usize) -> ShareData {
        let positions = blob();
        let n = positions.len();
        let (_tx, rx) = channel();
        let mut physics = Physics::new(positions.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
        physics.toggle_adaptive_dt();
        physics.set_strategy(Strategy::Mlsmpm);
        physics.set_mpm_params(MpmParams {
            material,
            ..MpmParams::default()
        });
        let mut share = ShareData {
            c_pos: positions,
            c_color: vec![0.0; n],
            ..Default::default()
        };
        for _ in 0..steps {
            physics.step(PHYS_TIME_STEP, &mut share);
        }
        share
    }

    fn finite_and_inside(share: &ShareData) -> (usize, usize) {
        let nan = share
            .c_pos
            .iter()
            .filter(|p| !p.x.is_finite() || !p.y.is_finite())
            .count();
        let escaped = share
            .c_pos
            .iter()
            .filter(|p| p.x < -1.0 || p.x > WIDTH + 1.0 || p.y < -1.0 || p.y > HEIGHT + 1.0)
            .count();
        (nan, escaped)
    }

    /// A liquid MLS-MPM blob must fall, spread, and settle to a near-rest
    /// (J ≈ 1) puddle — not collapse to a pressureless clump (J → 0, the bug
    /// the J clamp fixes) nor explode.
    #[test]
    fn mpm_liquid_is_stable() {
        let share = run_blob(MpmMaterial::Liquid, 1500);
        let (nan, escaped) = finite_and_inside(&share);
        assert_eq!(nan, 0, "MPM produced NaNs");
        assert_eq!(escaped, 0, "MPM particles escaped the box");
        assert!(
            share.perf_stats.mean_speed < 10.0,
            "MPM liquid did not settle: mean_speed = {}",
            share.perf_stats.mean_speed
        );
        let j = share.perf_stats.pbf_density_ratio;
        assert!(
            (0.85..=1.15).contains(&j),
            "MPM liquid volume ratio off (collapse/blow-up?): J = {j}"
        );
    }

    /// A jelly (elastic) blob must stay finite and in the box, and — unlike the
    /// liquid — hold together instead of spreading across the floor (its shear
    /// resistance keeps it compact).
    #[test]
    fn mpm_jelly_is_stable() {
        let share = run_blob(MpmMaterial::Jelly, 1200);
        let (nan, escaped) = finite_and_inside(&share);
        assert_eq!(nan, 0, "MPM jelly produced NaNs");
        assert_eq!(escaped, 0, "MPM jelly escaped the box");
        let spread_x = {
            let (mut lo, mut hi) = (f32::MAX, f32::MIN);
            for p in &share.c_pos {
                lo = lo.min(p.x);
                hi = hi.max(p.x);
            }
            hi - lo
        };
        // The initial blob is ~174 px wide. A liquid spreads to ~1400; a jelly
        // holds its shape, so it should stay well under half the box width.
        assert!(
            spread_x < 600.0,
            "MPM jelly spread like a liquid (lost elasticity): spread_x = {spread_x}"
        );
    }
}
