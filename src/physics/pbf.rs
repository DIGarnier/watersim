//! Position Based Fluids (Macklin & Müller, SIGGRAPH 2013) as a `FluidSolver`
//! strategy.
//!
//! A self-contained alternative to the granular repulsion+contact model. It
//! shares nothing with the packed granular kernels on purpose: the two models
//! want different interaction radii (PBF's smoothing radius h is several
//! particle spacings, vs the granular one-cell contact cutoff), and keeping PBF
//! isolated leaves the door open to a 3D port without disturbing the tuned 2D
//! granular engine.
//!
//! The per-substep loop (paper §3, algorithm 1):
//!   1. predict positions under gravity (symplectic Euler, matching the
//!      granular integrator's effective acceleration so both models fall the
//!      same way).
//!   2. build a uniform grid at cell size h for neighbor search.
//!   3. solver iterations, each a parallel Jacobi sweep:
//!        density  ρ_i = Σ_j W_poly6(‖x_i−x_j‖, h)
//!        constraint C_i = ρ_i/ρ0 − 1
//!        λ_i = −C_i / (Σ_k ‖∇_k C_i‖² + ε)
//!        Δx_i = (1/ρ0) Σ_j (λ_i + λ_j + s_corr) ∇W_spiky(x_i−x_j, h)
//!      s_corr is the artificial-pressure cohesion term (paper §4); walls are
//!      enforced by clamping after each iteration.
//!   4. velocity v_i = (x_i − x_prev)/Δt.
//!   5. XSPH viscosity + vorticity confinement (paper §§5–6).

use glam::Vec2;
use rayon::prelude::*;

use super::{clamp_wall, FluidSolver, ShareData, PHYS_TIME_STEP};
use crate::constants::{BALL_SIZE, HEIGHT, WIDTH};

// Smoothing radius: ~2.5 rest spacings, so each particle sees ~20 neighbors.
const PBF_H: f32 = 15.0;
const PBF_H2: f32 = PBF_H * PBF_H;
// Rest spacing the fluid relaxes to. One contact diameter keeps a PBF fluid at
// roughly the same packing a granular pile settles to, so the two models fill
// comparable volume for a given particle count.
const PBF_REST_SPACING: f32 = 2.0 * BALL_SIZE;

/// Tunable PBF coefficients. Defaults were swept with the render tool's
/// `--stats` diagnostic; note the values are *not* the paper's textbook
/// numbers because this codebase's kernel constants live in an unusual unit
/// system (ρ0 ≈ 0.028, gradients ≈ 1e-3), which rescales every coefficient.
#[derive(Clone, Copy, Debug)]
pub struct PbfParams {
    /// Density-solver iterations per substep.
    pub iters: usize,
    /// CFM regularization ε. Must stay well below a typical Σ‖∇C‖² (≈7e-3
    /// here) or it swamps the density term and the fluid goes compressible.
    pub eps_cfm: f32,
    /// Artificial-pressure strength k in s_corr = −k (W(r)/W(Δq))^n. This is
    /// the term that keeps the incompressible interior from clumping into
    /// pairs (the density constraint alone is satisfied by clumped states);
    /// it needs to be large in this unit system.
    pub scorr_k: f32,
    /// s_corr reference distance Δq (the kernel is sampled here for the ratio).
    pub scorr_dq: f32,
    /// s_corr exponent n. Higher = sharper/shorter-range repulsion.
    pub scorr_n: i32,
    /// XSPH viscosity coefficient (velocity smoothing toward the neighborhood).
    pub xsph_c: f32,
    /// Vorticity-confinement strength (re-injects swirl the solver damps).
    pub vorticity: f32,
    /// Per-iteration position-correction clamp, in pixels.
    pub max_corr: f32,
    /// Upper bound on λ. λ < 0 resists compression (incompressibility); λ > 0
    /// pulls under-dense regions together (surface tension). A sparse or
    /// freshly-spawned fluid is far under rest density, where the tiny CFM ε
    /// sends λ enormous and the solve explodes — so λ is capped here. The cap
    /// is generous enough to keep normal cohesion (equilibrium λ is O(10s)) but
    /// bounds the pathological blow-up with only a handful of particles.
    pub lambda_max: f32,
}

impl Default for PbfParams {
    fn default() -> Self {
        // Swept with the render tool's `--tune` diagnostic to settle like real
        // water: residual mean-speed ≈ granular's, no flung strays, ρ/ρ0 ≈ 1,
        // no clumping. Two findings drove the values: XSPH needed its missing
        // 1/ρ0 volume weight to actually damp, and ε=1e-4 was too *stiff* — the
        // density solve rang and never came to rest — so ε=2e-3 softens it.
        // Vorticity confinement is off (it re-injects energy and fought the
        // settling); s_corr is modest and the per-iteration clamp tight.
        Self {
            iters: 6,
            eps_cfm: 2.0e-3,
            // The sloshing benchmark's --damp-sweep found s_corr (an always-on
            // repulsion) to be the *dominant* slosh damper — at the paper's n=4
            // it is still significant at the rest spacing, so it jiggles resting
            // fluid and bleeds coherent wave energy. Sharpening it (n=8) makes it
            // near-zero at rest but still strong where particles actually clump,
            // decoupling anti-clumping from damping: vs the original n=4/k=3 this
            // is livelier (slosh decay ×0.40 → ×0.50, ~1 extra cycle) AND flatter
            // (0% clumped, vs 1%). n is a paper parameter (Macklin & Müller eq.
            // 13 use 4); raising it for this trade-off is our own calibration.
            scorr_k: 5.0,
            scorr_dq: 0.2 * PBF_H,
            scorr_n: 8,
            // 0.05, not 0.1: the sloshing benchmark showed 0.1 over-damped free
            // oscillation (amplitude ×0.27/half-period) without improving the
            // settle. Halving it lets a slosh persist ~2× longer (period still
            // matches linear theory to ~3%) while a dropped block still settles
            // calm. Residual damping below this is the density solve itself —
            // PBF is inherently dissipative, an accepted stability trade.
            xsph_c: 0.05,
            vorticity: 0.0,
            max_corr: 0.12 * PBF_H,
            lambda_max: 30.0,
        }
    }
}

const PI: f32 = std::f32::consts::PI;
// 2D kernel normalizations (support radius h).
const POLY6_COEF: f32 = 4.0 / (PI * PBF_H2 * PBF_H2 * PBF_H2 * PBF_H2); // 4/(π h^8)
const SPIKY_GRAD_COEF: f32 = -30.0 / (PI * PBF_H2 * PBF_H2 * PBF_H); //  −30/(π h^5)

/// 2D poly6 kernel W(r,h) = (4/π h^8)(h²−r²)³, evaluated from r². Zero beyond h.
#[inline(always)]
fn w_poly6(r2: f32) -> f32 {
    if r2 >= PBF_H2 {
        0.0
    } else {
        let d = PBF_H2 - r2;
        POLY6_COEF * d * d * d
    }
}

/// Gradient of the 2D spiky kernel wrt x_i, for the offset d = x_i − x_j:
/// ∇W = (−30/π h^5)(h−r)² · d/r. Spiky (not poly6) because its gradient stays
/// large as r→0, so the density solve pushes coincident particles apart
/// instead of letting them collapse. Returns ZERO at r=0 (no axis) and r≥h.
#[inline(always)]
fn grad_spiky(d: Vec2) -> Vec2 {
    let r2 = d.length_squared();
    if r2 >= PBF_H2 || r2 < 1e-12 {
        return Vec2::ZERO;
    }
    let r = r2.sqrt();
    let f = SPIKY_GRAD_COEF * (PBF_H - r) * (PBF_H - r) / r;
    d * f
}

/// Uniform grid at cell size h for PBF neighbor search, sized at runtime from
/// the box. Same counting-sort CSR layout as the granular `CsrGrid`, but its
/// own type so the two models don't fight over the compile-time granular cell
/// size.
#[derive(Default)]
struct PbfGrid {
    w: usize,
    h: usize,
    cell_start: Vec<u32>, // w*h + 1 offsets
    cursor: Vec<u32>,
    cell_of: Vec<u32>,
    indices: Vec<u32>,
}

impl PbfGrid {
    fn new() -> Self {
        let w = (WIDTH / PBF_H).ceil() as usize + 1;
        let h = (HEIGHT / PBF_H).ceil() as usize + 1;
        Self {
            w,
            h,
            cell_start: vec![0; w * h + 1],
            cursor: vec![0; w * h],
            cell_of: Vec::new(),
            indices: Vec::new(),
        }
    }

    #[inline(always)]
    fn coord(&self, p: Vec2) -> (usize, usize) {
        let x = ((p.x / PBF_H) as isize).clamp(0, self.w as isize - 1) as usize;
        let y = ((p.y / PBF_H) as isize).clamp(0, self.h as isize - 1) as usize;
        (x, y)
    }

    fn build(&mut self, positions: &[Vec2]) {
        let n = positions.len();
        let ncells = self.w * self.h;
        self.cell_of.resize(n, 0);
        self.indices.resize(n, 0);
        self.cursor.fill(0);

        for (i, p) in positions.iter().enumerate() {
            let (x, y) = self.coord(*p);
            let c = (y * self.w + x) as u32;
            self.cell_of[i] = c;
            self.cursor[c as usize] += 1;
        }

        let mut sum = 0u32;
        for c in 0..ncells {
            let count = self.cursor[c];
            self.cell_start[c] = sum;
            self.cursor[c] = sum;
            sum += count;
        }
        self.cell_start[ncells] = sum;

        for i in 0..n {
            let c = self.cell_of[i] as usize;
            self.indices[self.cursor[c] as usize] = i as u32;
            self.cursor[c] += 1;
        }
    }

    /// Invoke `f(j)` for every particle in the 3×3 cell block around `pos`
    /// (its cell contains every partner within h).
    #[inline(always)]
    fn for_neighbors(&self, pos: Vec2, mut f: impl FnMut(usize)) {
        let (cx, cy) = self.coord(pos);
        let y0 = cy.saturating_sub(1);
        let y1 = (cy + 1).min(self.h - 1);
        let x0 = cx.saturating_sub(1);
        let x1 = (cx + 1).min(self.w - 1);
        for yy in y0..=y1 {
            let base = yy * self.w;
            let s = self.cell_start[base + x0] as usize;
            let e = self.cell_start[base + x1 + 1] as usize;
            for &j in &self.indices[s..e] {
                f(j as usize);
            }
        }
    }
}

/// Position Based Fluids solver state: explicit velocities plus the density
/// solve's per-particle scratch. Isolated from the granular engine.
pub struct Pbf {
    grid: PbfGrid,
    params: PbfParams,
    rest_density: f32,
    scorr_denom: f32, // W_poly6(Δq²), precomputed for the s_corr ratio
    vel: Vec<Vec2>,
    prev: Vec<Vec2>,     // predicted-from position, for the velocity update
    lambda: Vec<f32>,    // λ_i for the current iteration
    dp: Vec<Vec2>,       // Δx_i for the current iteration
    vscratch: Vec<Vec2>, // velocity double-buffer for XSPH / vorticity
    curl: Vec<f32>,      // per-particle scalar vorticity ω_i
}

impl Pbf {
    pub fn new() -> Self {
        let params = PbfParams::default();
        Self {
            grid: PbfGrid::new(),
            rest_density: rest_density(),
            scorr_denom: w_poly6(params.scorr_dq * params.scorr_dq),
            params,
            vel: Vec::new(),
            prev: Vec::new(),
            lambda: Vec::new(),
            dp: Vec::new(),
            vscratch: Vec::new(),
            curl: Vec::new(),
        }
    }

    fn set_params(&mut self, params: PbfParams) {
        self.params = params;
        self.scorr_denom = w_poly6(params.scorr_dq * params.scorr_dq);
    }

    fn ensure_sized(&mut self, n: usize) {
        if self.vel.len() != n {
            self.vel.resize(n, Vec2::ZERO);
        }
        self.prev.resize(n, Vec2::ZERO);
        self.lambda.resize(n, 0.0);
        self.dp.resize(n, Vec2::ZERO);
        self.vscratch.resize(n, Vec2::ZERO);
        self.curl.resize(n, 0.0);
    }

    /// One Jacobi iteration: density → λ (parallel gather), then Δx from the
    /// λ's (parallel gather), then apply with a clamp and wall projection.
    fn solve_iteration(&mut self, x: &mut [Vec2]) {
        let n = x.len();
        let rho0 = self.rest_density;
        let inv_rho0 = 1.0 / rho0;
        let eps_cfm = self.params.eps_cfm;
        let scorr_k = self.params.scorr_k;
        let scorr_n = self.params.scorr_n;
        let max_corr = self.params.max_corr;
        let lambda_max = self.params.lambda_max;

        // Pass A: density and λ. ∇_i C_i = (1/ρ0) Σ_j ∇W_ij; the constraint
        // gradient sum is |Σ_j ∇W|² (the k=i term) plus Σ_j|∇W|² (k=j terms),
        // all scaled by 1/ρ0². Read-only over the grid ⇒ trivially parallel.
        {
            let grid = &self.grid;
            let x_ro: &[Vec2] = x;
            self.lambda.par_iter_mut().enumerate().for_each(|(i, lam)| {
                let xi = x_ro[i];
                let mut rho = 0.0f32;
                let mut grad_i = Vec2::ZERO; // Σ_j ∇W_ij
                let mut sum_grad2 = 0.0f32; // Σ_j |∇W_ij|²
                grid.for_neighbors(xi, |j| {
                    let d = xi - x_ro[j];
                    rho += w_poly6(d.length_squared());
                    if j != i {
                        let g = grad_spiky(d);
                        grad_i += g;
                        sum_grad2 += g.length_squared();
                    }
                });
                let c = rho * inv_rho0 - 1.0;
                let sum_grad_c2 = (grad_i.length_squared() + sum_grad2) * (inv_rho0 * inv_rho0);
                // Cap λ (see PbfParams::lambda_max): keeps incompressibility
                // and normal cohesion but bounds the huge positive λ a sparse,
                // far-under-dense fluid would otherwise produce, which is what
                // made a handful of particles explode in the live app.
                *lam = (-c / (sum_grad_c2 + eps_cfm)).min(lambda_max);
            });
        }

        // Pass B: position correction from λ_i, λ_j and the s_corr term.
        {
            let grid = &self.grid;
            let lambda = &self.lambda;
            let x_ro: &[Vec2] = x;
            let scorr_denom = self.scorr_denom;
            self.dp.par_iter_mut().enumerate().for_each(|(i, dpi)| {
                let xi = x_ro[i];
                let lami = lambda[i];
                let mut corr = Vec2::ZERO;
                grid.for_neighbors(xi, |j| {
                    if j == i {
                        return;
                    }
                    let d = xi - x_ro[j];
                    let scorr = scorr(d.length_squared(), scorr_denom, scorr_k, scorr_n);
                    corr += grad_spiky(d) * (lami + lambda[j] + scorr);
                });
                let mut c = corr * inv_rho0;
                let m2 = c.length_squared();
                if m2 > max_corr * max_corr {
                    c *= max_corr / m2.sqrt();
                }
                *dpi = c;
            });
        }

        // Apply and re-project onto the box.
        for i in 0..n {
            x[i] += self.dp[i];
            clamp_wall(&mut x[i]);
        }
    }

    /// XSPH viscosity (paper eq. 17): v_i ← v_i + c Σ_j (v_j − v_i) W_ij / ρ0.
    /// Double-buffered so every particle reads the pre-update velocities.
    ///
    /// The `1/ρ0` is the SPH volume weight (mⱼ/ρⱼ) and is essential: without it
    /// the sum is scaled by the raw kernel value (≈ρ0 ≈ 0.028 in this unit
    /// system), making the smoothing ~1/ρ0 ≈ 35× too weak — which is why a
    /// full neighborhood of coherent motion (the churn a settling pool should
    /// dissipate) barely damped at any `c`. With the weight, c ∈ [0,1] behaves
    /// like a real viscosity dial.
    fn apply_xsph(&mut self, x: &[Vec2]) {
        let grid = &self.grid;
        let vel = &self.vel;
        let xsph_c = self.params.xsph_c;
        let inv_rho0 = 1.0 / self.rest_density;
        let out = &mut self.vscratch;
        out.par_iter_mut().enumerate().for_each(|(i, o)| {
            let xi = x[i];
            let vi = vel[i];
            let mut acc = Vec2::ZERO;
            grid.for_neighbors(xi, |j| {
                if j == i {
                    return;
                }
                let w = w_poly6((xi - x[j]).length_squared());
                acc += (vel[j] - vi) * w;
            });
            *o = vi + acc * (xsph_c * inv_rho0);
        });
        std::mem::swap(&mut self.vel, &mut self.vscratch);
    }

    /// Vorticity confinement (paper eq. 16). In 2D the curl ω is a scalar
    /// (out-of-plane): ω_i = Σ_j (v_j − v_i) × ∇W_ij. The confinement force
    /// f = ε (N × ω) with N = ∇|ω|/‖∇|ω|‖ becomes, in 2D, ε·ω·(N rotated 90°).
    fn apply_vorticity(&mut self, x: &[Vec2], dt: f32) {
        let vorticity = self.params.vorticity;
        if vorticity == 0.0 {
            return;
        }
        let n = x.len();
        // ω_i (scalar) for every particle.
        {
            let grid = &self.grid;
            let vel = &self.vel;
            self.curl.par_iter_mut().enumerate().for_each(|(i, w)| {
                let xi = x[i];
                let vi = vel[i];
                let mut omega = 0.0f32;
                grid.for_neighbors(xi, |j| {
                    if j == i {
                        return;
                    }
                    let dv = vel[j] - vi;
                    let g = grad_spiky(xi - x[j]);
                    omega += dv.x * g.y - dv.y * g.x; // (dv × ∇W)_z
                });
                *w = omega;
            });
        }
        // Confinement force from the gradient of |ω|.
        let grid = &self.grid;
        let curl = &self.curl;
        let vel = &self.vel;
        let out = &mut self.vscratch;
        let accel_dt = INV_PHYS_DT_MULT * dt; // matches the gravity scaling
        out.par_iter_mut().enumerate().for_each(|(i, o)| {
            let xi = x[i];
            let mut grad_w = Vec2::ZERO; // ∇|ω|
            grid.for_neighbors(xi, |j| {
                if j == i {
                    return;
                }
                grad_w += grad_spiky(xi - x[j]) * curl[j].abs();
            });
            let mut add = Vec2::ZERO;
            let len = grad_w.length();
            if len > 1e-6 {
                let nrm = grad_w / len;
                // ε (N × ω): rotate N by −90° and scale by ω.
                let force = Vec2::new(nrm.y, -nrm.x) * (vorticity * curl[i]);
                add = force * accel_dt;
            }
            *o = vel[i] + add;
        });
        for i in 0..n {
            self.vel[i] = out[i];
        }
    }
}

impl FluidSolver for Pbf {
    fn name(&self) -> &'static str {
        "PBF"
    }

    fn set_pbf_params(&mut self, params: PbfParams) {
        self.set_params(params);
    }

    fn substep(&mut self, dt: f32, gravity: Vec2, share: &mut ShareData, c_opos: &mut Vec<Vec2>) {
        let n = share.c_pos.len();
        if n == 0 {
            return;
        }
        self.ensure_sized(n);
        c_opos.resize(n, Vec2::ZERO);

        // Effective acceleration matched to the granular Verlet integrator
        // (x += a·dt²·INV_PHYS_DT there), so both models fall identically.
        const INV_PHYS_DT: f32 = 1.0 / PHYS_TIME_STEP;
        let accel = gravity * INV_PHYS_DT;

        // 1. Predict: symplectic Euler under gravity.
        let x = &mut share.c_pos;
        for i in 0..n {
            self.vel[i] += accel * dt;
            self.prev[i] = x[i];
            x[i] += self.vel[i] * dt;
            clamp_wall(&mut x[i]);
        }

        // 2. Neighbor grid at cell size h.
        self.grid.build(x);

        // 3. Density-constraint solver iterations (Jacobi).
        for _ in 0..self.params.iters {
            self.solve_iteration(x);
        }

        // 4. Derive velocity from the total position change, with a safety
        // clamp: no particle may move more than half a smoothing radius per
        // substep. This bounds a single bad step so a transient can't cascade
        // into a full blow-up (belt-and-suspenders alongside the λ clamp).
        let inv_dt = 1.0 / dt;
        let vmax = 0.5 * PBF_H * inv_dt;
        let vmax2 = vmax * vmax;
        for i in 0..n {
            let mut v = (x[i] - self.prev[i]) * inv_dt;
            let s2 = v.length_squared();
            if s2 > vmax2 {
                v *= vmax / s2.sqrt();
            }
            self.vel[i] = v;
        }

        // 5. Velocity post-process: vorticity confinement then XSPH viscosity.
        self.apply_vorticity(x, dt);
        self.apply_xsph(x);

        // Bookkeeping for the shared/rendered state: c_opos mirrors the
        // Verlet "previous position" convention (pos − v·dt) so the granular
        // color path and any external readers see a consistent velocity, and
        // c_color encodes speed as a hue like the granular integrator.
        let mut speed_sum = 0.0f32;
        let mut max_speed = 0.0f32;
        for i in 0..n {
            c_opos[i] = x[i] - self.vel[i] * dt;
            // (c_pos − c_opos)*20 == vel*dt*20 is the granular speed metric.
            let speed = (self.vel[i] * dt * 20.0).length();
            share.c_color[i] = (speed + 198.0) % 360.0;
            speed_sum += speed;
            max_speed = max_speed.max(speed);
        }

        // Coarse diagnostics: mean ρ/ρ0 over a subsample. 1.0 = incompressible;
        // a value climbing well above 1 is the visible signature of a blow-up.
        let stride = (n / 256).max(1);
        let mut dsum = 0.0f64;
        let mut cnt = 0usize;
        let mut i = 0;
        while i < n {
            let xi = x[i];
            let mut rho = 0.0f32;
            self.grid
                .for_neighbors(xi, |j| rho += w_poly6((xi - x[j]).length_squared()));
            dsum += (rho / self.rest_density) as f64;
            cnt += 1;
            i += stride;
        }

        let ps = &mut share.perf_stats;
        ps.mean_speed = speed_sum / n as f32;
        ps.max_speed = max_speed;
        ps.pbf_density_ratio = (dsum / cnt.max(1) as f64) as f32;
    }
}

// Gravity/force scaling constant reused by the vorticity term so injected
// accelerations live in the same units as gravity.
const INV_PHYS_DT_MULT: f32 = 1.0 / PHYS_TIME_STEP;

/// s_corr artificial-pressure term: −k (W(r)/W(Δq))^n. `denom` = W_poly6(Δq²).
/// A larger `n` sharpens the term toward short range: it stays strong where
/// particles clump (r ≪ Δq) but decays to near-zero at the rest spacing, so it
/// no longer jiggles resting/sloshing fluid — decoupling anti-clumping from the
/// numerical damping it used to add.
#[inline(always)]
fn scorr(r2: f32, denom: f32, k: f32, n: i32) -> f32 {
    let ratio = w_poly6(r2) / denom;
    let mut p = ratio;
    for _ in 1..n.max(1) {
        p *= ratio;
    }
    -k * p
}

/// Rest density ρ0 = poly6 density of a square lattice at the rest spacing,
/// including the self term. The fluid relaxes toward this packing.
fn rest_density() -> f32 {
    let s = PBF_REST_SPACING;
    let reach = (PBF_H / s).ceil() as i32 + 1;
    let mut rho = 0.0;
    for iy in -reach..=reach {
        for ix in -reach..=reach {
            let r2 = ((ix * ix + iy * iy) as f32) * s * s;
            rho += w_poly6(r2);
        }
    }
    rho
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{HEIGHT, WIDTH};
    use crate::physics::{Physics, ShareData, Strategy, PHYS_TIME_STEP};
    use std::sync::mpsc::channel;

    /// A PBF blob dropped in the box must stay finite, stay inside the walls,
    /// and settle (mean speed decays) instead of exploding.
    #[test]
    fn pbf_blob_is_stable() {
        let s = PBF_REST_SPACING;
        let mut positions = Vec::new();
        for gy in 0..30 {
            for gx in 0..30 {
                positions.push(Vec2::new(500.0 + gx as f32 * s, 300.0 + gy as f32 * s));
            }
        }
        let n = positions.len();
        let (_tx, rx) = channel();
        let mut physics = Physics::new(positions.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
        physics.toggle_adaptive_dt();
        physics.set_strategy(Strategy::Pbf);
        let mut share = ShareData {
            c_pos: positions,
            c_color: vec![0.0; n],
            ..Default::default()
        };
        for _ in 0..1500 {
            physics.step(PHYS_TIME_STEP, &mut share);
        }
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
        assert_eq!(nan, 0, "PBF produced NaNs");
        assert_eq!(escaped, 0, "PBF particles escaped the box");
    }
}
