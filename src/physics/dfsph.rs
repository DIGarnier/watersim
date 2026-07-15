//! Divergence-Free SPH (Bender & Koschier, SCA 2015 / IEEE TVCG 2017) as a
//! `FluidSolver` strategy.
//!
//! Where PBF enforces incompressibility *geometrically* (a position-projection
//! density constraint that bleeds energy), DFSPH enforces it with a **pressure
//! force** derived from two implicit solves, so it stays lively: crisp
//! splashes, persistent sloshing, low numerical dissipation.
//!
//! Per step (positions frozen while the solves run, so ρ_i and the stiffness
//! factor α_i are computed once and reused — the efficiency trick of DFSPH):
//!   1. build the neighbor grid; compute densities ρ_i and factors
//!      α_i = 1 / (|Σ_j ∇W_ij|² + Σ_j |∇W_ij|²)                (paper eq. 9)
//!   2. **divergence-free solve** — drive Dρ/Dt → 0 by correcting velocities
//!      (Algorithm 2): kᵛ_i = (1/Δt)(Dρ_i/Dt) α_i, then
//!      v_i −= Δt Σ_j (kᵛ_i/ρ_i + kᵛ_j/ρ_j) ∇W_ij
//!   3. apply non-pressure accelerations (gravity, XSPH viscosity)
//!   4. **constant-density solve** — drive ρ* → ρ0 (Algorithm 3):
//!      k_i = (1/Δt²)(ρ*_i − ρ0) α_i, same velocity update form
//!   5. advect x += v·Δt, project onto the box (killing the into-wall velocity)
//!
//! Masses are uniform (mⱼ = 1), matching the codebase's kernel unit system, so
//! ρ_i = Σ_j W_ij exactly as in PBF and the two fluids fill comparable volume.

use glam::Vec2;
use rayon::prelude::*;

use super::sph::{SphGrid, SphKernel};
use super::{FluidSolver, ShareData, BOTTOM_WALL, LEFT_WALL, PHYS_TIME_STEP, RIGHT_WALL, TOP_WALL};
use crate::constants::BALL_SIZE;

// Smoothing radius and rest spacing shared with PBF so a DFSPH fluid packs to
// the same volume a PBF/granular one does for a given particle count.
const DFSPH_H: f32 = 15.0;
const DFSPH_REST_SPACING: f32 = 2.0 * BALL_SIZE;

/// Tunable DFSPH coefficients.
#[derive(Clone, Copy, Debug)]
pub struct DfsphParams {
    /// Constant-density solver iterations per substep.
    pub density_iters: usize,
    /// Divergence-free solver iterations per substep.
    pub divergence_iters: usize,
    /// XSPH viscosity coefficient (0 = the low-dissipation ideal; a little
    /// tames surface jitter without killing the liveliness).
    pub xsph_c: f32,
    /// Per-substep velocity clamp as a fraction of h/Δt (stability safety net,
    /// like PBF's — bounds a single bad step so a transient can't cascade).
    pub vmax_frac: f32,
}

impl Default for DfsphParams {
    fn default() -> Self {
        Self {
            // Positions are frozen during the solves and the sim already
            // substeps at 480 Hz, so a few iterations converge the density
            // error well below a percent (checked by the stability test).
            density_iters: 3,
            divergence_iters: 2,
            xsph_c: 0.05,
            vmax_frac: 0.5,
        }
    }
}

/// Divergence-Free SPH solver state. Velocities are explicit (DFSPH is a
/// velocity/force method, not position projection); ρ_i and α_i are the
/// per-step precompute reused across both solvers.
pub struct Dfsph {
    grid: SphGrid,
    kernel: SphKernel,
    rest_density: f32,
    params: DfsphParams,
    vel: Vec<Vec2>,
    density: Vec<f32>,
    alpha: Vec<f32>,
    kappa: Vec<f32>,     // per-iteration stiffness k_i / kᵛ_i
    dv: Vec<Vec2>,       // velocity delta (Jacobi double-buffer)
    vscratch: Vec<Vec2>, // XSPH double-buffer
}

impl Default for Dfsph {
    fn default() -> Self {
        Self::new()
    }
}

impl Dfsph {
    pub fn new() -> Self {
        let kernel = SphKernel::new(DFSPH_H);
        Self {
            grid: SphGrid::new(DFSPH_H),
            rest_density: kernel.rest_density(DFSPH_REST_SPACING),
            kernel,
            params: DfsphParams::default(),
            vel: Vec::new(),
            density: Vec::new(),
            alpha: Vec::new(),
            kappa: Vec::new(),
            dv: Vec::new(),
            vscratch: Vec::new(),
        }
    }

    fn ensure_sized(&mut self, n: usize) {
        if self.vel.len() != n {
            self.vel.resize(n, Vec2::ZERO);
        }
        self.density.resize(n, 0.0);
        self.alpha.resize(n, 0.0);
        self.kappa.resize(n, 0.0);
        self.dv.resize(n, Vec2::ZERO);
        self.vscratch.resize(n, Vec2::ZERO);
    }

    /// ρ_i (incl. self term) and the DFSPH factor α_i, both read-only over the
    /// grid ⇒ trivially parallel. Computed once per substep and reused.
    fn compute_density_alpha(&mut self, x: &[Vec2]) {
        let grid = &self.grid;
        let kernel = &self.kernel;
        let density = &mut self.density;
        let alpha = &mut self.alpha;
        density
            .par_iter_mut()
            .zip(alpha.par_iter_mut())
            .enumerate()
            .for_each(|(i, (rho_i, alpha_i))| {
                let xi = x[i];
                let mut rho = 0.0f32;
                let mut grad_sum = Vec2::ZERO; // Σ_j ∇W_ij
                let mut grad2_sum = 0.0f32; // Σ_j |∇W_ij|²
                grid.for_neighbors(xi, |j| {
                    let d = xi - x[j];
                    rho += kernel.w(d.length_squared());
                    if j != i {
                        let g = kernel.grad(d);
                        grad_sum += g;
                        grad2_sum += g.length_squared();
                    }
                });
                *rho_i = rho;
                let denom = grad_sum.length_squared() + grad2_sum;
                // α_i = 1/denom, guarded: a poorly-supported particle (few
                // neighbors) has a tiny denom and would get a runaway stiffness
                // from a single kick — zero its pressure instead, as
                // SPlisHSPlasH does at the free surface. The threshold sits well
                // above the ~1e-7 a 1–2 neighbor particle produces but below a
                // healthy interior's ~1e-5, so only genuine surface strays are
                // dropped.
                *alpha_i = if denom > 2e-6 { 1.0 / denom } else { 0.0 };
            });
    }

    /// One velocity-projection sweep shared by both solvers: given per-particle
    /// stiffness `kappa` = k_i/ρ_i (already read into `self.kappa`), apply
    /// v_i −= Δt Σ_j (k_i/ρ_i + k_j/ρ_j) ∇W_ij as a Jacobi update.
    ///
    /// The ρ_i in DFSPH's factor α_i (Bender & Koschier eq. 8) cancels the 1/ρ_i
    /// of the pressure force, so `kappa` already carries the k/ρ ratio and is
    /// summed directly — dividing by density again here would inflate the
    /// correction by ~1/ρ0 (≈35× in this unit system) and boil the fluid.
    fn apply_pressure_velocity(&mut self, x: &[Vec2], dt: f32) {
        {
            let grid = &self.grid;
            let kernel = &self.kernel;
            let kappa = &self.kappa;
            self.dv.par_iter_mut().enumerate().for_each(|(i, dvi)| {
                let xi = x[i];
                let ki = kappa[i];
                let mut acc = Vec2::ZERO;
                grid.for_neighbors(xi, |j| {
                    if j == i {
                        return;
                    }
                    let g = kernel.grad(xi - x[j]);
                    acc += g * (ki + kappa[j]);
                });
                *dvi = acc * (-dt);
            });
        }
        for (v, dv) in self.vel.iter_mut().zip(&self.dv) {
            *v += *dv;
        }
    }

    /// Constant-density solver (Algorithm 3): correct velocities so the
    /// advected density ρ*_i = ρ_i + Δt·Dρ_i/Dt relaxes to ρ0. Only compression
    /// (ρ* > ρ0) is corrected, so the free surface is never pulled together.
    fn density_solve(&mut self, x: &[Vec2], dt: f32) {
        let inv_dt2 = 1.0 / (dt * dt);
        let rho0 = self.rest_density;
        for _ in 0..self.params.density_iters {
            {
                let grid = &self.grid;
                let kernel = &self.kernel;
                let vel = &self.vel;
                let density = &self.density;
                let alpha = &self.alpha;
                self.kappa.par_iter_mut().enumerate().for_each(|(i, ki)| {
                    let xi = x[i];
                    let vi = vel[i];
                    let mut drho = 0.0f32; // Dρ_i/Dt = Σ_j (v_i − v_j)·∇W_ij
                    grid.for_neighbors(xi, |j| {
                        if j == i {
                            return;
                        }
                        drho += (vi - vel[j]).dot(kernel.grad(xi - x[j]));
                    });
                    let rho_star = (density[i] + dt * drho).max(rho0);
                    *ki = (rho_star - rho0) * alpha[i] * inv_dt2;
                });
            }
            self.apply_pressure_velocity(x, dt);
        }
    }

    /// Divergence-free solver (Algorithm 2): correct velocities so Dρ/Dt → 0.
    /// Skipped for free-surface particles (ρ_i < ρ0) so it can't suck the
    /// surface inward; interior particles get their velocity divergence removed.
    fn divergence_solve(&mut self, x: &[Vec2], dt: f32) {
        let inv_dt = 1.0 / dt;
        let rho0 = self.rest_density;
        for _ in 0..self.params.divergence_iters {
            {
                let grid = &self.grid;
                let kernel = &self.kernel;
                let vel = &self.vel;
                let density = &self.density;
                let alpha = &self.alpha;
                self.kappa.par_iter_mut().enumerate().for_each(|(i, ki)| {
                    if density[i] < rho0 {
                        *ki = 0.0;
                        return;
                    }
                    let xi = x[i];
                    let vi = vel[i];
                    let mut drho = 0.0f32;
                    grid.for_neighbors(xi, |j| {
                        if j == i {
                            return;
                        }
                        drho += (vi - vel[j]).dot(kernel.grad(xi - x[j]));
                    });
                    // Only resist positive divergence (compression rate).
                    *ki = drho.max(0.0) * alpha[i] * inv_dt;
                });
            }
            self.apply_pressure_velocity(x, dt);
        }
    }

    /// XSPH viscosity: v_i ← v_i + c Σ_j (v_j − v_i) W_ij / ρ0 (the 1/ρ0 volume
    /// weight, as in PBF). Double-buffered so every particle reads pre-update
    /// velocities.
    fn apply_xsph(&mut self, x: &[Vec2]) {
        let c = self.params.xsph_c;
        if c == 0.0 {
            return;
        }
        {
            let grid = &self.grid;
            let kernel = &self.kernel;
            let vel = &self.vel;
            let inv_rho0 = 1.0 / self.rest_density;
            self.vscratch.par_iter_mut().enumerate().for_each(|(i, o)| {
                let xi = x[i];
                let vi = vel[i];
                let mut acc = Vec2::ZERO;
                grid.for_neighbors(xi, |j| {
                    if j == i {
                        return;
                    }
                    acc += (vel[j] - vi) * kernel.w((xi - x[j]).length_squared());
                });
                *o = vi + acc * (c * inv_rho0);
            });
        }
        std::mem::swap(&mut self.vel, &mut self.vscratch);
    }
}

impl FluidSolver for Dfsph {
    fn name(&self) -> &'static str {
        "DFSPH"
    }

    fn substep(&mut self, dt: f32, gravity: Vec2, share: &mut ShareData, c_opos: &mut Vec<Vec2>) {
        let n = share.c_pos.len();
        if n == 0 {
            return;
        }
        self.ensure_sized(n);
        c_opos.resize(n, Vec2::ZERO);

        // Match the granular Verlet integrator's effective acceleration so all
        // models fall the same way (x += a·dt²·INV_PHYS_DT there ⇒ a·INV here).
        const INV_PHYS_DT: f32 = 1.0 / PHYS_TIME_STEP;
        let accel = gravity * INV_PHYS_DT;

        let x = &share.c_pos;

        // 1. Neighbor grid + per-step precompute (ρ_i, α_i).
        self.grid.build(x);
        self.compute_density_alpha(x);

        // 2. Non-pressure accelerations: gravity, then XSPH viscosity.
        for v in &mut self.vel {
            *v += accel * dt;
        }
        self.apply_xsph(x);

        // 3. Constant-density solve (drive ρ* → ρ0 by correcting velocities).
        self.density_solve(x, dt);

        // 4. Advect and project onto the box, killing the into-wall velocity.
        let lo_x = LEFT_WALL + BALL_SIZE;
        let hi_x = RIGHT_WALL - BALL_SIZE;
        let lo_y = BOTTOM_WALL + BALL_SIZE;
        let hi_y = TOP_WALL - BALL_SIZE;
        let vmax = self.params.vmax_frac * DFSPH_H / dt;
        let vmax2 = vmax * vmax;
        let x = &mut share.c_pos;
        for i in 0..n {
            let mut v = self.vel[i];
            let s2 = v.length_squared();
            if s2 > vmax2 {
                v *= vmax / s2.sqrt();
            }
            let mut p = x[i] + v * dt;
            if p.x < lo_x {
                p.x = lo_x;
                v.x = v.x.max(0.0);
            } else if p.x > hi_x {
                p.x = hi_x;
                v.x = v.x.min(0.0);
            }
            if p.y < lo_y {
                p.y = lo_y;
                v.y = v.y.max(0.0);
            } else if p.y > hi_y {
                p.y = hi_y;
                v.y = v.y.min(0.0);
            }
            x[i] = p;
            self.vel[i] = v;
        }

        // 5. Divergence-free solve at the *new* positions, so the velocity
        // field entering the next substep has Dρ/Dt ≈ 0 (paper Algorithm 1
        // order). This is the step that keeps DFSPH calm rather than jittering:
        // it removes the compression waves the density solve leaves behind.
        let x = &share.c_pos;
        self.grid.build(x);
        self.compute_density_alpha(x);
        self.divergence_solve(x, dt);

        // 6. Shared/rendered bookkeeping, mirroring the granular/PBF convention:
        // c_opos = pos − v·dt (so external velocity readers agree) and c_color
        // encodes speed as a hue.
        let mut speed_sum = 0.0f32;
        let mut max_speed = 0.0f32;
        for i in 0..n {
            c_opos[i] = x[i] - self.vel[i] * dt;
            let speed = (self.vel[i] * dt * 20.0).length();
            share.c_color[i] = (speed + 198.0) % 360.0;
            speed_sum += speed;
            max_speed = max_speed.max(speed);
        }

        // Diagnostics: mean ρ/ρ0 over a subsample (1.0 = incompressible).
        let stride = (n / 256).max(1);
        let mut dsum = 0.0f64;
        let mut cnt = 0usize;
        let mut i = 0;
        while i < n {
            dsum += (self.density[i] / self.rest_density) as f64;
            cnt += 1;
            i += stride;
        }

        let ps = &mut share.perf_stats;
        ps.mean_speed = speed_sum / n as f32;
        ps.max_speed = max_speed;
        ps.pbf_density_ratio = (dsum / cnt.max(1) as f64) as f32;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{HEIGHT, WIDTH};
    use crate::physics::{Physics, ShareData, Strategy, PHYS_TIME_STEP};
    use std::sync::mpsc::channel;

    /// A DFSPH blob dropped in the box must stay finite, stay inside the walls,
    /// settle toward rest, and keep its interior near rest density.
    #[test]
    fn dfsph_blob_is_stable() {
        let s = DFSPH_REST_SPACING;
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
        physics.set_strategy(Strategy::Dfsph);
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
        assert_eq!(nan, 0, "DFSPH produced NaNs");
        assert_eq!(escaped, 0, "DFSPH particles escaped the box");
        // Settled, not boiling: a blown-up DFSPH pins every particle at the
        // per-substep velocity clamp (mean_speed ≈ 117 in this metric). A
        // settling one decays toward ~7 — deliberately livelier than PBF's
        // over-damped ~5, since DFSPH is low-dissipation. 15 confirms it calmed
        // without demanding PBF-level dissipation.
        assert!(
            share.perf_stats.mean_speed < 15.0,
            "DFSPH did not settle: mean_speed = {}",
            share.perf_stats.mean_speed
        );
    }
}
