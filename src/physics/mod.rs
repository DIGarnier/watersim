//! Particle-fluid engine, organized as a **strategy pattern**: the numerical
//! method is chosen once at startup and kept for the run.
//!
//! [`Physics`] is a thin coordinator. It owns the state shared across every
//! method — the particle positions/colors ([`ShareData`]), the Størmer–Verlet
//! "previous position" array (`c_opos`, which also encodes velocity), gravity,
//! the substep count, and the adaptive-timestep controller — and delegates the
//! actual per-substep advance to a single boxed [`FluidSolver`]. Because the
//! strategy is selected once and never swapped per step, the one dynamic call
//! per substep is free relative to the O(n·neighbors) work it wraps.
//!
//! Each concrete strategy lives in its own module and keeps *its own* scratch
//! (grids, neighbor lists, velocities, deformation gradients, …) rather than
//! bundling every method's data into one struct:
//!
//! - [`granular`] — short-range 1/r² repulsion + hard non-penetration contact
//!   (piles/stacks like sand); the historical, heavily-optimized engine.
//! - [`pbf`] — Position Based Fluids density constraint (pours/splashes as an
//!   incompressible liquid).
//!
//! See `docs/solvers.md` for the survey of these and the methods being added.

mod dfsph;
mod granular;
mod mlsmpm;
mod pbf;
mod sph;

use glam::Vec2;
use std::sync::mpsc::Receiver;

use crate::constants::{BALL_SIZE, HEIGHT, INITIAL_BALL_SPEED_MODIFIER, WIDTH};

pub use dfsph::{Dfsph, DfsphParams};
pub use granular::GranularSolver;
pub use mlsmpm::{Mlsmpm, MpmMaterial, MpmParams};
pub use pbf::{Pbf, PbfParams};

const GRAVITY: Vec2 = Vec2::new(0.0, 9.8);
pub const PHYS_TIME_STEP: f32 = 1.0 / 480.0;

const ADAPTIVE_DT_MIN: f32 = PHYS_TIME_STEP * 0.1;
const ADAPTIVE_DT_MAX: f32 = PHYS_TIME_STEP * 2.0;
const MAX_SAFE_VELOCITY: f32 = 100.0; // Reduce dt when velocities exceed this

pub(crate) const LEFT_WALL: f32 = 0.;
pub(crate) const RIGHT_WALL: f32 = WIDTH;
pub(crate) const BOTTOM_WALL: f32 = 0.;
pub(crate) const TOP_WALL: f32 = HEIGHT;

/// Which fluid model the engine runs. Selected once (via `--sim` on the
/// command line, or [`Physics::set_strategy`]) and kept for the whole run.
///
/// `Granular` is the historical model: a short-range 1/r² repulsion field plus
/// a hard non-penetration contact projection — it stacks and piles like a
/// granular material (sand/balls). `Pbf` is Position Based Fluids (Macklin &
/// Müller, SIGGRAPH 2013): a single *density* constraint solved by a Jacobi
/// position-projection, so the particles behave as an incompressible liquid —
/// they pour, splash, and slosh. See `docs/solvers.md` and `docs/literature.md`
/// §§8–9.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Strategy {
    #[default]
    Granular,
    Pbf,
    /// Divergence-Free SPH (Bender & Koschier): crisp, low-dissipation
    /// incompressible water. See [`dfsph`].
    Dfsph,
    /// MLS-MPM (Hu et al.): hybrid grid+particle method; fluid by default,
    /// elastic jelly with a swapped constitutive model. See [`mlsmpm`].
    Mlsmpm,
}

impl Strategy {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "granular" | "balls" | "repulsion" => Some(Strategy::Granular),
            "pbf" | "fluid" | "water" => Some(Strategy::Pbf),
            "dfsph" | "divergence-free" => Some(Strategy::Dfsph),
            "mlsmpm" | "mpm" | "jelly" => Some(Strategy::Mlsmpm),
            _ => None,
        }
    }

    /// Every selectable strategy, for `--list-sims` / help text.
    pub fn all() -> &'static [Strategy] {
        &[
            Strategy::Granular,
            Strategy::Pbf,
            Strategy::Dfsph,
            Strategy::Mlsmpm,
        ]
    }

    /// The canonical `--sim` token for this strategy.
    pub fn token(self) -> &'static str {
        match self {
            Strategy::Granular => "granular",
            Strategy::Pbf => "pbf",
            Strategy::Dfsph => "dfsph",
            Strategy::Mlsmpm => "mlsmpm",
        }
    }

    fn make_solver(self, scale: f32) -> Box<dyn FluidSolver> {
        match self {
            Strategy::Granular => Box::new(GranularSolver::new(scale, Vec::new())),
            Strategy::Pbf => Box::new(Pbf::new()),
            Strategy::Dfsph => Box::new(Dfsph::new()),
            Strategy::Mlsmpm => Box::new(Mlsmpm::new()),
        }
    }
}

pub enum EventToPthread {
    Cannon((Vec2, Vec2)),
    Scale(f32),
    ToggleVerletLists,
    ToggleAdaptiveDt,
}

#[derive(Default, Clone)]
pub struct ShareData {
    pub c_pos: Vec<Vec2>,
    pub c_color: Vec<f32>,
    pub phys_time: f32,
    pub perf_stats: PerformanceStats,
}

// Performance statistics for benchmarking
#[derive(Default, Clone)]
pub struct PerformanceStats {
    pub integration_time_us: u64,
    pub collision_time_us: u64,
    pub neighbor_rebuild_time_us: u64,
    pub force_calc_time_us: u64,
    pub total_particles: usize,
    pub verlet_lists_enabled: bool,
    pub adaptive_dt_enabled: bool,
    pub current_dt: f32,
    // Coarse diagnostics (aggregate "vector field" summaries) so instability
    // is observable without a debugger. `*_speed` are the per-substep
    // displacement metric |Δx|·20 (the same units the color hue uses);
    // `pbf_density_ratio` is the mean ρ/ρ0 over a subsample (1.0 = perfectly
    // incompressible, ≫1 = compressed/exploding, ≪1 = torn apart).
    pub mean_speed: f32,
    pub max_speed: f32,
    pub pbf_density_ratio: f32,
}

/// A fluid-simulation strategy: one numerical method for advancing the
/// particle system by a substep. Concrete implementations own all of their
/// own scratch; the coordinator hands them the shared particle state plus the
/// Verlet "previous position" array each substep.
///
/// The knob setters and diagnostic hooks default to no-ops so [`Physics`] can
/// forward a command to whichever strategy is active without caring which one
/// it is — a strategy simply overrides the knobs it actually has.
pub trait FluidSolver: Send {
    /// Short human-readable name (for the HUD / logs).
    fn name(&self) -> &'static str;

    /// Advance the particle system by one substep of length `dt` under
    /// `gravity`. `share` carries positions/colors/stats; `c_opos` is the
    /// Størmer–Verlet previous-position array (velocity memory) that outlives
    /// any strategy switch.
    fn substep(&mut self, dt: f32, gravity: Vec2, share: &mut ShareData, c_opos: &mut Vec<Vec2>);

    /// Whether Verlet neighbor lists are active (HUD readout; granular only).
    fn verlet_lists_enabled(&self) -> bool {
        false
    }

    /// Adjust the repulsion force scale (granular only).
    fn add_scale(&mut self, _delta: f32) {}
    /// Far-field refresh interval in substeps (granular MTS only).
    fn set_force_interval(&mut self, _interval: usize) {}
    /// Contact/constraint iterations per substep (granular only).
    fn set_solver_iterations(&mut self, _iterations: usize) {}
    /// Over-relaxation factor (granular only).
    fn set_solver_omega(&mut self, _omega: f32) {}
    /// Serial→parallel crossover particle count (granular only).
    fn set_par_min_particles(&mut self, _par_min: usize) {}
    /// Toggle Verlet neighbor lists (granular only).
    fn toggle_verlet_lists(&mut self) {}
    /// Override the density-solver coefficients (PBF only).
    fn set_pbf_params(&mut self, _params: PbfParams) {}
    /// Override the MLS-MPM coefficients / material (MPM only).
    fn set_mpm_params(&mut self, _params: MpmParams) {}

    /// Bench/test hook: forces via the grid stencil (granular only; empty
    /// otherwise). See [`GranularSolver`].
    fn forces_grid(&mut self, _positions: &[Vec2]) -> Vec<Vec2> {
        Vec::new()
    }
    /// Bench/test hook: exact O(n²) forces (granular only; empty otherwise).
    fn forces_direct(&self, _positions: &[Vec2]) -> Vec<Vec2> {
        Vec::new()
    }
}

/// Coordinator over the selected [`FluidSolver`]. Owns the state shared across
/// strategies and the adaptive-timestep controller; delegates each substep to
/// the boxed strategy.
pub struct Physics {
    /// Størmer–Verlet previous position (also encodes velocity). Shared: the
    /// cannon writes it and it survives a strategy switch, so it lives here.
    c_opos: Vec<Vec2>,
    pub rx: Receiver<EventToPthread>,

    solver: Box<dyn FluidSolver>,
    strategy: Strategy,
    /// Seed force scale, kept so a strategy switch can rebuild a solver with it.
    scale: f32,

    // Per-substep gravity (settable so a scenario can tilt the box to make the
    // water slosh sideways) and the Small-Steps substep count.
    gravity: Vec2,
    substeps: usize,

    // Adaptive time-stepping controller (a coordinator-level concern: it scales
    // the dt handed to whichever strategy is active).
    use_adaptive_dt: bool,
    adaptive_dt: f32,
    last_max_velocity: f32,
}

impl Physics {
    pub fn new(
        c_opos: Vec<Vec2>,
        c_force: Vec<Vec2>,
        rx: Receiver<EventToPthread>,
        scale: f32,
    ) -> Self {
        Self {
            c_opos,
            rx,
            solver: Box::new(GranularSolver::new(scale, c_force)),
            strategy: Strategy::default(),
            scale,
            gravity: GRAVITY,
            substeps: 1,
            use_adaptive_dt: true, // Enable adaptive time-stepping by default
            adaptive_dt: PHYS_TIME_STEP,
            last_max_velocity: 0.0,
        }
    }

    /// The active strategy.
    pub fn strategy(&self) -> Strategy {
        self.strategy
    }

    /// Select the fluid model (default `Granular`). Builds a fresh solver, so a
    /// switch mid-run starts that method's solver from the current positions.
    pub fn set_strategy(&mut self, strategy: Strategy) {
        self.strategy = strategy;
        self.solver = strategy.make_solver(self.scale);
    }

    /// Per-substep gravity vector. Scenarios use this to tilt the box.
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.gravity = gravity;
    }

    /// Adjust the repulsion force scale (granular; ignored by other models).
    pub fn add_scale(&mut self, delta: f32) {
        self.scale += delta;
        self.solver.add_scale(delta);
    }

    /// Override the PBF coefficients (used by the render tool to sweep them).
    pub fn set_pbf_params(&mut self, params: PbfParams) {
        self.solver.set_pbf_params(params);
    }

    /// Override the MLS-MPM coefficients / material.
    pub fn set_mpm_params(&mut self, params: MpmParams) {
        self.solver.set_mpm_params(params);
    }

    /// Far-field force refresh interval in substeps (1 = every substep).
    pub fn set_force_interval(&mut self, interval: usize) {
        self.solver.set_force_interval(interval);
    }

    /// Contact-solver iterations per substep.
    pub fn set_solver_iterations(&mut self, iterations: usize) {
        self.solver.set_solver_iterations(iterations);
    }

    /// Contact-solver over-relaxation factor (1.0 = historical behavior).
    pub fn set_solver_omega(&mut self, omega: f32) {
        self.solver.set_solver_omega(omega);
    }

    /// Particle count at which the granular grid paths switch from the serial
    /// Gauss-Seidel engine to the packed parallel Jacobi engine
    /// (0 = always packed, usize::MAX = always serial).
    pub fn set_par_min_particles(&mut self, par_min: usize) {
        self.solver.set_par_min_particles(par_min);
    }

    /// Internal substeps per `step()` call (Small Steps): each runs the full
    /// solver pipeline at dt/substeps. Pair with proportionally fewer solver
    /// iterations and a scaled `force_interval` to hold the far-field refresh
    /// rate constant.
    pub fn set_substeps(&mut self, substeps: usize) {
        self.substeps = substeps.max(1);
    }

    /// Bench/test hook: granular forces via the grid stencil.
    pub fn forces_grid(&mut self, positions: &[Vec2]) -> Vec<Vec2> {
        self.solver.forces_grid(positions)
    }

    /// Bench/test hook: exact O(n²) granular forces.
    pub fn forces_direct(&self, positions: &[Vec2]) -> Vec<Vec2> {
        self.solver.forces_direct(positions)
    }

    pub fn step(&mut self, dt: f32, share: &mut ShareData) {
        // Adaptive time-stepping
        let effective_dt = if self.use_adaptive_dt {
            self.compute_adaptive_dt()
        } else {
            dt
        };

        // Phase timings accumulate across the internal substeps.
        let ps = &mut share.perf_stats;
        ps.integration_time_us = 0;
        ps.neighbor_rebuild_time_us = 0;
        ps.force_calc_time_us = 0;
        ps.collision_time_us = 0;

        // Small Steps (Macklin et al. 2019, stage 22): splitting the step
        // into substeps with proportionally fewer solver iterations is more
        // accurate than iterating; requires the a·dt² integrator (stage 21).
        let s = self.substeps.max(1);
        for _ in 0..s {
            self.solver.substep(
                effective_dt / s as f32,
                self.gravity,
                share,
                &mut self.c_opos,
            );
        }

        // Adaptive dt reads the last substep's peak speed (both models fill
        // perf_stats.max_speed in the shared |Δx|·20 units).
        self.last_max_velocity = share.perf_stats.max_speed;

        // Update performance stats
        share.perf_stats.total_particles = share.c_pos.len();
        share.perf_stats.verlet_lists_enabled = self.solver.verlet_lists_enabled();
        share.perf_stats.adaptive_dt_enabled = self.use_adaptive_dt;
        share.perf_stats.current_dt = effective_dt;
    }

    fn compute_adaptive_dt(&mut self) -> f32 {
        // Adjust dt based on maximum velocity to prevent instabilities
        if self.last_max_velocity > MAX_SAFE_VELOCITY {
            self.adaptive_dt = (self.adaptive_dt * 0.9).max(ADAPTIVE_DT_MIN);
        } else if self.last_max_velocity < MAX_SAFE_VELOCITY * 0.5 {
            self.adaptive_dt = (self.adaptive_dt * 1.05).min(ADAPTIVE_DT_MAX);
        }
        self.adaptive_dt
    }

    pub fn do_cannon(&mut self, dt: f32, share: &mut ShareData, start: Vec2, cannon: Vec2) {
        for k in 0..20 {
            self.cannon(-k as f32 * (2.2 * BALL_SIZE), 0., dt, share, start, cannon);
        }
    }

    pub fn toggle_verlet_lists(&mut self) {
        self.solver.toggle_verlet_lists();
    }

    pub fn toggle_adaptive_dt(&mut self) {
        self.use_adaptive_dt = !self.use_adaptive_dt;
        println!(
            "Adaptive dt: {}",
            if self.use_adaptive_dt { "ON" } else { "OFF" }
        );
    }

    /// Explicitly enable/disable adaptive time-stepping. The live app disables
    /// it and drives a fixed-timestep accumulator instead: the Størmer–Verlet
    /// integrator assumes a constant dt, and changing dt each step reinterprets
    /// the encoded velocity `(x − x_prev)`, injecting energy — which is what
    /// made the granular sim "breathe" (float, then drop) under the adaptive
    /// controller's limit cycle.
    pub fn set_adaptive_dt(&mut self, on: bool) {
        self.use_adaptive_dt = on;
    }

    fn cannon(
        &mut self,
        shift: f32,
        color: f32,
        dt: f32,
        share: &mut ShareData,
        start: Vec2,
        cannon: Vec2,
    ) {
        let ball_pos = start + cannon.perp().normalize() * shift;
        let speed = cannon * INITIAL_BALL_SPEED_MODIFIER * dt;
        let ball_opos = ball_pos - speed;

        share.c_pos.push(ball_pos);
        self.c_opos.push(ball_opos);
        share.c_color.push(color);
    }
}

/// Hardware reciprocal square root (SSE `rsqrtss`, ~12-bit) refined with one
/// Newton-Raphson step to ~22 bits — plenty for force directions and contact
/// normals, and much cheaper than `sqrt` + `div`. Callers must keep x > 0.
#[inline(always)]
pub(crate) fn fast_rsqrt(x: f32) -> f32 {
    #[cfg(target_arch = "x86_64")]
    {
        use core::arch::x86_64::{_mm_cvtss_f32, _mm_rsqrt_ss, _mm_set_ss};
        let y = unsafe { _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set_ss(x))) };
        y * (1.5 - 0.5 * x * y * y)
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        x.sqrt().recip()
    }
}

/// Keep a position inside the box by one ball radius, matching the granular
/// wall inset. Pure position clamp (SPH-family models handle the bounce via
/// the velocity update, which sees the clamped displacement).
#[inline(always)]
pub(crate) fn clamp_wall(p: &mut Vec2) {
    p.x = p.x.clamp(LEFT_WALL + BALL_SIZE, RIGHT_WALL - BALL_SIZE);
    p.y = p.y.clamp(BOTTOM_WALL + BALL_SIZE, TOP_WALL - BALL_SIZE);
}

enum Collision {
    Top,
    Bottom,
    Left,
    Right,
}

fn collides_wall(pos_a: Vec2, scale_a: f32) -> (Option<Collision>, Option<Collision>) {
    let mut vert = None;
    if pos_a.y - scale_a <= BOTTOM_WALL {
        vert = Some(Collision::Bottom);
    } else if pos_a.y + scale_a >= TOP_WALL {
        vert = Some(Collision::Top);
    }

    let mut hor = None;
    if pos_a.x - scale_a <= LEFT_WALL {
        hor = Some(Collision::Left);
    } else if pos_a.x + scale_a >= RIGHT_WALL {
        hor = Some(Collision::Right);
    }

    (hor, vert)
}

/// `salt` is a tiny per-particle offset added to the clamp coordinate.
/// Without it, every particle pushed past the same wall corner in the same
/// substep clamps onto the *identical* (x, y) — exactly-coincident pairs,
/// which the packed solver's degenerate mask can never separate. Sub-µm
/// magnitude: invisible, but breaks exact ties deterministically.
pub(crate) fn resolve_wall_collision(c_pos: &mut Vec2, c_opos: &mut Vec2, salt: f32) {
    let (hor, ver) = collides_wall(*c_pos, BALL_SIZE);

    let curr_vel = (*c_pos - *c_opos) * 0.4;
    const EPS: f32 = 0.00001;
    use Collision::*;
    match hor {
        Some(Left) => {
            c_pos.x = LEFT_WALL + BALL_SIZE + EPS + salt;
            c_opos.x = c_pos.x + curr_vel.x;
        }
        Some(Right) => {
            c_pos.x = RIGHT_WALL - BALL_SIZE - EPS - salt;
            c_opos.x = c_pos.x + curr_vel.x;
        }
        _ => {}
    }

    match ver {
        Some(Bottom) => {
            c_pos.y = BOTTOM_WALL + BALL_SIZE + EPS + salt;
            c_opos.y = c_pos.y + curr_vel.y;
        }
        Some(Top) => {
            c_pos.y = TOP_WALL - BALL_SIZE - EPS - salt;
            c_opos.y = c_pos.y + curr_vel.y;
        }
        _ => {}
    }
}

/// Deterministic per-particle wall salt: 0..~0.15 µm spread by particle id.
#[inline(always)]
pub(crate) fn wall_salt(i: usize) -> f32 {
    1e-5 * (i & 15) as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc::channel;

    /// Two exactly-coincident particles must separate to contact distance —
    /// the pre-stage-20 solver left them glued forever.
    #[test]
    fn coincident_pair_separates() {
        let p = Vec2::new(700.0, 600.0);
        let positions = vec![p, p];
        let (_tx, rx) = channel();
        let mut physics = Physics::new(positions.clone(), vec![Vec2::ZERO; 2], rx, 2000.0);
        physics.toggle_adaptive_dt();
        let mut share = ShareData {
            c_pos: positions,
            c_color: vec![0.0; 2],
            ..Default::default()
        };
        for _ in 0..20 {
            physics.step(PHYS_TIME_STEP, &mut share);
        }
        let dist = (share.c_pos[0] - share.c_pos[1]).length();
        assert!(
            dist > 1.5 * BALL_SIZE,
            "coincident pair still glued: dist = {dist}"
        );
    }
}
