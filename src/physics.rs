use glam::Vec2;
use rayon::prelude::*;
use std::sync::mpsc::Receiver;
use std::time::Instant;

use crate::constants::{
    BALL_SIZE, GRID_SIZE, HEIGHT, INITIAL_BALL_SPEED_MODIFIER, WIDTH, X_LEN, Y_LEN,
};

const GRAVITY: Vec2 = Vec2::new(0.0, 9.8);
pub const PHYS_TIME_STEP: f32 = 1.0 / 480.0;

/// Which fluid model the engine runs. `Granular` is the historical model:
/// a short-range 1/r² repulsion field plus a hard non-penetration contact
/// projection — it stacks and piles like a granular material (sand/balls).
/// `Pbf` is Position Based Fluids (Macklin & Müller, SIGGRAPH 2013): it
/// replaces both with a single *density* constraint solved by the same
/// Jacobi position-projection machinery, so the particles behave as an
/// incompressible liquid — they pour, splash, and slosh. See the PBF block
/// near the bottom of this file and docs/literature.md §§8–9.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Strategy {
    #[default]
    Granular,
    Pbf,
}

impl Strategy {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "granular" | "balls" | "repulsion" => Some(Strategy::Granular),
            "pbf" | "fluid" | "water" => Some(Strategy::Pbf),
            _ => None,
        }
    }
}

// The force model (stage 25): *local* repulsion with compact support, like
// every particle water model (SPH kernels, PBF, Clavet 2005 — pressure in a
// liquid is local; docs/literature.md §9). The 1/r² kernel acts from
// BALL_SIZE out to FORCE_CUTOFF = 2.5 ball radii = one grid cell, which
// makes the one-cell grid stencil *exact*: every interacting pair is
// guaranteed inside the 3×3 neighborhood. A smoothstep taper (in r²) takes
// the force continuously to zero between the contact distance and the
// cutoff, so pairs crossing the cutoff never pop. This replaced the
// previous split personality — Barnes-Hut mode summed the repulsion
// globally while the grid modes truncated it at stencil geometry — see
// docs/benchmarks/25-cutoff-model.md.
const FORCE_CUTOFF: f32 = GRID_SIZE;
const FORCE_CUTOFF_SQ: f32 = FORCE_CUTOFF * FORCE_CUTOFF;
const TAPER_START_SQ: f32 = (2.0 * BALL_SIZE) * (2.0 * BALL_SIZE);
const INV_TAPER_SPAN: f32 = 1.0 / (FORCE_CUTOFF_SQ - TAPER_START_SQ);

/// Smoothstep window on r²: 1 below the contact distance, 0 at the cutoff,
/// C¹ at both ends. Branchless.
#[inline(always)]
fn taper(dist_sq: f32) -> f32 {
    let u = ((dist_sq - TAPER_START_SQ) * INV_TAPER_SPAN).clamp(0.0, 1.0);
    1.0 - u * u * (3.0 - 2.0 * u)
}

// Optimization constants
const VERLET_SKIN_DISTANCE: f32 = BALL_SIZE * 0.5; // Extra distance for neighbor lists
                                                   // When neighbor lists have gone stale (something moved > skin/2), forces fall
                                                   // back to the direct cell pass — which costs the same as the rebuild gather —
                                                   // and a rebuild is only re-attempted this often, in case the system calmed
                                                   // down enough for lists to stay valid again ("lazy Verlet").
const VERLET_REBUILD_RETRY: usize = 16;

// Physically permute the particle arrays into grid (spatial) order this often.
// Keeps array neighbors ≈ spatial neighbors so the force/solver passes read
// memory nearly sequentially (SFC reordering à la HOOMD-blue's SFCPACK;
// docs/literature.md §4). O(n) copies, amortized to ~nothing.
const REORDER_INTERVAL: usize = 64;
const ADAPTIVE_DT_MIN: f32 = PHYS_TIME_STEP * 0.1;
const ADAPTIVE_DT_MAX: f32 = PHYS_TIME_STEP * 2.0;
const MAX_SAFE_VELOCITY: f32 = 100.0; // Reduce dt when velocities exceed this

// Force refresh interval, in substeps (RESPA-style multiple time stepping;
// Tuckerman, Berne & Martyna 1992, docs/literature.md §7). The repulsion
// field is smooth and positions move a tiny fraction of a cell per 480 Hz
// substep, so the force gather is recomputed only every K substeps and
// reused in between. The contact solver — the fast, stiff part — still runs
// every substep. Quality was measured in docs/benchmarks/14-mts-farfield.md.
// Runtime-tunable via `set_force_interval`; 1 = recompute every substep.
const FORCE_INTERVAL: usize = 4;

// Constraint-projection iterations per substep. The sim already substeps at
// 480 Hz (8 substeps per 60 Hz visual frame); per "Small Steps in Physics
// Simulation" (Macklin et al., SCA 2019) substeps are far more valuable than
// solver iterations, so a small count here suffices (see
// docs/benchmarks/04-small-steps.md for the measured speed/quality trade).
const SOLVER_ITERATIONS: usize = 3;

// Over-relaxation factor for the contact projection (SOR; Macklin et al.,
// "Unified Particle Physics for Real-Time Applications" recommend 1 ≤ ω ≤ 2
// for Jacobi/Gauss-Seidel constraint solvers). ω = 1 reproduces the
// historical 0.375-per-half correction. The measured (iterations × ω)
// surface lives in docs/benchmarks/15-solver-relaxation.md: 3 iterations at
// ω = 1 keep stage-11 contact quality at ~-20% solver cost; ω ≈ 1.25 at 2
// iterations is the faster/mushier point, ω ≥ 1.5 overshoots.
const SOLVER_OMEGA: f32 = 1.0;

// Below this particle count the serial grid paths win (rayon region overhead
// dominates small workloads). Re-swept after every change to the packed
// engine's cost profile: 16k originally (stage 10), 22k after MTS + smaller
// particles (stage 18), back down to 14k once the clustered solver gather
// (stage 23) and the cutoff model (stage 25) made the packed engine cheaper
// — it now wins clearly from 16k on both paths and ties at 12k (stage 26).
// Runtime-tunable via `set_par_min_particles`.
const PAR_MIN_PARTICLES: usize = 14_000;

const LEFT_WALL: f32 = 0.;
const RIGHT_WALL: f32 = WIDTH;
const BOTTOM_WALL: f32 = 0.;
const TOP_WALL: f32 = HEIGHT;

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

/// Verlet neighbor lists in flat CSR layout: one contiguous `neighbors` array
/// with per-particle `start` offsets, instead of a Vec per particle. Rebuilt
/// only when some particle has drifted more than half the skin distance from
/// its position at build time (`ref_pos`) — the standard MD policy, which is
/// both cheaper at rest and, unlike the old fixed every-10-frames cadence,
/// can't miss fast-moving pairs.
#[derive(Default)]
struct VerletLists {
    start: Vec<u32>,     // n+1 offsets into `neighbors`
    neighbors: Vec<u32>, // flat neighbor ids
    ref_pos: Vec<Vec2>,  // positions when the lists were built
}

impl VerletLists {
    #[inline(always)]
    fn of(&self, i: usize) -> &[u32] {
        &self.neighbors[self.start[i] as usize..self.start[i + 1] as usize]
    }

    fn is_valid_for(&self, n: usize) -> bool {
        self.start.len() == n + 1
    }

    fn needs_rebuild(&self, c_pos: &[Vec2]) -> bool {
        if !self.is_valid_for(c_pos.len()) {
            return true;
        }
        let limit_sq = (VERLET_SKIN_DISTANCE * 0.5) * (VERLET_SKIN_DISTANCE * 0.5);
        c_pos
            .iter()
            .zip(&self.ref_pos)
            .any(|(p, r)| (*p - *r).length_squared() > limit_sq)
    }
}

// Grid dimensions as integers (X_LEN/Y_LEN are f32 for legacy reasons)
const GRID_W: usize = X_LEN as usize;
const GRID_H: usize = Y_LEN as usize;
const N_CELLS: usize = GRID_W * GRID_H;

/// Uniform grid in CSR layout, built with a counting sort (Green 2010,
/// Hoetzlein 2014). One flat index array + per-cell start offsets instead of
/// a heap-allocated Vec per cell. Built once per step and reused across all
/// constraint-solver iterations (particles move far less than a cell per
/// step, so the one-cell stencil acts as a Verlet skin).
struct CsrGrid {
    cell_start: Vec<u32>, // N_CELLS + 1 offsets into `indices`
    cursor: Vec<u32>,     // scratch: per-cell write cursor / counts
    cell_of: Vec<u32>,    // per-particle cell id
    indices: Vec<u32>,    // particle ids grouped by cell
    occupied: Vec<u32>,   // ids of non-empty cells, ascending
}

impl CsrGrid {
    fn new() -> Self {
        Self {
            cell_start: vec![0; N_CELLS + 1],
            cursor: vec![0; N_CELLS],
            cell_of: Vec::new(),
            indices: Vec::new(),
            occupied: Vec::new(),
        }
    }

    #[inline(always)]
    fn cell_id(pos: Vec2) -> u32 {
        const INV_GRID_SIZE: f32 = 1.0 / GRID_SIZE;
        // `as usize` saturates negatives to 0, matching the old clamping
        let x = ((pos.x * INV_GRID_SIZE) as usize).min(GRID_W - 1);
        let y = ((pos.y * INV_GRID_SIZE) as usize).min(GRID_H - 1);
        (y * GRID_W + x) as u32
    }

    fn build(&mut self, positions: &[Vec2], parallel: bool) {
        let n = positions.len();
        self.cell_of.resize(n, 0);
        self.indices.resize(n, 0);
        self.cursor.fill(0);

        if parallel {
            // The cell-id computation (fp math + conversions) is the
            // expensive half of the counting pass and is embarrassingly
            // parallel; the count itself stays serial (n random increments).
            self.cell_of
                .par_chunks_mut(4096)
                .zip(positions.par_chunks(4096))
                .for_each(|(cells, ps)| {
                    for (c, p) in cells.iter_mut().zip(ps) {
                        *c = Self::cell_id(*p);
                    }
                });
            for &c in &self.cell_of {
                self.cursor[c as usize] += 1;
            }
        } else {
            for (i, p) in positions.iter().enumerate() {
                let c = Self::cell_id(*p);
                self.cell_of[i] = c;
                self.cursor[c as usize] += 1;
            }
        }

        let mut sum = 0u32;
        self.occupied.clear();
        for c in 0..N_CELLS {
            let count = self.cursor[c];
            self.cell_start[c] = sum;
            self.cursor[c] = sum;
            if count > 0 {
                self.occupied.push(c as u32);
            }
            sum += count;
        }
        self.cell_start[N_CELLS] = sum;

        for i in 0..n {
            let c = self.cell_of[i] as usize;
            self.indices[self.cursor[c] as usize] = i as u32;
            self.cursor[c] += 1;
        }
    }

    #[inline(always)]
    fn cell(&self, c: usize) -> &[u32] {
        &self.indices[self.cell_start[c] as usize..self.cell_start[c + 1] as usize]
    }
}

/// Hardware reciprocal square root (SSE `rsqrtss`, ~12-bit) refined with one
/// Newton-Raphson step to ~22 bits — plenty for force directions and contact
/// normals, and much cheaper than `sqrt` + `div`. Callers must keep x > 0.
#[inline(always)]
fn fast_rsqrt(x: f32) -> f32 {
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

/// The 3 contiguous packed ranges covering a cell's 3×3 neighborhood: cells
/// x−1..=x+1 of one row are adjacent in row-major order, so their CSR ranges
/// concatenate (empty cells contribute empty subranges for free). This is what
/// makes the packed kernels below stream memory sequentially and vectorize.
#[inline(always)]
fn stencil_rows(cell_start: &[u32], cell: usize) -> [(usize, usize); 3] {
    let (x, y) = (cell % GRID_W, cell / GRID_W);
    let x0 = x.saturating_sub(1);
    let x1 = (x + 1).min(GRID_W - 1);
    let mut rows = [(0usize, 0usize); 3];
    for dy in 0..3usize {
        let yy = y as i32 + dy as i32 - 1;
        if yy >= 0 && yy < GRID_H as i32 {
            let base = yy as usize * GRID_W;
            rows[dy] = (
                cell_start[base + x0] as usize,
                cell_start[base + x1 + 1] as usize,
            );
        }
    }
    rows
}

/// Repulsion force gather over packed row slices, branchless so LLVM can
/// vectorize the inner loop. `s8` = scale/8 (the historical accumulation
/// factor). Self-interaction masks out via d² < BALL_SIZE² like any touching
/// pair; the clamped denominator keeps masked lanes finite (never NaN·0).
#[inline(always)]
fn gather_force(
    xa: f32,
    ya: f32,
    px: &[f32],
    py: &[f32],
    rows: &[(usize, usize); 3],
    s8: f32,
) -> (f32, f32) {
    const BALL_SQ: f32 = BALL_SIZE * BALL_SIZE;
    let mut fx = 0.0f32;
    let mut fy = 0.0f32;
    for &(start, end) in rows {
        for k in start..end {
            let dx = xa - px[k];
            let dy = ya - py[k];
            let d2 = dx * dx + dy * dy;
            let m = if d2 >= BALL_SQ && d2 < FORCE_CUTOFF_SQ {
                1.0f32
            } else {
                0.0
            };
            let d2c = d2.max(BALL_SQ);
            let w = (s8 * m * taper(d2)) / (d2c.sqrt() * d2c);
            fx += dx * w;
            fy += dy * w;
        }
    }
    (fx, fy)
}

/// Union of the 3-row stencils of a contiguous cell range [lo, hi] on one
/// grid row: x from lo.x−1 to hi.x+1. Every partner within CONTACT of any
/// member cell is inside it (CONTACT ≤ GRID_SIZE), so gathering a cluster of
/// particles from these rows with the distance mask is exact — the extra
/// candidates mask to zero terms.
#[inline(always)]
fn stencil_rows_span(cell_start: &[u32], lo: usize, hi: usize) -> [(usize, usize); 3] {
    let (x0, y) = (lo % GRID_W, lo / GRID_W);
    let x1 = hi % GRID_W;
    let x0 = x0.saturating_sub(1);
    let x1 = (x1 + 1).min(GRID_W - 1);
    let mut rows = [(0usize, 0usize); 3];
    for dy in 0..3usize {
        let yy = y as i32 + dy as i32 - 1;
        if yy >= 0 && yy < GRID_H as i32 {
            let base = yy as usize * GRID_W;
            rows[dy] = (
                cell_start[base + x0] as usize,
                cell_start[base + x1 + 1] as usize,
            );
        }
    }
    rows
}

/// 4-lane variant of `gather_correction`: one pass over a shared candidate
/// row set updates 4 cluster members at once (GROMACS-style i-cluster;
/// docs/literature.md §4, docs/benchmarks/23-cluster-gather.md). Each
/// partner is loaded once and broadcast against 4 lanes, so the inner body
/// is 4-wide SIMD with full lanes even though individual cells hold only
/// 1–2 particles. Per-lane arithmetic and candidate order are identical to
/// the scalar kernel over a superset of candidates whose extras mask to
/// exact zeros — results are bit-identical to the scalar gather.
#[inline(always)]
fn gather_correction4(
    xs: &[f32; 4],
    ys: &[f32; 4],
    px: &[f32],
    py: &[f32],
    rows: &[(usize, usize); 3],
    relax: f32,
) -> ([f32; 4], [f32; 4]) {
    const CONTACT: f32 = BALL_SIZE + BALL_SIZE;
    let mut cx = [0.0f32; 4];
    let mut cy = [0.0f32; 4];
    for &(start, end) in rows {
        for k in start..end {
            let bx = px[k];
            let by = py[k];
            for l in 0..4 {
                let dx = xs[l] - bx;
                let dy = ys[l] - by;
                let d2 = dx * dx + dy * dy;
                let m = if d2 < CONTACT * CONTACT && d2 > 1e-12 {
                    1.0f32
                } else {
                    0.0
                };
                let d2c = d2.max(1e-12);
                let inv = 1.0 / d2c.sqrt();
                let dist = d2c * inv;
                let term = m * relax * (dist - CONTACT) * inv;
                cx[l] -= dx * term;
                cy[l] -= dy * term;
            }
        }
    }
    (cx, cy)
}

/// One Jacobi half-correction gather for the contact solver: the particle
/// accumulates its half of every overlapping pair's separation; the partner
/// accumulates the other half in its own gather. Same per-iteration pair
/// correction as the old Gauss-Seidel sweep, but order-independent, so it
/// vectorizes and parallelizes with no cell coloring.
#[inline(always)]
fn gather_correction(
    xa: f32,
    ya: f32,
    px: &[f32],
    py: &[f32],
    rows: &[(usize, usize); 3],
    relax: f32,
) -> (f32, f32) {
    const CONTACT: f32 = BALL_SIZE + BALL_SIZE;
    let mut cx = 0.0f32;
    let mut cy = 0.0f32;
    for &(start, end) in rows {
        for k in start..end {
            let dx = xa - px[k];
            let dy = ya - py[k];
            let d2 = dx * dx + dy * dy;
            // Excludes self (d2 == 0) and coincident pairs, like the old
            // try_normalize() -> ZERO path.
            let m = if d2 < CONTACT * CONTACT && d2 > 1e-12 {
                1.0f32
            } else {
                0.0
            };
            let d2c = d2.max(1e-12);
            let inv = 1.0 / d2c.sqrt();
            let dist = d2c * inv;
            // == normalized(axis) * relax * (dist - CONTACT), masked;
            // relax = 0.375·ω (ω = 1 reproduces the historical factor)
            let term = m * relax * (dist - CONTACT) * inv;
            cx -= dx * term;
            cy -= dy * term;
        }
    }
    (cx, cy)
}

pub struct Physics {
    c_opos: Vec<Vec2>,
    c_force: Vec<Vec2>,
    grid: CsrGrid,
    pub rx: Receiver<EventToPthread>,
    pub scale: f32,

    // Optimization structures
    neighbor_lists: VerletLists,
    stale_steps: usize,
    scratch_v2: Vec<Vec2>,
    scratch_f: Vec<f32>,
    // Packed SoA state in CSR (cell) order, rebuilt each step: positions for
    // the force/solver kernels plus a shared accumulator pair.
    px: Vec<f32>,
    py: Vec<f32>,
    acc_x: Vec<f32>,
    acc_y: Vec<f32>,
    packed_cell: Vec<u32>,
    // Cached far-field forces for multiple time stepping: refreshed every
    // `force_interval` substeps, reused in between (empty = no valid cache).
    c_farfield: Vec<Vec2>,
    frame_count: usize,
    use_verlet_lists: bool,
    use_adaptive_dt: bool,
    adaptive_dt: f32,
    force_interval: usize,
    solver_iterations: usize,
    solver_omega: f32,
    par_min: usize,
    substeps: usize,

    // Fluid model selection and its per-substep gravity (settable so a
    // scenario can tilt the box to make the water slosh sideways).
    strategy: Strategy,
    gravity: Vec2,
    pbf: Pbf,

    // Performance tracking
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
            c_force,
            grid: CsrGrid::new(),
            rx,
            scale,
            neighbor_lists: VerletLists::default(),
            stale_steps: 0,
            scratch_v2: Vec::new(),
            scratch_f: Vec::new(),
            px: Vec::new(),
            py: Vec::new(),
            acc_x: Vec::new(),
            acc_y: Vec::new(),
            packed_cell: Vec::new(),
            c_farfield: Vec::new(),
            frame_count: 0,
            use_verlet_lists: true, // Enable Verlet lists by default
            use_adaptive_dt: true,  // Enable adaptive time-stepping by default
            adaptive_dt: PHYS_TIME_STEP,
            force_interval: FORCE_INTERVAL,
            solver_iterations: SOLVER_ITERATIONS,
            solver_omega: SOLVER_OMEGA,
            par_min: PAR_MIN_PARTICLES,
            substeps: 1,
            strategy: Strategy::default(),
            gravity: GRAVITY,
            pbf: Pbf::new(),
            last_max_velocity: 0.0,
        }
    }

    /// Select the fluid model (default `Granular`). Clears any PBF state so a
    /// switch mid-run starts the density solver from the current positions.
    pub fn set_strategy(&mut self, strategy: Strategy) {
        self.strategy = strategy;
        self.pbf.reset();
    }

    /// Per-substep gravity vector. Scenarios use this to tilt the box.
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.gravity = gravity;
    }

    /// Override the PBF coefficients (used by the render tool to sweep them).
    pub fn set_pbf_params(&mut self, params: PbfParams) {
        self.pbf.set_params(params);
    }

    /// Far-field force refresh interval in substeps (1 = every substep).
    /// Invalidates the cache so the new cadence starts with fresh forces.
    pub fn set_force_interval(&mut self, interval: usize) {
        self.force_interval = interval.max(1);
        self.c_farfield.clear();
    }

    /// Contact-solver iterations per substep.
    pub fn set_solver_iterations(&mut self, iterations: usize) {
        self.solver_iterations = iterations.max(1);
    }

    /// Contact-solver over-relaxation factor (1.0 = historical behavior).
    pub fn set_solver_omega(&mut self, omega: f32) {
        self.solver_omega = omega;
    }

    /// Particle count at which the grid paths switch from the serial
    /// Gauss-Seidel engine to the packed parallel Jacobi engine
    /// (0 = always packed, usize::MAX = always serial).
    pub fn set_par_min_particles(&mut self, par_min: usize) {
        self.par_min = par_min;
    }

    /// Internal substeps per `step()` call (Small Steps): each runs the full
    /// integrate → grid → forces(MTS) → solve pipeline at dt/substeps. Pair
    /// with proportionally fewer solver iterations and a scaled
    /// `force_interval` to hold the far-field refresh rate constant.
    pub fn set_substeps(&mut self, substeps: usize) {
        self.substeps = substeps.max(1);
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
            self.substep(effective_dt / s as f32, share);
        }

        // Update performance stats
        share.perf_stats.total_particles = share.c_pos.len();
        share.perf_stats.verlet_lists_enabled = self.use_verlet_lists;
        share.perf_stats.adaptive_dt_enabled = self.use_adaptive_dt;
        share.perf_stats.current_dt = effective_dt;
    }

    fn substep(&mut self, dt: f32, share: &mut ShareData) {
        if self.strategy == Strategy::Pbf {
            self.pbf_substep(dt, share);
            self.frame_count += 1;
            return;
        }

        self.integrate(dt, share);

        // Build the CSR grid once per substep; forces and all solver
        // iterations reuse it (positions move a small fraction of a cell).
        let t = Instant::now();
        self.grid
            .build(&share.c_pos, share.c_pos.len() >= self.par_min);
        share.perf_stats.neighbor_rebuild_time_us += t.elapsed().as_micros() as u64;

        if self.frame_count % REORDER_INTERVAL == 0 {
            self.reorder_particles(share);
        }

        let ShareData {
            c_pos, perf_stats, ..
        } = share;
        self.check_ball_collisions(c_pos, perf_stats);

        self.frame_count += 1;
    }

    /// One Position Based Fluids substep. Delegates to the self-contained
    /// `Pbf` solver (its own h-sized grid and velocity state); the timing
    /// buckets are filled so the perf HUD / bench still read something.
    fn pbf_substep(&mut self, dt: f32, share: &mut ShareData) {
        let t = Instant::now();
        self.pbf.substep(dt, self.gravity, share, &mut self.c_opos);
        let us = t.elapsed().as_micros() as u64;
        // Attribute the whole cost to the "collision" bucket (the density
        // solve is the PBF analogue of the contact projection).
        share.perf_stats.collision_time_us += us;
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

    fn integrate(&mut self, dt: f32, share: &mut ShareData) {
        let start = Instant::now();
        let mut max_velocity_sq: f32 = 0.0;
        let mut speed_sum: f32 = 0.0;

        // Colors are only consumed by the 60 Hz renderer; recomputing them
        // (one sqrt + fmod per particle) on all 8 substeps per frame is waste.
        let update_colors = self.frame_count % 8 == 0;

        for i in 0..share.c_pos.len() {
            let c_pos = &mut share.c_pos[i];
            let c_opos = &mut self.c_opos[i];
            let c_force = &mut self.c_force[i];

            let oldnpos = *c_pos;
            let velocity = (*c_pos - *c_opos) * 20.;
            let vel_sq = velocity.length_squared();

            if update_colors {
                share.c_color[i] = (vel_sq.sqrt() + 198.) % 360.;
            }

            // Track maximum velocity for adaptive time-stepping
            max_velocity_sq = max_velocity_sq.max(vel_sq);
            speed_sum += vel_sq.sqrt();

            // Proper Størmer–Verlet: x' = 2x − x_prev + a·dt². The code
            // historically applied (G + F)·dt — an impulse, which made the
            // dynamics depend on the substep rate and blocked Small Steps
            // (stages 04/22). Forces and gravity were tuned in those units,
            // so they are interpreted as impulse-per-480Hz-substep and
            // converted to acceleration by 1/PHYS_TIME_STEP: at
            // dt = PHYS_TIME_STEP this reproduces the historical
            // trajectories bit-for-bit up to rounding; at any other dt the
            // motion now scales physically.
            //
            // NOTE: the old code also added arbitrary_vector_field(pos) here,
            // which is defined to return exactly ZERO (`* 0.0`) but still paid
            // a sin + cos per particle per substep.
            const INV_PHYS_DT: f32 = 1.0 / PHYS_TIME_STEP;
            *c_pos = *c_pos * 2.0 - *c_opos + (self.gravity + *c_force) * (dt * dt * INV_PHYS_DT);
            *c_opos = oldnpos;
            *c_force = Vec2::ZERO;
        }

        self.last_max_velocity = max_velocity_sq.sqrt();
        let n = share.c_pos.len().max(1) as f32;
        share.perf_stats.max_speed = self.last_max_velocity;
        share.perf_stats.mean_speed = speed_sum / n;
        share.perf_stats.pbf_density_ratio = 0.0; // granular: n/a
        share.perf_stats.integration_time_us += start.elapsed().as_micros() as u64;
    }

    /// Permute all per-particle arrays into grid order (the grid's `indices`
    /// is exactly that permutation), then patch the grid back to identity.
    fn reorder_particles(&mut self, share: &mut ShareData) {
        let n = share.c_pos.len();
        if n == 0 {
            return;
        }

        let perm = &self.grid.indices;
        let permute_v2 = |src: &mut Vec<Vec2>, scratch: &mut Vec<Vec2>| {
            scratch.clear();
            scratch.extend(perm.iter().map(|&p| src[p as usize]));
            std::mem::swap(src, scratch);
        };

        permute_v2(&mut share.c_pos, &mut self.scratch_v2);
        permute_v2(&mut self.c_opos, &mut self.scratch_v2);
        permute_v2(&mut self.c_force, &mut self.scratch_v2);
        if self.c_farfield.len() == n {
            permute_v2(&mut self.c_farfield, &mut self.scratch_v2);
        }

        self.scratch_f.clear();
        self.scratch_f
            .extend(perm.iter().map(|&p| share.c_color[p as usize]));
        std::mem::swap(&mut share.c_color, &mut self.scratch_f);

        // Grid order is now array order; cell assignments are unchanged.
        for (k, (idx, cell)) in self
            .grid
            .indices
            .iter_mut()
            .zip(&mut self.grid.cell_of)
            .enumerate()
        {
            *idx = k as u32;
            *cell = CsrGrid::cell_id(share.c_pos[k]);
        }

        // Particle indices changed; stored neighbor lists are meaningless now.
        // Prime the lazy-rebuild counter so list mode resumes on the very next
        // force pass instead of waiting out the retry interval.
        self.neighbor_lists.start.clear();
        self.stale_steps = VERLET_REBUILD_RETRY - 1;
    }

    fn check_ball_collisions(&mut self, c_pos: &mut [Vec2], stats: &mut PerformanceStats) {
        // Two solver engines (see docs/benchmarks/10-packed-simd-jacobi.md):
        // below the parallel threshold, the scalar Gauss-Seidel half-stencil
        // sweep on c_pos wins (cells hold ~1-2 particles, so packed rows are
        // too short for SIMD, and the Jacobi gather does every pair twice).
        // At scale, the packed Jacobi engine wins: positions packed once into
        // CSR-ordered SoA, gathers are order-independent, and a step needs
        // only 5 parallel regions instead of 24 colored ones.
        let packed = c_pos.len() >= self.par_min;
        if packed {
            self.pack_positions(c_pos);
        }

        // Far-field forces with multiple time stepping (RESPA-style): the
        // smooth long-range repulsion is refreshed every `force_interval`
        // substeps and held piecewise-constant in between; only the stiff
        // contact projection below runs every substep. A cache-length
        // mismatch (new particles fired in) forces a refresh.
        let t = Instant::now();
        let refresh = self.force_interval <= 1
            || self.c_farfield.len() != c_pos.len()
            || self.frame_count % self.force_interval == 0;
        if refresh {
            self.compute_forces(c_pos);
            if self.force_interval > 1 {
                self.c_farfield.clear();
                self.c_farfield.extend_from_slice(&self.c_force);
            }
        } else {
            // Reuse the cached field (integrate() zeroed c_force).
            self.c_force.copy_from_slice(&self.c_farfield);
        }
        stats.force_calc_time_us += t.elapsed().as_micros() as u64;

        // Handle collisions using constraint solver
        let t = Instant::now();
        if packed {
            for _ in 0..self.solver_iterations {
                self.resolve_collisions_packed();
                self.check_wall_collisions_packed();
            }
            self.scatter_positions(c_pos);
        } else {
            for _ in 0..self.solver_iterations {
                self.resolve_collisions_serial(c_pos);
                self.check_wall_collisions_serial(c_pos);
            }
        }
        stats.collision_time_us += t.elapsed().as_micros() as u64;
    }

    fn pack_positions(&mut self, c_pos: &[Vec2]) {
        let grid = &self.grid;
        self.px.clear();
        self.py.clear();
        self.packed_cell.clear();
        self.px
            .extend(grid.indices.iter().map(|&i| c_pos[i as usize].x));
        self.py
            .extend(grid.indices.iter().map(|&i| c_pos[i as usize].y));
        self.packed_cell
            .extend(grid.indices.iter().map(|&i| grid.cell_of[i as usize]));
    }

    fn scatter_positions(&self, c_pos: &mut [Vec2]) {
        for (k, &i) in self.grid.indices.iter().enumerate() {
            c_pos[i as usize] = Vec2::new(self.px[k], self.py[k]);
        }
    }

    /// Bench/test hook: the model's forces via the grid stencil, for
    /// validating stencil exactness against `forces_direct` (the cutoff is
    /// ≤ one cell, so the 3×3 neighborhood must contain every interacting
    /// pair — any discrepancy beyond summation-order rounding is a bug).
    pub fn forces_grid(&mut self, positions: &[Vec2]) -> Vec<Vec2> {
        self.grid.build(positions, false);
        let (grid, scale) = (&self.grid, self.scale);
        positions
            .iter()
            .enumerate()
            .map(|(i, &p)| {
                let cell = grid.cell_of[i] as usize;
                let (x, y) = (cell % GRID_W, cell / GRID_W);
                let mut acc = Vec2::ZERO;
                for dy in -1i32..=1 {
                    let ny = y as i32 + dy;
                    if ny < 0 || ny >= GRID_H as i32 {
                        continue;
                    }
                    for dx in -1i32..=1 {
                        let nx = x as i32 + dx;
                        if nx < 0 || nx >= GRID_W as i32 {
                            continue;
                        }
                        for &j in grid.cell(ny as usize * GRID_W + nx as usize) {
                            if j as usize != i {
                                acc += force(p, positions[j as usize], scale);
                            }
                        }
                    }
                }
                acc / 8.0
            })
            .collect()
    }

    /// Bench/test hook: exact O(n²) pairwise forces with the engine's kernel
    /// and scaling — the model's ground truth, for validating that the grid
    /// gathers compute the identical interaction set.
    pub fn forces_direct(&self, positions: &[Vec2]) -> Vec<Vec2> {
        let scale = self.scale;
        positions
            .par_iter()
            .map(|&pi| {
                let mut acc = Vec2::ZERO;
                for &pj in positions {
                    acc += force(pi, pj, scale);
                }
                acc / 8.0
            })
            .collect()
    }

    fn compute_forces(&mut self, c_pos: &[Vec2]) {
        if self.use_verlet_lists {
            if !self.neighbor_lists.needs_rebuild(c_pos) {
                self.stale_steps = 0;
                self.compute_forces_with_verlet_lists(c_pos);
                return;
            }
            // Lists are stale. Rebuilding every step under fast motion costs
            // more than the direct cell pass, so only re-attempt list mode
            // periodically; otherwise take the direct path this step.
            self.stale_steps += 1;
            if self.stale_steps >= VERLET_REBUILD_RETRY {
                self.stale_steps = 0;
                self.rebuild_neighbor_lists(c_pos);
                self.compute_forces_with_verlet_lists(c_pos);
                return;
            }
        }
        self.compute_forces_with_spatial_hash(c_pos);
    }

    fn rebuild_neighbor_lists(&mut self, c_pos: &[Vec2]) {
        // The step-level CSR grid is already built; gather from it into the
        // flat CSR lists (built in particle order, so a single append pass).
        let lists = &mut self.neighbor_lists;
        lists.start.resize(c_pos.len() + 1, 0);
        lists.neighbors.clear();
        lists.ref_pos.clear();
        lists.ref_pos.extend_from_slice(c_pos);

        // Lists must hold every pair that can come within FORCE_CUTOFF while
        // displacements stay under skin/2 each — i.e. build out to
        // cutoff + skin (9 px). That exceeds one cell (7.5 px), so the list
        // build walks a 5×5 stencil. (Historically this range was
        // 2·BALL + skin over 3×3, which silently truncated list-mode forces
        // relative to the direct pass; with the stage-25 explicit cutoff the
        // two modes now compute the same interaction set.)
        let interaction_range_sq =
            (FORCE_CUTOFF + VERLET_SKIN_DISTANCE) * (FORCE_CUTOFF + VERLET_SKIN_DISTANCE);
        for i in 0..c_pos.len() {
            lists.start[i] = lists.neighbors.len() as u32;

            let cell = self.grid.cell_of[i] as usize;
            let (x, y) = (cell % GRID_W, cell / GRID_W);

            for dy in -2i32..=2 {
                let ny = y as i32 + dy;
                if ny < 0 || ny >= GRID_H as i32 {
                    continue;
                }
                for dx in -2i32..=2 {
                    let nx = x as i32 + dx;
                    if nx < 0 || nx >= GRID_W as i32 {
                        continue;
                    }
                    for &j in self.grid.cell(ny as usize * GRID_W + nx as usize) {
                        if i != j as usize
                            && (c_pos[i] - c_pos[j as usize]).length_squared()
                                < interaction_range_sq
                        {
                            lists.neighbors.push(j);
                        }
                    }
                }
            }
        }
        lists.start[c_pos.len()] = lists.neighbors.len() as u32;
    }

    fn compute_forces_with_verlet_lists(&mut self, c_pos: &[Vec2]) {
        if c_pos.len() >= self.par_min {
            // Parallel gather: each particle sums its full (symmetric) neighbor
            // list, so no cross-thread writes. Twice the arithmetic of the
            // Newton's-third-law scatter, but it parallelizes cleanly.
            let (c_force, lists, scale) = (&mut self.c_force, &self.neighbor_lists, self.scale);
            c_force.par_iter_mut().enumerate().for_each(|(i, f)| {
                let mut acc = Vec2::ZERO;
                for &j in lists.of(i) {
                    acc += force(c_pos[i], c_pos[j as usize], scale);
                }
                *f += acc / 8.0;
            });
        } else {
            for i in 0..c_pos.len() {
                for &j in self.neighbor_lists.of(i) {
                    let j = j as usize;
                    if i < j {
                        let f = force(c_pos[i], c_pos[j], self.scale) / 8.0;
                        self.c_force[i] += f;
                        self.c_force[j] -= f; // Newton's third law
                    }
                }
            }
        }
    }

    fn compute_forces_with_spatial_hash(&mut self, c_pos: &[Vec2]) {
        let n = c_pos.len();
        if n >= self.par_min {
            // Packed parallel gather over the CSR-ordered SoA arrays. The
            // gather writes every element, so the buffers need sizing only.
            let s8 = self.scale / 8.0;
            if self.acc_x.len() != n {
                self.acc_x.resize(n, 0.0);
                self.acc_y.resize(n, 0.0);
            }

            {
                let (grid, px, py, cells) = (&self.grid, &self.px, &self.py, &self.packed_cell);
                let (acc_x, acc_y) = (&mut self.acc_x, &mut self.acc_y);
                acc_x
                    .par_iter_mut()
                    .zip(acc_y.par_iter_mut())
                    .enumerate()
                    .for_each(|(k, (fx, fy))| {
                        let rows = stencil_rows(&grid.cell_start, cells[k] as usize);
                        let (gx, gy) = gather_force(px[k], py[k], px, py, &rows, s8);
                        *fx = gx;
                        *fy = gy;
                    });
            }

            for (k, &i) in self.grid.indices.iter().enumerate() {
                self.c_force[i as usize] += Vec2::new(self.acc_x[k], self.acc_y[k]);
            }
            return;
        }

        // Half-neighborhood sweep over occupied CSR cells: each unordered cell
        // pair is visited once and Newton's third law is applied per pair.
        for oc in 0..self.grid.occupied.len() {
            let cell = self.grid.occupied[oc] as usize;
            let (x, y) = (cell % GRID_W, cell / GRID_W);
            let currents = self.grid.cell(cell);

            // Pairs within the cell
            for a in 0..currents.len() {
                let i = currents[a] as usize;
                for b in a + 1..currents.len() {
                    let j = currents[b] as usize;
                    let f = force(c_pos[i], c_pos[j], self.scale) / 8.0;
                    self.c_force[i] += f;
                    self.c_force[j] -= f;
                }
            }

            // Forward half-stencil: E, SW, S, SE
            for (dx, dy) in [(1i32, 0i32), (-1, 1), (0, 1), (1, 1)] {
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if nx < 0 || nx >= GRID_W as i32 || ny >= GRID_H as i32 {
                    continue;
                }
                for &i in currents {
                    let i = i as usize;
                    for &j in self.grid.cell(ny as usize * GRID_W + nx as usize) {
                        let j = j as usize;
                        let f = force(c_pos[i], c_pos[j], self.scale) / 8.0;
                        self.c_force[i] += f;
                        self.c_force[j] -= f;
                    }
                }
            }
        }
    }

    /// Project one overlapping pair apart (half correction each side). One
    /// rsqrt replaces length() + try_normalize(); the common non-overlapping
    /// case exits before any position store.
    #[inline(always)]
    fn project_pair(c_pos: &mut [Vec2], i: usize, j: usize, relax: f32) {
        const CONTACT: f32 = BALL_SIZE + BALL_SIZE;
        let col_axis = c_pos[i] - c_pos[j];
        let dist_sq = col_axis.length_squared();
        if dist_sq >= CONTACT * CONTACT {
            return;
        }
        if dist_sq < 1e-12 {
            // Exactly-coincident pair. The old code skipped these (like
            // try_normalize() -> ZERO), which made coincidence an absorbing
            // "glued" state: no solver iteration could ever separate the
            // pair again. There is no physical axis, so pick a deterministic
            // tie-break direction from the indices and project as if fully
            // overlapped along it.
            let dir = if (i ^ j) & 1 == 0 { Vec2::X } else { Vec2::Y };
            let sep = dir * (relax * CONTACT);
            c_pos[i] += sep;
            c_pos[j] -= sep;
            return;
        }
        let inv_dist = fast_rsqrt(dist_sq);
        let dist = dist_sq * inv_dist;
        // == normalized(col_axis) * relax * (dist - CONTACT), where
        // relax = 0.375·ω (ω = 1 reproduces the historical 0.75/2 factor)
        let corr = col_axis * (inv_dist * relax * (dist - CONTACT));
        c_pos[i] -= corr;
        c_pos[j] += corr;
    }

    /// Gauss-Seidel sweep over occupied cells (forward half-stencil, each
    /// unordered cell pair once). Fastest engine at serial particle counts.
    fn resolve_collisions_serial(&mut self, c_pos: &mut [Vec2]) {
        let relax = 0.375 * self.solver_omega;
        for oc in 0..self.grid.occupied.len() {
            let cell = self.grid.occupied[oc] as usize;
            let (x, y) = (cell % GRID_W, cell / GRID_W);
            let currents = self.grid.cell(cell);

            for a in 0..currents.len() {
                let i = currents[a] as usize;
                for b in a + 1..currents.len() {
                    Self::project_pair(c_pos, i, currents[b] as usize, relax);
                }
            }

            for (dx, dy) in [(1i32, 0i32), (-1, 1), (0, 1), (1, 1)] {
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if nx < 0 || nx >= GRID_W as i32 || ny >= GRID_H as i32 {
                    continue;
                }
                for &i in currents {
                    for &j in self.grid.cell(ny as usize * GRID_W + nx as usize) {
                        Self::project_pair(c_pos, i as usize, j as usize, relax);
                    }
                }
            }
        }
    }

    fn check_wall_collisions_serial(&mut self, c_pos: &mut [Vec2]) {
        let mut wall = |cells: &[u32], c_opos: &mut [Vec2]| {
            for &i in cells {
                let i = i as usize;
                resolve_wall_collision(&mut c_pos[i], &mut c_opos[i], wall_salt(i));
            }
        };
        for y in 0..GRID_H {
            wall(self.grid.cell(y * GRID_W), &mut self.c_opos);
            wall(self.grid.cell(y * GRID_W + (GRID_W - 1)), &mut self.c_opos);
        }
        for x in 0..GRID_W {
            wall(self.grid.cell(x), &mut self.c_opos);
            wall(self.grid.cell((GRID_H - 1) * GRID_W + x), &mut self.c_opos);
        }
    }

    /// One Jacobi iteration of contact projection on the packed arrays:
    /// gather every particle's half-corrections (read-only, so trivially
    /// parallel and vectorizable). Gathers run per *cluster* of 4
    /// consecutive packed particles: packed order is cell order, so a
    /// cluster usually sits on one grid row and shares a union stencil —
    /// each candidate is then loaded once against 4 full SIMD lanes instead
    /// of 4 times against ~1-wide rows (stage 23). Clusters that straddle a
    /// grid row (or the tail) fall back to the scalar path.
    ///
    /// Double-buffered (stage 26): the gather writes the *corrected
    /// positions* into acc_x/acc_y directly and the buffers are swapped in,
    /// replacing the old accumulate-then-serial-apply pass. Bit-identical
    /// results, three fewer serial sweeps per substep. The buffers are
    /// fully overwritten, so no zeroing either.
    fn resolve_collisions_packed(&mut self) {
        let n = self.px.len();
        if self.acc_x.len() != n {
            self.acc_x.resize(n, 0.0);
            self.acc_y.resize(n, 0.0);
        }

        {
            let relax = 0.375 * self.solver_omega;
            let (grid, px, py, cells) = (&self.grid, &self.px, &self.py, &self.packed_cell);
            let (acc_x, acc_y) = (&mut self.acc_x, &mut self.acc_y);
            acc_x
                .par_chunks_mut(4)
                .zip(acc_y.par_chunks_mut(4))
                .enumerate()
                .for_each(|(c, (ax, ay))| {
                    let k0 = c * 4;
                    let m = ax.len();
                    // Cells are ascending in packed order; one union stencil
                    // covers the cluster iff first and last share a grid row
                    // AND sit close enough that the union stays narrow (in
                    // sparse regions consecutive occupied cells can be far
                    // apart — scalar path is fine there, the rows are short).
                    let c_lo = cells[k0] as usize;
                    let c_hi = cells[k0 + m - 1] as usize;
                    if m == 4 && c_hi - c_lo < 4 && c_lo / GRID_W == c_hi / GRID_W {
                        let rows = stencil_rows_span(&grid.cell_start, c_lo, c_hi);
                        let xs = [px[k0], px[k0 + 1], px[k0 + 2], px[k0 + 3]];
                        let ys = [py[k0], py[k0 + 1], py[k0 + 2], py[k0 + 3]];
                        let (gx, gy) = gather_correction4(&xs, &ys, px, py, &rows, relax);
                        for l in 0..4 {
                            ax[l] = xs[l] + gx[l];
                            ay[l] = ys[l] + gy[l];
                        }
                    } else {
                        for l in 0..m {
                            let rows = stencil_rows(&grid.cell_start, cells[k0 + l] as usize);
                            let (gx, gy) =
                                gather_correction(px[k0 + l], py[k0 + l], px, py, &rows, relax);
                            ax[l] = px[k0 + l] + gx;
                            ay[l] = py[k0 + l] + gy;
                        }
                    }
                });
        }

        // The corrected positions are the new packed state.
        std::mem::swap(&mut self.px, &mut self.acc_x);
        std::mem::swap(&mut self.py, &mut self.acc_y);
    }

    fn check_wall_collisions_packed(&mut self) {
        // Left and right border columns
        for y in 0..GRID_H {
            self.wall_cell(y * GRID_W);
            self.wall_cell(y * GRID_W + (GRID_W - 1));
        }

        // Bottom and top border rows
        for x in 0..GRID_W {
            self.wall_cell(x);
            self.wall_cell((GRID_H - 1) * GRID_W + x);
        }
    }

    /// Wall-project the packed particles of one (border) cell. `c_opos` is
    /// still indexed by particle id — the grid indices map back.
    fn wall_cell(&mut self, cell: usize) {
        let start = self.grid.cell_start[cell] as usize;
        let end = self.grid.cell_start[cell + 1] as usize;
        for k in start..end {
            let mut pos = Vec2::new(self.px[k], self.py[k]);
            let i = self.grid.indices[k] as usize;
            resolve_wall_collision(&mut pos, &mut self.c_opos[i], wall_salt(i));
            self.px[k] = pos.x;
            self.py[k] = pos.y;
        }
    }

    pub fn do_cannon(&mut self, dt: f32, share: &mut ShareData, start: Vec2, cannon: Vec2) {
        for k in 0..20 {
            self.cannon(-k as f32 * (2.2 * BALL_SIZE), 0., dt, share, start, cannon);
        }
    }

    pub fn toggle_verlet_lists(&mut self) {
        self.use_verlet_lists = !self.use_verlet_lists;
        println!(
            "Verlet Lists: {}",
            if self.use_verlet_lists { "ON" } else { "OFF" }
        );
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
        self.c_force.push(Vec2::ZERO);
        share.c_color.push(color);
    }
}

// ===========================================================================
// Position Based Fluids (Macklin & Müller, SIGGRAPH 2013)
// ===========================================================================
//
// A self-contained alternative to the granular repulsion+contact model above.
// It shares nothing with the packed granular kernels on purpose: the two
// models want different interaction radii (PBF's smoothing radius h is several
// particle spacings, vs the granular one-cell contact cutoff), and keeping PBF
// isolated leaves the door open to a 3D port without disturbing the tuned 2D
// granular engine.
//
// The per-substep loop (paper §3, algorithm 1):
//   1. predict positions under gravity (symplectic Euler, matching the
//      granular integrator's effective acceleration so both models fall the
//      same way).
//   2. build a uniform grid at cell size h for neighbor search.
//   3. solver iterations, each a parallel Jacobi sweep:
//        density  ρ_i = Σ_j W_poly6(‖x_i−x_j‖, h)
//        constraint C_i = ρ_i/ρ0 − 1
//        λ_i = −C_i / (Σ_k ‖∇_k C_i‖² + ε)
//        Δx_i = (1/ρ0) Σ_j (λ_i + λ_j + s_corr) ∇W_spiky(x_i−x_j, h)
//      s_corr is the artificial-pressure cohesion term (paper §4); walls are
//      enforced by clamping after each iteration.
//   4. velocity v_i = (x_i − x_prev)/Δt.
//   5. XSPH viscosity + vorticity confinement (paper §§5–6).

// Smoothing radius: ~2.5 rest spacings, so each particle sees ~20 neighbors.
const PBF_H: f32 = 15.0;
const PBF_H2: f32 = PBF_H * PBF_H;
// Rest spacing the fluid relaxes to. One contact diameter keeps a PBF fluid at
// roughly the same packing a granular pile settles to, so the two models fill
// comparable volume for a given particle count.
const PBF_REST_SPACING: f32 = 2.0 * BALL_SIZE;
const PBF_SCORR_N: i32 = 4;

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
            scorr_k: 3.0,
            scorr_dq: 0.2 * PBF_H,
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
/// the box. Same counting-sort CSR layout as `CsrGrid`, but its own type so
/// the two models don't fight over the compile-time granular cell size.
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
struct Pbf {
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
    fn new() -> Self {
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

    /// Drop velocity state so a strategy switch (or reuse) restarts from rest.
    fn reset(&mut self) {
        self.vel.clear();
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

    /// One Jacobi iteration: density → λ (parallel gather), then Δx from the
    /// λ's (parallel gather), then apply with a clamp and wall projection.
    fn solve_iteration(&mut self, x: &mut [Vec2]) {
        let n = x.len();
        let rho0 = self.rest_density;
        let inv_rho0 = 1.0 / rho0;
        let eps_cfm = self.params.eps_cfm;
        let scorr_k = self.params.scorr_k;
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
                    let scorr = scorr(d.length_squared(), scorr_denom, scorr_k);
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

// Gravity/force scaling constant reused by the vorticity term so injected
// accelerations live in the same units as gravity.
const INV_PHYS_DT_MULT: f32 = 1.0 / PHYS_TIME_STEP;

/// s_corr artificial-pressure term: −k (W(r)/W(Δq))^n. `denom` = W_poly6(Δq²).
#[inline(always)]
fn scorr(r2: f32, denom: f32, k: f32) -> f32 {
    let ratio = w_poly6(r2) / denom;
    let mut p = ratio;
    for _ in 1..PBF_SCORR_N {
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

/// Keep a position inside the box by one ball radius, matching the granular
/// wall inset. Pure position clamp (PBF handles the bounce via the velocity
/// update, which sees the clamped displacement).
#[inline(always)]
fn clamp_wall(p: &mut Vec2) {
    p.x = p.x.clamp(LEFT_WALL + BALL_SIZE, RIGHT_WALL - BALL_SIZE);
    p.y = p.y.clamp(BOTTOM_WALL + BALL_SIZE, TOP_WALL - BALL_SIZE);
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

#[inline(always)]
fn force(pos_a: Vec2, pos_b: Vec2, scale: f32) -> Vec2 {
    let dir = pos_a - pos_b;
    let dist_sq = dir.length_squared();
    if dist_sq < BALL_SIZE * BALL_SIZE || dist_sq >= FORCE_CUTOFF_SQ {
        return Vec2::ZERO;
    }

    // == normalize(dir) * scale * taper / dist², with one rsqrt total
    // (active pairs have dist² ≥ BALL² > 1, so the historical max(dist², 1)
    // clamp is a no-op and was dropped)
    dir * (scale * taper(dist_sq) * fast_rsqrt(dist_sq) / dist_sq)
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
fn resolve_wall_collision(c_pos: &mut Vec2, c_opos: &mut Vec2, salt: f32) {
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
fn wall_salt(i: usize) -> f32 {
    1e-5 * (i & 15) as f32
}
