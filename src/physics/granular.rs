//! Granular repulsion + non-penetration contact model as a `FluidSolver`
//! strategy. This is the historical, heavily-optimized engine: a short-range
//! 1/r² repulsion field with compact support plus a hard non-penetration
//! contact projection — it stacks and piles like a granular material
//! (sand/balls).
//!
//! The engine keeps all of its own scratch (the CSR grid, packed SoA arrays,
//! Verlet neighbor lists, far-field force cache) inside the solver. The one
//! piece of state it shares with the coordinator is `c_opos` — the
//! Størmer–Verlet "previous position" that also encodes velocity — which is
//! threaded in as a parameter so a strategy switch and the cannon can keep it
//! consistent across models.

use glam::Vec2;
use rayon::prelude::*;
use std::time::Instant;

use super::{
    fast_rsqrt, resolve_wall_collision, wall_salt, FluidSolver, PerformanceStats, ShareData,
    PHYS_TIME_STEP,
};
use crate::constants::{BALL_SIZE, GRID_SIZE, X_LEN, Y_LEN};

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

// Grid dimensions as integers (X_LEN/Y_LEN are f32 for legacy reasons)
const GRID_W: usize = X_LEN as usize;
const GRID_H: usize = Y_LEN as usize;
const N_CELLS: usize = GRID_W * GRID_H;

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

/// Granular repulsion + non-penetration contact solver, with all of its
/// optimization scratch owned here.
pub struct GranularSolver {
    c_force: Vec<Vec2>,
    grid: CsrGrid,
    scale: f32,

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
    force_interval: usize,
    solver_iterations: usize,
    solver_omega: f32,
    par_min: usize,
}

impl GranularSolver {
    pub fn new(scale: f32, c_force: Vec<Vec2>) -> Self {
        Self {
            c_force,
            grid: CsrGrid::new(),
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
            force_interval: FORCE_INTERVAL,
            solver_iterations: SOLVER_ITERATIONS,
            solver_omega: SOLVER_OMEGA,
            par_min: PAR_MIN_PARTICLES,
        }
    }

    fn integrate(&mut self, dt: f32, gravity: Vec2, share: &mut ShareData, c_opos: &mut [Vec2]) {
        let start = Instant::now();
        let mut max_velocity_sq: f32 = 0.0;
        let mut speed_sum: f32 = 0.0;

        // Colors are only consumed by the 60 Hz renderer; recomputing them
        // (one sqrt + fmod per particle) on all 8 substeps per frame is waste.
        let update_colors = self.frame_count % 8 == 0;

        for i in 0..share.c_pos.len() {
            let c_pos = &mut share.c_pos[i];
            let c_opos = &mut c_opos[i];
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
            const INV_PHYS_DT: f32 = 1.0 / PHYS_TIME_STEP;
            *c_pos = *c_pos * 2.0 - *c_opos + (gravity + *c_force) * (dt * dt * INV_PHYS_DT);
            *c_opos = oldnpos;
            *c_force = Vec2::ZERO;
        }

        let n = share.c_pos.len().max(1) as f32;
        share.perf_stats.max_speed = max_velocity_sq.sqrt();
        share.perf_stats.mean_speed = speed_sum / n;
        share.perf_stats.pbf_density_ratio = 0.0; // granular: n/a
        share.perf_stats.integration_time_us += start.elapsed().as_micros() as u64;
    }

    /// Permute all per-particle arrays into grid order (the grid's `indices`
    /// is exactly that permutation), then patch the grid back to identity.
    fn reorder_particles(&mut self, share: &mut ShareData, c_opos: &mut Vec<Vec2>) {
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
        permute_v2(c_opos, &mut self.scratch_v2);
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

    fn check_ball_collisions(
        &mut self,
        c_pos: &mut [Vec2],
        c_opos: &mut [Vec2],
        stats: &mut PerformanceStats,
    ) {
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
                self.check_wall_collisions_packed(c_opos);
            }
            self.scatter_positions(c_pos);
        } else {
            for _ in 0..self.solver_iterations {
                self.resolve_collisions_serial(c_pos);
                self.check_wall_collisions_serial(c_pos, c_opos);
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
    fn forces_grid_impl(&mut self, positions: &[Vec2]) -> Vec<Vec2> {
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
    fn forces_direct_impl(&self, positions: &[Vec2]) -> Vec<Vec2> {
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

    fn check_wall_collisions_serial(&mut self, c_pos: &mut [Vec2], c_opos: &mut [Vec2]) {
        let mut wall = |cells: &[u32], c_opos: &mut [Vec2]| {
            for &i in cells {
                let i = i as usize;
                resolve_wall_collision(&mut c_pos[i], &mut c_opos[i], wall_salt(i));
            }
        };
        for y in 0..GRID_H {
            wall(self.grid.cell(y * GRID_W), c_opos);
            wall(self.grid.cell(y * GRID_W + (GRID_W - 1)), c_opos);
        }
        for x in 0..GRID_W {
            wall(self.grid.cell(x), c_opos);
            wall(self.grid.cell((GRID_H - 1) * GRID_W + x), c_opos);
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

    fn check_wall_collisions_packed(&mut self, c_opos: &mut [Vec2]) {
        // Left and right border columns
        for y in 0..GRID_H {
            self.wall_cell(y * GRID_W, c_opos);
            self.wall_cell(y * GRID_W + (GRID_W - 1), c_opos);
        }

        // Bottom and top border rows
        for x in 0..GRID_W {
            self.wall_cell(x, c_opos);
            self.wall_cell((GRID_H - 1) * GRID_W + x, c_opos);
        }
    }

    /// Wall-project the packed particles of one (border) cell. `c_opos` is
    /// still indexed by particle id — the grid indices map back.
    fn wall_cell(&mut self, cell: usize, c_opos: &mut [Vec2]) {
        let start = self.grid.cell_start[cell] as usize;
        let end = self.grid.cell_start[cell + 1] as usize;
        for k in start..end {
            let mut pos = Vec2::new(self.px[k], self.py[k]);
            let i = self.grid.indices[k] as usize;
            resolve_wall_collision(&mut pos, &mut c_opos[i], wall_salt(i));
            self.px[k] = pos.x;
            self.py[k] = pos.y;
        }
    }
}

impl FluidSolver for GranularSolver {
    fn name(&self) -> &'static str {
        "Granular"
    }

    fn verlet_lists_enabled(&self) -> bool {
        self.use_verlet_lists
    }

    fn add_scale(&mut self, delta: f32) {
        self.scale += delta;
    }

    fn set_force_interval(&mut self, interval: usize) {
        self.force_interval = interval.max(1);
        self.c_farfield.clear();
    }

    fn set_solver_iterations(&mut self, iterations: usize) {
        self.solver_iterations = iterations.max(1);
    }

    fn set_solver_omega(&mut self, omega: f32) {
        self.solver_omega = omega;
    }

    fn set_par_min_particles(&mut self, par_min: usize) {
        self.par_min = par_min;
    }

    fn toggle_verlet_lists(&mut self) {
        self.use_verlet_lists = !self.use_verlet_lists;
        println!(
            "Verlet Lists: {}",
            if self.use_verlet_lists { "ON" } else { "OFF" }
        );
    }

    fn forces_grid(&mut self, positions: &[Vec2]) -> Vec<Vec2> {
        self.forces_grid_impl(positions)
    }

    fn forces_direct(&self, positions: &[Vec2]) -> Vec<Vec2> {
        self.forces_direct_impl(positions)
    }

    fn substep(&mut self, dt: f32, gravity: Vec2, share: &mut ShareData, c_opos: &mut Vec<Vec2>) {
        let n = share.c_pos.len();
        // The cannon pushes to c_pos/c_opos/c_color only; grow c_force to match
        // (new particles start with zero accumulated force, as before).
        self.c_force.resize(n, Vec2::ZERO);

        self.integrate(dt, gravity, share, c_opos);

        // Build the CSR grid once per substep; forces and all solver
        // iterations reuse it (positions move a small fraction of a cell).
        let t = Instant::now();
        self.grid
            .build(&share.c_pos, share.c_pos.len() >= self.par_min);
        share.perf_stats.neighbor_rebuild_time_us += t.elapsed().as_micros() as u64;

        if self.frame_count % REORDER_INTERVAL == 0 {
            self.reorder_particles(share, c_opos);
        }

        let ShareData {
            c_pos, perf_stats, ..
        } = share;
        self.check_ball_collisions(c_pos, c_opos, perf_stats);

        self.frame_count += 1;
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
