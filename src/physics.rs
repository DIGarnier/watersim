use glam::Vec2;
use rayon::prelude::*;
use std::sync::mpsc::Receiver;
use std::time::Instant;

use crate::constants::{
    BALL_SIZE, GRID_SIZE, HEIGHT, INITIAL_BALL_SPEED_MODIFIER, WIDTH, X_LEN, Y_LEN,
};

const GRAVITY: Vec2 = Vec2::new(0.0, 9.8);
pub const PHYS_TIME_STEP: f32 = 1.0 / 480.0;

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

// Barnes-Hut opening angle. Monopole force error grows ~θ²; the sweep in
// docs/benchmarks/13-bh-theta.md measured the actual error/speed curve for
// this sim and picked the default. Runtime-tunable via `set_bh_theta`.
const BARNES_HUT_THETA: f32 = 0.9;

// Far-field force refresh interval, in substeps (RESPA-style multiple time
// stepping; Tuckerman, Berne & Martyna 1992, docs/literature.md §7). The
// repulsion field is smooth and positions move a tiny fraction of a cell per
// 480 Hz substep, so the expensive force pass (BH tree or grid gather) is
// recomputed only every K substeps and reused in between. The contact solver
// — the fast, stiff part — still runs every substep. Quality was measured in
// docs/benchmarks/14-mts-farfield.md. Runtime-tunable via
// `set_force_interval`; 1 = recompute every substep (old behavior).
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
// dominates small workloads). Originally 16k (docs/benchmarks/10); re-swept
// after MTS + 3 iterations + smaller particles changed the packed engine's
// cost profile (docs/benchmarks/18-engine-crossover.md): packed now wins at
// ~16-20k on spatial-hash but only ~24k on verlet-lists, margins within the
// VM noise band, so the single threshold sits between. Runtime-tunable via
// `set_par_min_particles`. The Barnes-Hut traversal has no threshold: it is
// one region of heavy independent work and wins at every tested size.
const PAR_MIN_PARTICLES: usize = 22_000;

const LEFT_WALL: f32 = 0.;
const RIGHT_WALL: f32 = WIDTH;
const BOTTOM_WALL: f32 = 0.;
const TOP_WALL: f32 = HEIGHT;

pub enum EventToPthread {
    Cannon((Vec2, Vec2)),
    Scale(f32),
    ToggleBarnesHut,
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
    pub tree_build_time_us: u64,
    pub force_calc_time_us: u64,
    pub total_particles: usize,
    pub neighbor_checks: usize,
    pub tree_node_count: usize,
    pub barnes_hut_enabled: bool,
    pub verlet_lists_enabled: bool,
    pub adaptive_dt_enabled: bool,
    pub current_dt: f32,
}

#[derive(Clone, Copy)]
struct AABB {
    min: Vec2,
    max: Vec2,
}

impl AABB {
    fn new(min: Vec2, max: Vec2) -> Self {
        Self { min, max }
    }

    fn center(&self) -> Vec2 {
        (self.min + self.max) * 0.5
    }

    fn size(&self) -> f32 {
        (self.max.x - self.min.x).max(self.max.y - self.min.y)
    }

    fn quadrant(&self, idx: usize) -> AABB {
        let center = self.center();
        match idx {
            0 => AABB::new(self.min, center), // Bottom-left
            1 => AABB::new(Vec2::new(center.x, self.min.y), Vec2::new(self.max.x, center.y)), // Bottom-right
            2 => AABB::new(Vec2::new(self.min.x, center.y), Vec2::new(center.x, self.max.y)), // Top-left
            3 => AABB::new(center, self.max), // Top-right
            _ => unreachable!(),
        }
    }
}

const BH_NONE: u32 = u32::MAX;

// Max particles per leaf before it subdivides. Bucketed leaves keep the tree
// shallow and turn near-field work into tight exact loops (standard treecode
// practice; see docs/literature.md §3).
const BH_LEAF_CAP: u32 = 8;

/// Traversal-hot node data: 24 bytes, so ~2.7 nodes per cache line versus 44
/// bytes when bounds/count rode along. Force traversal touches only this.
#[derive(Clone, Copy)]
struct BhHot {
    center_of_mass: Vec2,
    total_mass: f32,
    size_sq: f32,   // bounds.size()² cached for the sqrt-free acceptance test
    children: u32,  // BH_NONE = leaf, else index of first of 4 contiguous children
    leaf_start: u32, // offset into the packed leaf-member arrays (set by finalize)
    leaf_count: u32,
}

/// Arena-backed Barnes-Hut quadtree (Burtscher & Pingali 2011 style: contiguous
/// node storage, no per-node allocation, non-recursive traversal). Node data is
/// split hot/cold: `hot` is everything the force traversal reads; `bounds` and
/// `count` exist only for tree construction. All buffers keep their capacity
/// across frames, so steady-state tree builds are allocation-free.
struct BhTree {
    hot: Vec<BhHot>,
    bounds: Vec<AABB>, // build-only
    count: Vec<u32>,   // build-only: particles in each leaf
    first: Vec<u32>,   // build-only: head of each leaf's intrusive member list
    next: Vec<u32>,    // per-particle intrusive list linking leaf members
    ppos: Vec<Vec2>,   // per-particle (clamped) position as inserted
    // Leaf members packed contiguously by finalize(): the near-field loop
    // streams these instead of chasing next[] through scattered ppos.
    leaf_x: Vec<f32>,
    leaf_y: Vec<f32>,
}

impl BhTree {
    fn new() -> Self {
        Self {
            hot: Vec::new(),
            bounds: Vec::new(),
            count: Vec::new(),
            first: Vec::new(),
            next: Vec::new(),
            ppos: Vec::new(),
            leaf_x: Vec::new(),
            leaf_y: Vec::new(),
        }
    }

    fn push_node(&mut self, bounds: AABB) {
        self.hot.push(BhHot {
            center_of_mass: Vec2::ZERO,
            total_mass: 0.0,
            size_sq: bounds.size() * bounds.size(),
            children: BH_NONE,
            leaf_start: 0,
            leaf_count: 0,
        });
        self.bounds.push(bounds);
        self.count.push(0);
        self.first.push(BH_NONE);
    }

    fn clear(&mut self, bounds: AABB, n_particles: usize) {
        self.hot.clear();
        self.bounds.clear();
        self.count.clear();
        self.first.clear();
        self.push_node(bounds);
        self.next.resize(n_particles, BH_NONE);
        self.ppos.resize(n_particles, Vec2::ZERO);
    }

    /// Copy every leaf's members into the contiguous arrays and stamp the
    /// (offset, count) into the hot nodes. One O(nodes + n) pass after build.
    fn finalize(&mut self) {
        self.leaf_x.clear();
        self.leaf_y.clear();
        for node in 0..self.hot.len() {
            if self.hot[node].children != BH_NONE {
                continue;
            }
            let start = self.leaf_x.len() as u32;
            let mut p = self.first[node];
            while p != BH_NONE {
                let pos = self.ppos[p as usize];
                self.leaf_x.push(pos.x);
                self.leaf_y.push(pos.y);
                p = self.next[p as usize];
            }
            let h = &mut self.hot[node];
            h.leaf_start = start;
            h.leaf_count = self.leaf_x.len() as u32 - start;
        }
    }

    #[inline(always)]
    fn quadrant_index(bounds: &AABB, pos: Vec2) -> usize {
        let c = bounds.center();
        ((pos.y >= c.y) as usize) * 2 + ((pos.x >= c.x) as usize)
    }

    #[inline(always)]
    fn fold_into(hot: &mut BhHot, pos: Vec2, mass: f32) {
        let total = hot.total_mass + mass;
        hot.center_of_mass = (hot.center_of_mass * hot.total_mass + pos * mass) / total;
        hot.total_mass = total;
    }

    fn insert(&mut self, idx: u32, pos: Vec2, mass: f32) {
        // Below this cell size, keep piling particles into the leaf instead of
        // subdividing forever (guards against (near-)coincident particles).
        const MIN_CELL: f32 = 1e-3;

        self.ppos[idx as usize] = pos;
        let mut node = 0usize;
        loop {
            let n = self.hot[node];

            if n.children != BH_NONE {
                // Internal node: fold the new mass into the aggregate, descend.
                Self::fold_into(&mut self.hot[node], pos, mass);
                node = n.children as usize + Self::quadrant_index(&self.bounds[node], pos);
                continue;
            }

            if self.count[node] < BH_LEAF_CAP || self.bounds[node].size() < MIN_CELL {
                // Leaf with room: link the particle in and update the aggregate.
                self.next[idx as usize] = self.first[node];
                self.first[node] = idx;
                self.count[node] += 1;
                Self::fold_into(&mut self.hot[node], pos, mass);
                return;
            }

            // Full leaf: subdivide and redistribute the residents, then retry
            // this node (now internal) with the incoming particle.
            let node_bounds = self.bounds[node];
            let first_child = self.hot.len() as u32;
            for q in 0..4 {
                self.push_node(node_bounds.quadrant(q));
            }

            let mut p = self.first[node];
            let per_particle_mass = n.total_mass / self.count[node] as f32;
            while p != BH_NONE {
                let p_next = self.next[p as usize];
                let p_pos = self.ppos[p as usize];
                let child = first_child as usize + Self::quadrant_index(&node_bounds, p_pos);
                self.next[p as usize] = self.first[child];
                self.first[child] = p;
                self.count[child] += 1;
                Self::fold_into(&mut self.hot[child], p_pos, per_particle_mass);
                p = p_next;
            }

            self.hot[node].children = first_child;
            self.first[node] = BH_NONE;
            self.count[node] = 0;
        }
    }

    /// Traversal is read-only so it can run for many particles in parallel;
    /// callers supply a reusable scratch stack (one per thread).
    fn compute_force(&self, pos: Vec2, scale: f32, theta: f32, stack: &mut Vec<u32>) -> Vec2 {
        let theta_sq = theta * theta;
        let mut force = Vec2::ZERO;
        stack.clear();
        stack.push(0);

        while let Some(idx) = stack.pop() {
            let n = &self.hot[idx as usize];
            if n.total_mass == 0.0 {
                continue;
            }

            // Repulsion, matching the exact pair kernel `force()`: diff points
            // from the aggregate mass toward the particle. (The original BH
            // code had this inverted — attraction — so toggling BH silently
            // simulated a different system; see docs/benchmarks/13-bh-theta.md.)
            let diff = pos - n.center_of_mass;
            let dist_sq = diff.length_squared();

            // size/dist < theta  <=>  size² < theta²·dist², sqrt-free
            if n.size_sq < theta_sq * dist_sq {
                // Far enough: use the aggregate (internal or leaf alike).
                if dist_sq < BALL_SIZE * BALL_SIZE {
                    continue;
                }
                // == normalize(diff) * scale * mass / max(dist², 1)
                force += diff * (scale * n.total_mass * fast_rsqrt(dist_sq) / dist_sq.max(1.0));
            } else if n.children != BH_NONE {
                let c = n.children;
                // The 4 children (96 B, contiguous) will be popped within the
                // next few iterations; start their cache lines now.
                #[cfg(target_arch = "x86_64")]
                unsafe {
                    use core::arch::x86_64::{_mm_prefetch, _MM_HINT_T0};
                    let p = self.hot.as_ptr().add(c as usize) as *const i8;
                    _mm_prefetch(p, _MM_HINT_T0);
                    _mm_prefetch(p.add(64), _MM_HINT_T0);
                }
                stack.extend_from_slice(&[c, c + 1, c + 2, c + 3]);
            } else {
                // Near leaf: exact particle-particle interactions over the
                // packed member slice — contiguous, branchless (masked), and
                // free of the old next[]-chain dependent loads. Self and
                // touching pairs mask out; the clamp keeps lanes finite.
                const BALL_SQ: f32 = BALL_SIZE * BALL_SIZE;
                let start = n.leaf_start as usize;
                let end = start + n.leaf_count as usize;
                for k in start..end {
                    let dx = pos.x - self.leaf_x[k];
                    let dy = pos.y - self.leaf_y[k];
                    let d2 = dx * dx + dy * dy;
                    let m = if d2 >= BALL_SQ { 1.0f32 } else { 0.0 };
                    // active pairs have d2 ≥ BALL² > 1, so max(d2,1) == d2c
                    let d2c = d2.max(BALL_SQ);
                    let w = (scale * m) / (d2c.sqrt() * d2c);
                    force += Vec2::new(dx * w, dy * w);
                }
            }
        }

        force
    }
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

    fn build(&mut self, positions: &[Vec2]) {
        let n = positions.len();
        self.cell_of.resize(n, 0);
        self.indices.resize(n, 0);
        self.cursor.fill(0);

        for (i, p) in positions.iter().enumerate() {
            let c = Self::cell_id(*p);
            self.cell_of[i] = c;
            self.cursor[c as usize] += 1;
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
            let m = if d2 >= BALL_SQ { 1.0f32 } else { 0.0 };
            // active pairs have d2 ≥ BALL² > 1, so max(d2,1) == d2 here
            let d2c = d2.max(BALL_SQ);
            let w = (s8 * m) / (d2c.sqrt() * d2c);
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
    bh_tree: BhTree,
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
    use_barnes_hut: bool,
    use_verlet_lists: bool,
    use_adaptive_dt: bool,
    adaptive_dt: f32,
    bh_theta: f32,
    force_interval: usize,
    solver_iterations: usize,
    solver_omega: f32,
    par_min: usize,
    substeps: usize,

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
            bh_tree: BhTree::new(),
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
            use_barnes_hut: true,  // Enable Barnes-Hut by default
            use_verlet_lists: true, // Enable Verlet lists by default
            use_adaptive_dt: true,  // Enable adaptive time-stepping by default
            adaptive_dt: PHYS_TIME_STEP,
            bh_theta: BARNES_HUT_THETA,
            force_interval: FORCE_INTERVAL,
            solver_iterations: SOLVER_ITERATIONS,
            solver_omega: SOLVER_OMEGA,
            par_min: PAR_MIN_PARTICLES,
            substeps: 1,
            last_max_velocity: 0.0,
        }
    }

    /// Barnes-Hut opening angle (accuracy/speed knob; error ~ θ²).
    pub fn set_bh_theta(&mut self, theta: f32) {
        self.bh_theta = theta;
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
        ps.tree_build_time_us = 0;
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
        share.perf_stats.barnes_hut_enabled = self.use_barnes_hut;
        share.perf_stats.verlet_lists_enabled = self.use_verlet_lists;
        share.perf_stats.adaptive_dt_enabled = self.use_adaptive_dt;
        share.perf_stats.current_dt = effective_dt;
    }

    fn substep(&mut self, dt: f32, share: &mut ShareData) {
        self.integrate(dt, share);

        // Build the CSR grid once per substep; forces and all solver
        // iterations reuse it (positions move a small fraction of a cell).
        let t = Instant::now();
        self.grid.build(&share.c_pos);
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
            *c_pos = *c_pos * 2.0 - *c_opos + (GRAVITY + *c_force) * (dt * dt * INV_PHYS_DT);
            *c_opos = oldnpos;
            *c_force = Vec2::ZERO;
        }

        self.last_max_velocity = max_velocity_sq.sqrt();
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
            if self.use_barnes_hut {
                self.compute_forces_barnes_hut(c_pos, stats);
            } else {
                self.compute_forces_naive(c_pos);
            }
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

    /// Rebuild the arena tree (capacity persists across frames).
    fn build_bh_tree(&mut self, c_pos: &[Vec2]) {
        let bounds = AABB::new(
            Vec2::new(LEFT_WALL, BOTTOM_WALL),
            Vec2::new(RIGHT_WALL, TOP_WALL),
        );
        self.bh_tree.clear(bounds, c_pos.len());

        for (i, &pos) in c_pos.iter().enumerate() {
            // The old tree silently dropped out-of-bounds particles; clamping
            // keeps them contributing from the nearest edge instead.
            self.bh_tree
                .insert(i as u32, pos.clamp(bounds.min, bounds.max), 1.0);
        }
        self.bh_tree.finalize();
    }

    fn compute_forces_barnes_hut(&mut self, c_pos: &[Vec2], stats: &mut PerformanceStats) {
        let t = Instant::now();
        self.build_bh_tree(c_pos);
        stats.tree_build_time_us += t.elapsed().as_micros() as u64;
        stats.tree_node_count = self.bh_tree.hot.len();

        // Independent read-only traversals: parallelize over particles with a
        // per-thread scratch stack.
        let (tree, scale, theta) = (&self.bh_tree, self.scale, self.bh_theta);
        self.c_force
            .par_iter_mut()
            .enumerate()
            .for_each_init(
                || Vec::with_capacity(256),
                |stack, (i, f)| {
                    *f += tree.compute_force(c_pos[i], scale, theta, stack) / 8.0;
                },
            );
    }

    /// Bench/test hook: Barnes-Hut forces for `positions` at an explicit θ,
    /// same scaling as the engine's force pass. Not used by the sim loop.
    pub fn forces_barnes_hut_at(&mut self, positions: &[Vec2], theta: f32) -> Vec<Vec2> {
        self.build_bh_tree(positions);
        let (tree, scale) = (&self.bh_tree, self.scale);
        let mut stack = Vec::with_capacity(256);
        positions
            .iter()
            .map(|&p| tree.compute_force(p, scale, theta, &mut stack) / 8.0)
            .collect()
    }

    /// Bench/test hook: exact O(n²) pairwise forces with the engine's kernel
    /// and scaling — the ground truth the tree approximates.
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

    fn compute_forces_naive(&mut self, c_pos: &[Vec2]) {
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

        let interaction_range_sq =
            (BALL_SIZE * 2.0 + VERLET_SKIN_DISTANCE) * (BALL_SIZE * 2.0 + VERLET_SKIN_DISTANCE);
        for i in 0..c_pos.len() {
            lists.start[i] = lists.neighbors.len() as u32;

            let cell = self.grid.cell_of[i] as usize;
            let (x, y) = (cell % GRID_W, cell / GRID_W);

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
            // Packed parallel gather over the CSR-ordered SoA arrays.
            let s8 = self.scale / 8.0;
            self.acc_x.clear();
            self.acc_x.resize(n, 0.0);
            self.acc_y.clear();
            self.acc_y.resize(n, 0.0);

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
    /// parallel and vectorizable), then apply them in one sweep. Gathers run
    /// per *cluster* of 4 consecutive packed particles: packed order is cell
    /// order, so a cluster usually sits on one grid row and shares a union
    /// stencil — each candidate is then loaded once against 4 full SIMD
    /// lanes instead of 4 times against ~1-wide rows (stage 23). Clusters
    /// that straddle a grid row (or the tail) fall back to the scalar path.
    fn resolve_collisions_packed(&mut self) {
        let n = self.px.len();
        self.acc_x.clear();
        self.acc_x.resize(n, 0.0);
        self.acc_y.clear();
        self.acc_y.resize(n, 0.0);

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
                        ax.copy_from_slice(&gx);
                        ay.copy_from_slice(&gy);
                    } else {
                        for l in 0..m {
                            let rows =
                                stencil_rows(&grid.cell_start, cells[k0 + l] as usize);
                            let (gx, gy) =
                                gather_correction(px[k0 + l], py[k0 + l], px, py, &rows, relax);
                            ax[l] = gx;
                            ay[l] = gy;
                        }
                    }
                });
        }

        for k in 0..n {
            self.px[k] += self.acc_x[k];
            self.py[k] += self.acc_y[k];
        }
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

    pub fn toggle_barnes_hut(&mut self) {
        self.use_barnes_hut = !self.use_barnes_hut;
        println!("Barnes-Hut: {}", if self.use_barnes_hut { "ON" } else { "OFF" });
    }

    pub fn toggle_verlet_lists(&mut self) {
        self.use_verlet_lists = !self.use_verlet_lists;
        println!("Verlet Lists: {}", if self.use_verlet_lists { "ON" } else { "OFF" });
    }

    pub fn toggle_adaptive_dt(&mut self) {
        self.use_adaptive_dt = !self.use_adaptive_dt;
        println!("Adaptive dt: {}", if self.use_adaptive_dt { "ON" } else { "OFF" });
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

#[inline(always)]
fn force(pos_a: Vec2, pos_b: Vec2, scale: f32) -> Vec2 {
    let dir = pos_a - pos_b;
    let dist_sq = dir.length_squared();
    if dist_sq < BALL_SIZE * BALL_SIZE {
        return Vec2::ZERO;
    }

    // == normalize(dir) * scale / max(dist², 1), with one rsqrt total
    dir * (scale * fast_rsqrt(dist_sq) / dist_sq.max(1.0))
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
