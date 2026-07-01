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
const BARNES_HUT_THETA: f32 = 0.5; // Barnes-Hut approximation parameter

// Constraint-projection iterations per substep. The sim already substeps at
// 480 Hz (8 substeps per 60 Hz visual frame); per "Small Steps in Physics
// Simulation" (Macklin et al., SCA 2019) substeps are far more valuable than
// solver iterations, so a small count here suffices (see
// docs/benchmarks/04-small-steps.md for the measured speed/quality trade).
const SOLVER_ITERATIONS: usize = 4;

// Below this particle count the serial grid paths win. Measured on a 4-core
// Xeon: parallel force gather + colored solver are a ~2x win at 24k particles,
// within noise at 12k, and a regression at 3-6k (the solver alone spawns
// 6 colors x 4 iterations = 24 parallel regions per step, whose overhead
// dominates small workloads). The Barnes-Hut traversal has no threshold: it
// is one region of heavy independent work and wins at every tested size.
const PAR_MIN_PARTICLES: usize = 16_000;
const PAR_SOLVER_MIN_PARTICLES: usize = 16_000;

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

// Barnes-Hut quadtree node stored in a flat arena. `children` is the index of
// the first of 4 contiguous children, or BH_NONE for a leaf.
// Max particles per leaf before it subdivides. Bucketed leaves keep the tree
// shallow and turn near-field work into tight exact loops (standard treecode
// practice; see docs/literature.md §3).
const BH_LEAF_CAP: u32 = 8;

#[derive(Clone, Copy)]
struct BhNode {
    bounds: AABB,
    center_of_mass: Vec2,
    total_mass: f32,
    size_sq: f32, // bounds.size()² cached for the sqrt-free acceptance test
    children: u32, // BH_NONE = leaf, else index of first of 4 contiguous children
    first: u32,    // head of this leaf's particle list (BH_NONE when empty)
    count: u32,    // particles in this leaf
}

/// Arena-backed Barnes-Hut quadtree (Burtscher & Pingali 2011 style: contiguous
/// node storage, no per-node allocation, non-recursive traversal). The arena,
/// per-particle lists and traversal stack keep their capacity across frames,
/// so steady-state tree builds are allocation-free.
struct BhTree {
    nodes: Vec<BhNode>,
    next: Vec<u32>,  // per-particle intrusive list linking leaf members
    ppos: Vec<Vec2>, // per-particle (clamped) position as inserted
}

impl BhTree {
    fn new() -> Self {
        Self {
            nodes: Vec::new(),
            next: Vec::new(),
            ppos: Vec::new(),
        }
    }

    fn clear(&mut self, bounds: AABB, n_particles: usize) {
        self.nodes.clear();
        self.nodes.push(BhNode {
            bounds,
            center_of_mass: Vec2::ZERO,
            total_mass: 0.0,
            size_sq: bounds.size() * bounds.size(),
            children: BH_NONE,
            first: BH_NONE,
            count: 0,
        });
        self.next.resize(n_particles, BH_NONE);
        self.ppos.resize(n_particles, Vec2::ZERO);
    }

    #[inline(always)]
    fn quadrant_index(bounds: &AABB, pos: Vec2) -> usize {
        let c = bounds.center();
        ((pos.y >= c.y) as usize) * 2 + ((pos.x >= c.x) as usize)
    }

    fn insert(&mut self, idx: u32, pos: Vec2, mass: f32) {
        // Below this cell size, keep piling particles into the leaf instead of
        // subdividing forever (guards against (near-)coincident particles).
        const MIN_CELL: f32 = 1e-3;

        self.ppos[idx as usize] = pos;
        let mut node = 0usize;
        loop {
            let n = self.nodes[node];

            if n.children != BH_NONE {
                // Internal node: fold the new mass into the aggregate, descend.
                let total = n.total_mass + mass;
                self.nodes[node].center_of_mass =
                    (n.center_of_mass * n.total_mass + pos * mass) / total;
                self.nodes[node].total_mass = total;
                node = n.children as usize + Self::quadrant_index(&n.bounds, pos);
                continue;
            }

            if n.count < BH_LEAF_CAP || n.bounds.size() < MIN_CELL {
                // Leaf with room: link the particle in and update the aggregate.
                self.next[idx as usize] = n.first;
                let total = n.total_mass + mass;
                let nn = &mut self.nodes[node];
                nn.first = idx;
                nn.count = n.count + 1;
                nn.center_of_mass = (n.center_of_mass * n.total_mass + pos * mass) / total;
                nn.total_mass = total;
                return;
            }

            // Full leaf: subdivide and redistribute the residents, then retry
            // this node (now internal) with the incoming particle.
            let first_child = self.nodes.len() as u32;
            for q in 0..4 {
                let qb = n.bounds.quadrant(q);
                self.nodes.push(BhNode {
                    bounds: qb,
                    center_of_mass: Vec2::ZERO,
                    total_mass: 0.0,
                    size_sq: qb.size() * qb.size(),
                    children: BH_NONE,
                    first: BH_NONE,
                    count: 0,
                });
            }

            let mut p = n.first;
            let per_particle_mass = n.total_mass / n.count as f32;
            while p != BH_NONE {
                let p_next = self.next[p as usize];
                let p_pos = self.ppos[p as usize];
                let child =
                    first_child as usize + Self::quadrant_index(&n.bounds, p_pos);
                let c = self.nodes[child];
                self.next[p as usize] = c.first;
                let total = c.total_mass + per_particle_mass;
                let cn = &mut self.nodes[child];
                cn.first = p;
                cn.count = c.count + 1;
                cn.center_of_mass =
                    (c.center_of_mass * c.total_mass + p_pos * per_particle_mass) / total;
                cn.total_mass = total;
                p = p_next;
            }

            let nn = &mut self.nodes[node];
            nn.children = first_child;
            nn.first = BH_NONE;
            nn.count = 0;
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
            let n = &self.nodes[idx as usize];
            if n.total_mass == 0.0 {
                continue;
            }

            let diff = n.center_of_mass - pos;
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
                stack.extend_from_slice(&[c, c + 1, c + 2, c + 3]);
            } else {
                // Near leaf: exact particle-particle interactions.
                let mut p = n.first;
                while p != BH_NONE {
                    let diff = self.ppos[p as usize] - pos;
                    let d_sq = diff.length_squared();
                    // Skip self-interaction and touching particles
                    if d_sq >= BALL_SIZE * BALL_SIZE {
                        force += diff * (scale * fast_rsqrt(d_sq) / d_sq.max(1.0));
                    }
                    p = self.next[p as usize];
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
    // Occupied cells split into 6 color classes for the parallel solver.
    // Colors are (x mod 3) + 3·(y mod 2): with the forward half-stencil
    // {E, SW, S, SE}, a cell's write region spans x±1 and y..y+1, so two
    // same-color cells (Δx ≥ 3 or Δy ≥ 2) always touch disjoint particles.
    color_buckets: [Vec<u32>; 6],
}

impl CsrGrid {
    fn new() -> Self {
        Self {
            cell_start: vec![0; N_CELLS + 1],
            cursor: vec![0; N_CELLS],
            cell_of: Vec::new(),
            indices: Vec::new(),
            occupied: Vec::new(),
            color_buckets: Default::default(),
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
        for bucket in &mut self.color_buckets {
            bucket.clear();
        }
        for c in 0..N_CELLS {
            let count = self.cursor[c];
            self.cell_start[c] = sum;
            self.cursor[c] = sum;
            if count > 0 {
                self.occupied.push(c as u32);
                let (x, y) = (c % GRID_W, c / GRID_W);
                self.color_buckets[(x % 3) + 3 * (y % 2)].push(c as u32);
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

/// Raw pointer that rayon closures may capture. Every use is guarded by a
/// partitioning argument (cell coloring / per-cell ownership) documented at
/// the use site.
#[derive(Clone, Copy)]
struct SendPtr<T>(*mut T);
unsafe impl<T> Send for SendPtr<T> {}
unsafe impl<T> Sync for SendPtr<T> {}

impl<T> SendPtr<T> {
    // Accessor (rather than field access) so closures capture the whole
    // wrapper, not the raw pointer, which would defeat the Send/Sync impls.
    #[inline(always)]
    fn get(self) -> *mut T {
        self.0
    }
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
    frame_count: usize,
    use_barnes_hut: bool,
    use_verlet_lists: bool,
    use_adaptive_dt: bool,
    adaptive_dt: f32,

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
            frame_count: 0,
            use_barnes_hut: true,  // Enable Barnes-Hut by default
            use_verlet_lists: true, // Enable Verlet lists by default
            use_adaptive_dt: true,  // Enable adaptive time-stepping by default
            adaptive_dt: PHYS_TIME_STEP,
            last_max_velocity: 0.0,
        }
    }

    pub fn step(&mut self, dt: f32, share: &mut ShareData) {
        // Adaptive time-stepping
        let effective_dt = if self.use_adaptive_dt {
            self.compute_adaptive_dt()
        } else {
            dt
        };

        self.integrate(effective_dt, share);

        // Build the CSR grid once per step; forces and all solver iterations
        // reuse it (positions move a small fraction of a cell per step).
        self.grid.build(&share.c_pos);

        if self.frame_count % REORDER_INTERVAL == 0 {
            self.reorder_particles(share);
        }

        self.check_ball_collisions(&mut share.c_pos);

        self.frame_count += 1;

        // Update performance stats
        share.perf_stats.total_particles = share.c_pos.len();
        share.perf_stats.barnes_hut_enabled = self.use_barnes_hut;
        share.perf_stats.verlet_lists_enabled = self.use_verlet_lists;
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

            // NOTE: the old code also added arbitrary_vector_field(pos) here,
            // which is defined to return exactly ZERO (`* 0.0`) but still paid
            // a sin + cos per particle per substep.
            *c_pos = *c_pos * 2.0 - *c_opos + (GRAVITY + *c_force) * dt;
            *c_opos = oldnpos;
            *c_force = Vec2::ZERO;
        }

        self.last_max_velocity = max_velocity_sq.sqrt();
        share.perf_stats.integration_time_us = start.elapsed().as_micros() as u64;
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

    fn check_ball_collisions(&mut self, c_pos: &mut [Vec2]) {
        let _collision_start = Instant::now();

        // Use Barnes-Hut for force calculation if enabled
        if self.use_barnes_hut {
            self.compute_forces_barnes_hut(c_pos);
        } else {
            self.compute_forces_naive(c_pos);
        }

        // Handle collisions using constraint solver
        for _ in 0..SOLVER_ITERATIONS {
            self.resolve_collisions(c_pos);
            self.check_wall_collisions(c_pos);
        }
    }

    fn compute_forces_barnes_hut(&mut self, c_pos: &[Vec2]) {
        // Rebuild the arena tree (capacity persists across frames)
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

        // Independent read-only traversals: parallelize over particles with a
        // per-thread scratch stack.
        let (tree, scale) = (&self.bh_tree, self.scale);
        self.c_force
            .par_iter_mut()
            .enumerate()
            .for_each_init(
                || Vec::with_capacity(256),
                |stack, (i, f)| {
                    *f += tree.compute_force(c_pos[i], scale, BARNES_HUT_THETA, stack) / 8.0;
                },
            );
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
        if c_pos.len() >= PAR_MIN_PARTICLES {
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
        if c_pos.len() >= PAR_MIN_PARTICLES {
            // Parallel over occupied cells: each cell gathers forces for its
            // own particles from the full 3×3 neighborhood, so force writes
            // are disjoint across cells.
            let (grid, scale) = (&self.grid, self.scale);
            let c_force = SendPtr(self.c_force.as_mut_ptr());
            grid.occupied.par_iter().for_each(|&cell| {
                let cell = cell as usize;
                let (x, y) = (cell % GRID_W, cell / GRID_W);
                for &i in grid.cell(cell) {
                    let i = i as usize;
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
                                let j = j as usize;
                                if i != j {
                                    acc += force(c_pos[i], c_pos[j], scale);
                                }
                            }
                        }
                    }
                    // Safety: cell membership partitions particles, so no other
                    // thread writes c_force[i].
                    unsafe { *c_force.get().add(i) += acc / 8.0 };
                }
            });
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

    /// Half the positional correction for an overlapping pair, or None for
    /// the common non-overlapping case — callers must skip the position
    /// stores entirely then (most checked pairs don't collide, and the write
    /// traffic of applying a ZERO correction measurably costs). One rsqrt
    /// replaces the old length() + try_normalize() (two sqrts).
    #[inline(always)]
    fn pair_correction(pos_a: Vec2, pos_b: Vec2) -> Option<Vec2> {
        const CONTACT: f32 = BALL_SIZE + BALL_SIZE;
        let col_axis = pos_a - pos_b;
        let dist_sq = col_axis.length_squared();
        // Degenerate coincident pair: old try_normalize() produced ZERO.
        if dist_sq >= CONTACT * CONTACT || dist_sq < 1e-12 {
            return None;
        }
        let inv_dist = fast_rsqrt(dist_sq);
        let dist = dist_sq * inv_dist;
        // == normalized(col_axis) * 0.75 * (dist - CONTACT) / 2
        Some(col_axis * (inv_dist * 0.375 * (dist - CONTACT)))
    }

    /// Safety: caller must guarantee no other thread concurrently accesses
    /// positions i, j (ensured by the 6-coloring of cells).
    #[inline(always)]
    unsafe fn project_pair_raw(p: *mut Vec2, i: usize, j: usize) {
        if let Some(corr) = Self::pair_correction(*p.add(i), *p.add(j)) {
            *p.add(i) -= corr;
            *p.add(j) += corr;
        }
    }

    #[inline(always)]
    fn project_pair(c_pos: &mut [Vec2], i: usize, j: usize) {
        if let Some(corr) = Self::pair_correction(c_pos[i], c_pos[j]) {
            c_pos[i] -= corr;
            c_pos[j] += corr;
        }
    }

    fn resolve_collisions(&mut self, c_pos: &mut [Vec2]) {
        if c_pos.len() >= PAR_SOLVER_MIN_PARTICLES {
            // Parallel constraint projection via cell coloring: the 6 color
            // classes are processed sequentially, cells within a class in
            // parallel — same-color cells touch disjoint particles (see
            // CsrGrid::color_buckets), so the raw-pointer writes never race.
            let grid = &self.grid;
            let pos_ptr = SendPtr(c_pos.as_mut_ptr());
            for bucket in &grid.color_buckets {
                bucket.par_iter().for_each(|&cell| {
                    let cell = cell as usize;
                    let (x, y) = (cell % GRID_W, cell / GRID_W);
                    let currents = grid.cell(cell);
                    let p = pos_ptr.get();
                    unsafe {
                        for a in 0..currents.len() {
                            let i = currents[a] as usize;
                            for b in a + 1..currents.len() {
                                Self::project_pair_raw(p, i, currents[b] as usize);
                            }
                        }
                        for (dx, dy) in [(1i32, 0i32), (-1, 1), (0, 1), (1, 1)] {
                            let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                            if nx < 0 || nx >= GRID_W as i32 || ny >= GRID_H as i32 {
                                continue;
                            }
                            for &i in currents {
                                for &j in grid.cell(ny as usize * GRID_W + nx as usize) {
                                    Self::project_pair_raw(p, i as usize, j as usize);
                                }
                            }
                        }
                    }
                });
            }
            return;
        }

        // Sweep only occupied cells of the step-level CSR grid, visiting each
        // unordered cell pair once via the forward half-stencil. Reusing the
        // grid across solver iterations is safe: a particle moves far less
        // than one 10 px cell within a single 1/480 s step, so the one-cell
        // stencil still finds every potentially overlapping pair.
        for oc in 0..self.grid.occupied.len() {
            let cell = self.grid.occupied[oc] as usize;
            let (x, y) = (cell % GRID_W, cell / GRID_W);
            let currents = self.grid.cell(cell);

            for a in 0..currents.len() {
                let i = currents[a] as usize;
                for b in a + 1..currents.len() {
                    Self::project_pair(c_pos, i, currents[b] as usize);
                }
            }

            for (dx, dy) in [(1i32, 0i32), (-1, 1), (0, 1), (1, 1)] {
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if nx < 0 || nx >= GRID_W as i32 || ny >= GRID_H as i32 {
                    continue;
                }
                for &i in currents {
                    for &j in self.grid.cell(ny as usize * GRID_W + nx as usize) {
                        Self::project_pair(c_pos, i as usize, j as usize);
                    }
                }
            }
        }
    }

    fn check_wall_collisions(&mut self, c_pos: &mut [Vec2]) {
        // Left and right border columns
        for y in 0..GRID_H {
            for &i in self.grid.cell(y * GRID_W) {
                resolve_wall_collision(&mut c_pos[i as usize], &mut self.c_opos[i as usize]);
            }
            for &i in self.grid.cell(y * GRID_W + (GRID_W - 1)) {
                resolve_wall_collision(&mut c_pos[i as usize], &mut self.c_opos[i as usize]);
            }
        }

        // Bottom and top border rows
        for x in 0..GRID_W {
            for &i in self.grid.cell(x) {
                resolve_wall_collision(&mut c_pos[i as usize], &mut self.c_opos[i as usize]);
            }
            for &i in self.grid.cell((GRID_H - 1) * GRID_W + x) {
                resolve_wall_collision(&mut c_pos[i as usize], &mut self.c_opos[i as usize]);
            }
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

fn resolve_wall_collision(c_pos: &mut Vec2, c_opos: &mut Vec2) {
    let (hor, ver) = collides_wall(*c_pos, BALL_SIZE);

    let curr_vel = (*c_pos - *c_opos) * 0.4;
    const EPS: f32 = 0.00001;
    use Collision::*;
    match hor {
        Some(Left) => {
            c_pos.x = LEFT_WALL + BALL_SIZE + EPS;
            c_opos.x = c_pos.x + curr_vel.x;
        }
        Some(Right) => {
            c_pos.x = RIGHT_WALL - BALL_SIZE - EPS;
            c_opos.x = c_pos.x + curr_vel.x;
        }
        _ => {}
    }

    match ver {
        Some(Bottom) => {
            c_pos.y = BOTTOM_WALL + BALL_SIZE + EPS;
            c_opos.y = c_pos.y + curr_vel.y;
        }
        Some(Top) => {
            c_pos.y = TOP_WALL - BALL_SIZE - EPS;
            c_opos.y = c_pos.y + curr_vel.y;
        }
        _ => {}
    }
}
