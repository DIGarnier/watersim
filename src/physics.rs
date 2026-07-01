use glam::Vec2;
use std::sync::mpsc::Receiver;
use std::time::Instant;

use crate::constants::{
    BALL_SIZE, GRID_SIZE, HEIGHT, INITIAL_BALL_SPEED_MODIFIER, WIDTH, X_LEN, Y_LEN,
};

const GRAVITY: Vec2 = Vec2::new(0.0, 9.8);
pub const PHYS_TIME_STEP: f32 = 1.0 / 480.0;

// Optimization constants
const VERLET_REBUILD_INTERVAL: usize = 10; // Rebuild neighbor lists every N frames
const VERLET_SKIN_DISTANCE: f32 = BALL_SIZE * 0.5; // Extra distance for neighbor lists
const ADAPTIVE_DT_MIN: f32 = PHYS_TIME_STEP * 0.1;
const ADAPTIVE_DT_MAX: f32 = PHYS_TIME_STEP * 2.0;
const MAX_SAFE_VELOCITY: f32 = 100.0; // Reduce dt when velocities exceed this
const BARNES_HUT_THETA: f32 = 0.5; // Barnes-Hut approximation parameter

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

// Barnes-Hut Quadtree node
struct QuadNode {
    bounds: AABB,
    center_of_mass: Vec2,
    total_mass: f32,
    particle_idx: Option<usize>,
    children: Option<Box<[QuadNode; 4]>>,
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

    fn contains(&self, point: Vec2) -> bool {
        point.x >= self.min.x && point.x <= self.max.x &&
        point.y >= self.min.y && point.y <= self.max.y
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

impl QuadNode {
    fn new(bounds: AABB) -> Self {
        Self {
            bounds,
            center_of_mass: Vec2::ZERO,
            total_mass: 0.0,
            particle_idx: None,
            children: None,
        }
    }

    fn insert(&mut self, pos: Vec2, mass: f32, idx: usize) -> bool {
        if !self.bounds.contains(pos) {
            return false;
        }

        // If this is a leaf node with no particle, store it here
        if self.particle_idx.is_none() && self.children.is_none() {
            self.particle_idx = Some(idx);
            self.center_of_mass = pos;
            self.total_mass = mass;
            return true;
        }

        // If this leaf already has a particle, subdivide
        if self.particle_idx.is_some() && self.children.is_none() {
            let old_idx = self.particle_idx.take().unwrap();
            let old_com = self.center_of_mass;
            let old_mass = self.total_mass;

            // Create children
            self.children = Some(Box::new([
                QuadNode::new(self.bounds.quadrant(0)),
                QuadNode::new(self.bounds.quadrant(1)),
                QuadNode::new(self.bounds.quadrant(2)),
                QuadNode::new(self.bounds.quadrant(3)),
            ]));

            // Re-insert old particle
            for child in self.children.as_mut().unwrap().iter_mut() {
                if child.insert(old_com, old_mass, old_idx) {
                    break;
                }
            }
        }

        // Insert new particle into appropriate child
        if let Some(ref mut children) = self.children {
            for child in children.iter_mut() {
                if child.insert(pos, mass, idx) {
                    // Update center of mass and total mass
                    let total = self.total_mass + mass;
                    self.center_of_mass = (self.center_of_mass * self.total_mass + pos * mass) / total;
                    self.total_mass = total;
                    return true;
                }
            }
        }

        false
    }

    fn compute_force(&self, pos: Vec2, scale: f32, theta: f32) -> Vec2 {
        if self.total_mass == 0.0 {
            return Vec2::ZERO;
        }

        let diff = self.center_of_mass - pos;
        let dist_sq = diff.length_squared();

        // Avoid self-interaction
        if dist_sq < 0.01 {
            return Vec2::ZERO;
        }

        let dist = dist_sq.sqrt();
        let size = self.bounds.size();

        // If far enough or is a leaf, use approximation
        if self.children.is_none() || (size / dist) < theta {
            if dist < BALL_SIZE {
                return Vec2::ZERO;
            }
            return (diff.normalize() * scale * self.total_mass) / dist_sq.max(1.0);
        }

        // Otherwise, recurse into children
        let mut force = Vec2::ZERO;
        if let Some(ref children) = self.children {
            for child in children.iter() {
                force += child.compute_force(pos, scale, theta);
            }
        }
        force
    }

    fn count_nodes(&self) -> usize {
        1 + self.children.as_ref()
            .map(|c| c.iter().map(|child| child.count_nodes()).sum())
            .unwrap_or(0)
    }
}

// Verlet neighbor list for a particle
struct NeighborList {
    neighbors: Vec<usize>,
    last_pos: Vec2,
}

impl NeighborList {
    fn new() -> Self {
        Self {
            neighbors: Vec::new(),
            last_pos: Vec2::ZERO,
        }
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
        // `as usize` saturates negatives to 0, matching the old clamping
        let x = ((pos.x / GRID_SIZE) as usize).min(GRID_W - 1);
        let y = ((pos.y / GRID_SIZE) as usize).min(GRID_H - 1);
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

fn arbitrary_vector_field(mut pos: Vec2) -> Vec2 {
    pos *= 0.01;
    let dx = pos.x.sin();
    let dy = pos.y.cos() * 10.0;
    Vec2::new(dx, dy).perp() * 0.0 // dirty way of disabling it
}

pub struct Physics {
    c_opos: Vec<Vec2>,
    c_force: Vec<Vec2>,
    grid: CsrGrid,
    pub rx: Receiver<EventToPthread>,
    pub scale: f32,

    // Optimization structures
    neighbor_lists: Vec<NeighborList>,
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
            rx,
            scale,
            neighbor_lists: Vec::new(),
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
        let mut max_velocity: f32 = 0.0;

        for i in 0..share.c_pos.len() {
            let c_pos = &mut share.c_pos[i];
            let c_opos = &mut self.c_opos[i];
            let c_color = &mut share.c_color[i];
            let c_force = &mut self.c_force[i];

            let oldnpos = *c_pos;
            let velocity = (*c_pos - *c_opos) * 20.;
            let vel_length = velocity.length();
            *c_color = (vel_length + 198.) % 360.;

            // Track maximum velocity for adaptive time-stepping
            max_velocity = max_velocity.max(vel_length);

            *c_pos =
                *c_pos * 2.0 - *c_opos + (arbitrary_vector_field(*c_pos) + GRAVITY + *c_force) * dt;
            *c_opos = oldnpos;
            *c_force = Vec2::ZERO;
        }

        self.last_max_velocity = max_velocity;
        share.perf_stats.integration_time_us = start.elapsed().as_micros() as u64;
    }

    fn check_ball_collisions(&mut self, c_pos: &mut [Vec2]) {
        let _collision_start = Instant::now();

        // Build the CSR grid once per step; forces and all solver iterations
        // reuse it (positions move a small fraction of a cell per step).
        self.grid.build(c_pos);

        // Use Barnes-Hut for force calculation if enabled
        if self.use_barnes_hut {
            self.compute_forces_barnes_hut(c_pos);
        } else {
            self.compute_forces_naive(c_pos);
        }

        // Handle collisions using constraint solver
        for _ in 0..8 {
            self.resolve_collisions(c_pos);
            self.check_wall_collisions(c_pos);
        }
    }

    fn compute_forces_barnes_hut(&mut self, c_pos: &[Vec2]) {
        // Build Barnes-Hut tree
        let bounds = AABB::new(
            Vec2::new(LEFT_WALL, BOTTOM_WALL),
            Vec2::new(RIGHT_WALL, TOP_WALL),
        );
        let mut tree = QuadNode::new(bounds);

        for (i, &pos) in c_pos.iter().enumerate() {
            tree.insert(pos, 1.0, i); // Assume unit mass for all particles
        }

        // Compute forces using the tree
        for i in 0..c_pos.len() {
            let force = tree.compute_force(c_pos[i], self.scale, BARNES_HUT_THETA);
            self.c_force[i] += force / 8.0;
        }
    }

    fn compute_forces_naive(&mut self, c_pos: &[Vec2]) {
        // Use spatial hashing as before
        if self.use_verlet_lists && self.should_rebuild_neighbors() {
            self.rebuild_neighbor_lists(c_pos);
        }

        if self.use_verlet_lists && !self.neighbor_lists.is_empty() {
            self.compute_forces_with_verlet_lists(c_pos);
        } else {
            self.compute_forces_with_spatial_hash(c_pos);
        }
    }

    fn should_rebuild_neighbors(&self) -> bool {
        self.frame_count % VERLET_REBUILD_INTERVAL == 0 ||
        self.neighbor_lists.len() != self.c_force.len()
    }

    fn rebuild_neighbor_lists(&mut self, c_pos: &[Vec2]) {
        // Resize neighbor lists if needed
        self.neighbor_lists.resize_with(c_pos.len(), NeighborList::new);

        // The step-level CSR grid is already built; gather from it.
        let interaction_range_sq =
            (BALL_SIZE * 2.0 + VERLET_SKIN_DISTANCE) * (BALL_SIZE * 2.0 + VERLET_SKIN_DISTANCE);
        for i in 0..c_pos.len() {
            self.neighbor_lists[i].neighbors.clear();
            self.neighbor_lists[i].last_pos = c_pos[i];

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
                        let j = j as usize;
                        if i != j
                            && (c_pos[i] - c_pos[j]).length_squared() < interaction_range_sq
                        {
                            self.neighbor_lists[i].neighbors.push(j);
                        }
                    }
                }
            }
        }
    }

    fn compute_forces_with_verlet_lists(&mut self, c_pos: &[Vec2]) {
        for i in 0..c_pos.len() {
            for &j in &self.neighbor_lists[i].neighbors {
                if i < j {
                    let f = force(c_pos[i], c_pos[j], self.scale) / 8.0;
                    self.c_force[i] += f;
                    self.c_force[j] -= f; // Newton's third law
                }
            }
        }
    }

    fn compute_forces_with_spatial_hash(&mut self, c_pos: &[Vec2]) {
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

    #[inline(always)]
    fn project_pair(c_pos: &mut [Vec2], i: usize, j: usize) {
        let pos_a = c_pos[i];
        let pos_b = c_pos[j];
        if ball_collides(pos_a, BALL_SIZE, pos_b, BALL_SIZE) {
            let mut col_axis = pos_a - pos_b;
            let mvt = 0.75 * (col_axis.length() - (BALL_SIZE + BALL_SIZE));
            col_axis = col_axis.try_normalize().unwrap_or(Vec2::ZERO);
            c_pos[i] -= col_axis * mvt / 2.0;
            c_pos[j] += col_axis * mvt / 2.0;
        }
    }

    fn resolve_collisions(&mut self, c_pos: &mut [Vec2]) {
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
fn ball_collides(pos_a: Vec2, scale_a: f32, pos_b: Vec2, scale_b: f32) -> bool {
    (pos_a - pos_b).length_squared() < ((scale_a + scale_b) * (scale_a + scale_b))
}

#[inline(always)]
fn force(pos_a: Vec2, pos_b: Vec2, scale: f32) -> Vec2 {
    let dir = pos_a - pos_b;
    let dist = dir.length();
    if dist < BALL_SIZE {
        return Vec2::ZERO;
    }

    (dir.normalize() * scale) / (dist * dist).max(1.0)
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
