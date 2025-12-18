use ggez::glam::Vec2;
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

    fn needs_rebuild(&self, current_pos: Vec2) -> bool {
        (current_pos - self.last_pos).length() > VERLET_SKIN_DISTANCE * 0.5
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
    table: Vec<Vec<usize>>,
    others: Vec<usize>,
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
        table: Vec<Vec<usize>>,
        others: Vec<usize>,
        rx: Receiver<EventToPthread>,
        scale: f32,
    ) -> Self {
        Self {
            c_opos,
            c_force,
            table,
            others,
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
        let step_start = Instant::now();

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
        let mut max_velocity = 0.0;

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
        let collision_start = Instant::now();

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

        let collision_time = collision_start.elapsed();
    }

    fn compute_forces_barnes_hut(&mut self, c_pos: &[Vec2]) {
        let tree_start = Instant::now();

        // Build Barnes-Hut tree
        let bounds = AABB::new(
            Vec2::new(LEFT_WALL, BOTTOM_WALL),
            Vec2::new(RIGHT_WALL, TOP_WALL),
        );
        let mut tree = QuadNode::new(bounds);

        for (i, &pos) in c_pos.iter().enumerate() {
            tree.insert(pos, 1.0, i); // Assume unit mass for all particles
        }

        let tree_build_time = tree_start.elapsed();

        // Compute forces using the tree
        let force_start = Instant::now();
        for i in 0..c_pos.len() {
            let force = tree.compute_force(c_pos[i], self.scale, BARNES_HUT_THETA);
            self.c_force[i] += force / 8.0;
        }

        // Note: Performance stats will be set in a mutable ShareData later
    }

    fn compute_forces_naive(&mut self, c_pos: &[Vec2]) {
        let force_start = Instant::now();

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
        let rebuild_start = Instant::now();

        // Resize neighbor lists if needed
        self.neighbor_lists.resize_with(c_pos.len(), NeighborList::new);

        // Build spatial hash for neighbor finding
        for i in 0..(X_LEN * Y_LEN) as usize {
            self.table[i].clear();
        }

        for (i, pos_a) in c_pos.iter().enumerate() {
            let y = (pos_a.y / GRID_SIZE).min(Y_LEN - 1.0) as usize;
            let x = (pos_a.x / GRID_SIZE).min(X_LEN - 1.0) as usize;
            self.table[y * X_LEN as usize + x].push(i);
        }

        // Build neighbor lists
        let interaction_range = BALL_SIZE * 2.0 + VERLET_SKIN_DISTANCE;
        for i in 0..c_pos.len() {
            self.neighbor_lists[i].neighbors.clear();
            self.neighbor_lists[i].last_pos = c_pos[i];

            let y = (c_pos[i].y / GRID_SIZE).min(Y_LEN - 1.0) as usize;
            let x = (c_pos[i].x / GRID_SIZE).min(X_LEN - 1.0) as usize;

            // Check neighboring cells
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let ny = (y as i32 + dy).clamp(0, (Y_LEN - 1.0) as i32) as usize;
                    let nx = (x as i32 + dx).clamp(0, (X_LEN - 1.0) as i32) as usize;

                    for &j in &self.table[ny * X_LEN as usize + nx] {
                        if i != j && (c_pos[i] - c_pos[j]).length() < interaction_range {
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
        // Original spatial hash approach
        for i in 0..(X_LEN * Y_LEN) as usize {
            self.table[i].clear();
        }

        for (i, pos_a) in c_pos.iter().enumerate() {
            let y = (pos_a.y / GRID_SIZE).min(Y_LEN - 1.0) as usize;
            let x = (pos_a.x / GRID_SIZE).min(X_LEN - 1.0) as usize;
            self.table[y * X_LEN as usize + x].push(i);
        }

        for y in 0..(Y_LEN) as usize {
            for x in 0..(X_LEN) as usize {
                self.others.clear();
                let currents = &self.table[y * X_LEN as usize + x];

                for dy in [-1, 0, 1] {
                    for dx in [-1, 0, 1] {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let ny = (y as i32 + dy).clamp(0, Y_LEN as i32) as usize;
                        let nx = (x as i32 + dx).clamp(0, X_LEN as i32) as usize;
                        self.others.extend(&self.table[ny * X_LEN as usize + nx]);
                    }
                }

                for i in 0..currents.len() {
                    let pos_a = c_pos[currents[i]];
                    for j in i + 1..currents.len() {
                        let pos_b = c_pos[currents[j]];
                        self.c_force[currents[i]] += force(pos_a, pos_b, self.scale) / 8.0;
                    }

                    for j in 0..self.others.len() {
                        let pos_b = c_pos[self.others[j]];
                        if pos_a != pos_b {
                            self.c_force[currents[i]] += force(pos_a, pos_b, self.scale) / 8.0;
                        }
                    }
                }
            }
        }
    }

    fn resolve_collisions(&mut self, c_pos: &mut [Vec2]) {
        // Rebuild spatial hash for collision detection
        for i in 0..(X_LEN * Y_LEN) as usize {
            self.table[i].clear();
        }

        for (i, pos_a) in c_pos.iter().enumerate() {
            let y = (pos_a.y / GRID_SIZE).min(Y_LEN - 1.0) as usize;
            let x = (pos_a.x / GRID_SIZE).min(X_LEN - 1.0) as usize;
            self.table[y * X_LEN as usize + x].push(i);
        }

        for y in 0..(Y_LEN) as usize {
            for x in 0..(X_LEN) as usize {
                self.others.clear();
                let currents = &self.table[y * X_LEN as usize + x];

                for dy in [-1, 0, 1] {
                    for dx in [-1, 0, 1] {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let ny = (y as i32 + dy).clamp(0, Y_LEN as i32) as usize;
                        let nx = (x as i32 + dx).clamp(0, X_LEN as i32) as usize;
                        self.others.extend(&self.table[ny * X_LEN as usize + nx]);
                    }
                }

                for i in 0..currents.len() {
                    let pos_a = c_pos[currents[i]];
                    for j in i + 1..currents.len() {
                        let pos_b = c_pos[currents[j]];

                        if ball_collides(pos_a, BALL_SIZE, pos_b, BALL_SIZE) {
                            let mut col_axis = pos_a - pos_b;
                            let mvt = 0.75 * (col_axis.length() - (BALL_SIZE + BALL_SIZE));
                            col_axis = col_axis
                                .try_normalize()
                                .unwrap_or_else(|| Vec2::new(0., 0.));
                            c_pos[currents[i]] -= col_axis * mvt / 2.0;
                            c_pos[currents[j]] += col_axis * mvt / 2.0;
                        }
                    }

                    for j in 0..self.others.len() {
                        let pos_b = c_pos[self.others[j]];

                        if pos_a == pos_b {
                            continue;
                        }

                        if ball_collides(pos_a, BALL_SIZE, pos_b, BALL_SIZE) {
                            let mut col_axis = pos_a - pos_b;
                            let mvt = 0.75 * (col_axis.length() - (BALL_SIZE + BALL_SIZE));
                            col_axis = col_axis
                                .try_normalize()
                                .unwrap_or_else(|| Vec2::new(0., 0.));
                            c_pos[currents[i]] -= col_axis * mvt / 2.0;
                            c_pos[self.others[j]] += col_axis * mvt / 2.0;
                        }
                    }
                }
            }
        }
    }

    fn check_wall_collisions(&mut self, c_pos: &mut [Vec2]) {
        for y in 0..(Y_LEN) as usize {
            for &i in self.table[y * X_LEN as usize].iter() {
                resolve_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }

        for y in 0..(Y_LEN) as usize {
            for &i in self.table[y * X_LEN as usize + (X_LEN - 1.0) as usize].iter() {
                resolve_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }

        for x in 0..(X_LEN) as usize {
            for &i in self.table[x].iter() {
                resolve_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }

        for x in 0..(X_LEN) as usize {
            for &i in self.table[((Y_LEN - 1.0) * X_LEN) as usize + x].iter() {
                resolve_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
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
