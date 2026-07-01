// GPU Compute Shader for N-Body Physics Simulation
// This shader computes forces and integrates particle positions on the GPU

struct Particle {
    pos: vec2<f32>,
    opos: vec2<f32>,
    force: vec2<f32>,
    color: f32,
    _padding: f32,
}

struct SimulationParams {
    dt: f32,
    gravity_y: f32,
    scale: f32,
    ball_size: f32,
    particle_count: u32,
    use_barnes_hut: u32,
    _padding1: u32,
    _padding2: u32,
}

// Storage buffers
@group(0) @binding(0) var<storage, read_write> particles: array<Particle>;
@group(0) @binding(1) var<uniform> params: SimulationParams;

// Shared memory for tile-based force calculation
var<workgroup> tile_positions: array<vec2<f32>, 256>;
var<workgroup> tile_forces: array<vec2<f32>, 256>;

// Compute force between two particles
fn compute_force(pos_a: vec2<f32>, pos_b: vec2<f32>, scale: f32, ball_size: f32) -> vec2<f32> {
    let dir = pos_a - pos_b;
    let dist_sq = dot(dir, dir);

    if dist_sq < ball_size * ball_size || dist_sq < 0.01 {
        return vec2<f32>(0.0, 0.0);
    }

    let dist = sqrt(dist_sq);
    let force_mag = scale / max(dist_sq, 1.0);

    return normalize(dir) * force_mag;
}

// Force calculation kernel (tile-based for better GPU utilization)
@compute @workgroup_size(256)
fn force_calculation(@builtin(global_invocation_id) global_id: vec3<u32>,
                     @builtin(local_invocation_id) local_id: vec3<u32>,
                     @builtin(workgroup_id) group_id: vec3<u32>) {

    let tid = global_id.x;
    if tid >= params.particle_count {
        return;
    }

    let my_pos = particles[tid].pos;
    var total_force = vec2<f32>(0.0, 0.0);

    // Tile-based force calculation for better memory coalescing
    let num_tiles = (params.particle_count + 255u) / 256u;

    for (var tile = 0u; tile < num_tiles; tile++) {
        let tile_idx = tile * 256u + local_id.x;

        // Load tile into shared memory
        if tile_idx < params.particle_count {
            tile_positions[local_id.x] = particles[tile_idx].pos;
        } else {
            tile_positions[local_id.x] = vec2<f32>(0.0, 0.0);
        }

        workgroupBarrier();

        // Compute forces with all particles in this tile
        for (var i = 0u; i < 256u; i++) {
            let other_idx = tile * 256u + i;
            if other_idx < params.particle_count && other_idx != tid {
                total_force += compute_force(my_pos, tile_positions[i], params.scale, params.ball_size);
            }
        }

        workgroupBarrier();
    }

    // Store computed force
    particles[tid].force = total_force / 8.0;
}

// Integration kernel
@compute @workgroup_size(256)
fn integration(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let tid = global_id.x;
    if tid >= params.particle_count {
        return;
    }

    let pos = particles[tid].pos;
    let opos = particles[tid].opos;
    let force = particles[tid].force;

    // Compute velocity for color
    let velocity = (pos - opos) * 20.0;
    let vel_length = length(velocity);
    particles[tid].color = (vel_length + 198.0) % 360.0;

    // Verlet integration
    let gravity = vec2<f32>(0.0, params.gravity_y);
    let new_pos = pos * 2.0 - opos + (force + gravity) * params.dt;

    particles[tid].opos = pos;
    particles[tid].pos = new_pos;
    particles[tid].force = vec2<f32>(0.0, 0.0);
}

// Wall collision kernel
@compute @workgroup_size(256)
fn wall_collisions(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let tid = global_id.x;
    if tid >= params.particle_count {
        return;
    }

    let LEFT_WALL = 0.0;
    let RIGHT_WALL = 1500.0;
    let BOTTOM_WALL = 0.0;
    let TOP_WALL = 1200.0;
    let EPS = 0.00001;

    var pos = particles[tid].pos;
    var opos = particles[tid].opos;
    let ball_size = params.ball_size;

    let curr_vel = (pos - opos) * 0.4;

    // Horizontal walls
    if pos.x - ball_size <= LEFT_WALL {
        pos.x = LEFT_WALL + ball_size + EPS;
        opos.x = pos.x + curr_vel.x;
    } else if pos.x + ball_size >= RIGHT_WALL {
        pos.x = RIGHT_WALL - ball_size - EPS;
        opos.x = pos.x + curr_vel.x;
    }

    // Vertical walls
    if pos.y - ball_size <= BOTTOM_WALL {
        pos.y = BOTTOM_WALL + ball_size + EPS;
        opos.y = pos.y + curr_vel.y;
    } else if pos.y + ball_size >= TOP_WALL {
        pos.y = TOP_WALL - ball_size - EPS;
        opos.y = pos.y + curr_vel.y;
    }

    particles[tid].pos = pos;
    particles[tid].opos = opos;
}

// Barnes-Hut quadtree node (for GPU implementation)
struct QuadNode {
    center_of_mass: vec2<f32>,
    total_mass: f32,
    bounds_min: vec2<f32>,
    bounds_max: vec2<f32>,
    is_leaf: u32,
    particle_idx: u32,
    children_offset: u32,
    _padding: u32,
}

@group(1) @binding(0) var<storage, read> quadtree: array<QuadNode>;

// Barnes-Hut force calculation (more advanced, optional)
fn compute_force_barnes_hut(pos: vec2<f32>, node_idx: u32, theta: f32, scale: f32) -> vec2<f32> {
    if node_idx >= arrayLength(&quadtree) {
        return vec2<f32>(0.0, 0.0);
    }

    let node = quadtree[node_idx];

    if node.total_mass == 0.0 {
        return vec2<f32>(0.0, 0.0);
    }

    let diff = node.center_of_mass - pos;
    let dist_sq = dot(diff, diff);

    if dist_sq < 0.01 {
        return vec2<f32>(0.0, 0.0);
    }

    let dist = sqrt(dist_sq);
    let size = max(node.bounds_max.x - node.bounds_min.x, node.bounds_max.y - node.bounds_min.y);

    // If far enough or is a leaf, use approximation
    if node.is_leaf != 0u || (size / dist) < theta {
        return normalize(diff) * (scale * node.total_mass) / max(dist_sq, 1.0);
    }

    // Otherwise, recurse into children (unrolled)
    var total_force = vec2<f32>(0.0, 0.0);
    for (var i = 0u; i < 4u; i++) {
        total_force += compute_force_barnes_hut(pos, node.children_offset + i, theta, scale);
    }

    return total_force;
}
