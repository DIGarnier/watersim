// Fully GPU-resident granular simulation. wgpu compiles this WGSL to Metal
// on Apple silicon. Particles are binned into fixed-capacity cell buckets;
// the generous bucket size keeps the hot path lock-free after insertion.

struct Params {
    particle_count: u32,
    grid_w: u32,
    grid_h: u32,
    max_per_cell: u32,
    scale_over_8: f32,
    relax: f32,
    ball_size: f32,
    grid_size: f32,
    accel_dt2: f32,
    gravity_x: f32,
    gravity_y: f32,
    _pad0: f32,
    width: f32,
    height: f32,
    _pad1: f32,
    _pad2: f32,
};

@group(0) @binding(0) var<storage, read_write> positions: array<vec2<f32>>;
@group(0) @binding(1) var<storage, read_write> scratch: array<vec2<f32>>;
@group(0) @binding(2) var<storage, read_write> forces: array<vec2<f32>>;
@group(0) @binding(3) var<storage, read_write> old_positions: array<vec2<f32>>;
// The final counter is the global cell-overflow diagnostic.
@group(0) @binding(4) var<storage, read_write> cell_counts: array<atomic<u32>>;
@group(0) @binding(5) var<storage, read_write> cell_particles: array<u32>;
@group(0) @binding(6) var<uniform> params: Params;

fn cell_id(p: vec2<f32>) -> u32 {
    let x = u32(clamp(i32(p.x / params.grid_size), 0, i32(params.grid_w) - 1));
    let y = u32(clamp(i32(p.y / params.grid_size), 0, i32(params.grid_h) - 1));
    return y * params.grid_w + x;
}

@compute @workgroup_size(256)
fn clear_grid(@builtin(global_invocation_id) gid: vec3<u32>) {
    let cell_count = params.grid_w * params.grid_h;
    if (gid.x <= cell_count) { atomicStore(&cell_counts[gid.x], 0u); }
}

@compute @workgroup_size(256)
fn integrate_and_bin(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= params.particle_count) { return; }
    let p = positions[i];
    let next = 2.0 * p - old_positions[i]
        + (vec2<f32>(params.gravity_x, params.gravity_y) + forces[i]) * params.accel_dt2;
    positions[i] = next;
    old_positions[i] = p;
    forces[i] = vec2<f32>(0.0);

    let cell = cell_id(next);
    let slot = atomicAdd(&cell_counts[cell], 1u);
    if (slot < params.max_per_cell) {
        cell_particles[cell * params.max_per_cell + slot] = i;
    } else {
        atomicAdd(&cell_counts[params.grid_w * params.grid_h], 1u);
    }
}

fn taper(dist_sq: f32) -> f32 {
    let contact_sq = 4.0 * params.ball_size * params.ball_size;
    let cutoff_sq = params.grid_size * params.grid_size;
    let u = clamp((dist_sq - contact_sq) / (cutoff_sq - contact_sq), 0.0, 1.0);
    return 1.0 - u * u * (3.0 - 2.0 * u);
}

@compute @workgroup_size(256)
fn force_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= params.particle_count) { return; }
    let pi = positions[i];
    let cell = cell_id(pi);
    let cx = i32(cell % params.grid_w);
    let cy = i32(cell / params.grid_w);
    let ball_sq = params.ball_size * params.ball_size;
    let cutoff_sq = params.grid_size * params.grid_size;
    var total = vec2<f32>(0.0);
    for (var dy = -1; dy <= 1; dy = dy + 1) {
        let y = cy + dy;
        if (y < 0 || y >= i32(params.grid_h)) { continue; }
        for (var dx = -1; dx <= 1; dx = dx + 1) {
            let x = cx + dx;
            if (x < 0 || x >= i32(params.grid_w)) { continue; }
            let c = u32(y) * params.grid_w + u32(x);
            let count = min(atomicLoad(&cell_counts[c]), params.max_per_cell);
            for (var k = 0u; k < count; k = k + 1u) {
                let j = cell_particles[c * params.max_per_cell + k];
                let d = pi - positions[j];
                let d2 = dot(d, d);
                if (d2 >= ball_sq && d2 < cutoff_sq) {
                    total += d * (params.scale_over_8 * taper(d2) / (sqrt(d2) * d2));
                }
            }
        }
    }
    forces[i] = total;
}

fn collide(i: u32, from_positions: bool) {
    let pi = select(scratch[i], positions[i], from_positions);
    let cell = cell_id(pi);
    let cx = i32(cell % params.grid_w);
    let cy = i32(cell / params.grid_w);
    let contact = 2.0 * params.ball_size;
    let contact_sq = contact * contact;
    var correction = vec2<f32>(0.0);
    for (var dy = -1; dy <= 1; dy = dy + 1) {
        let y = cy + dy;
        if (y < 0 || y >= i32(params.grid_h)) { continue; }
        for (var dx = -1; dx <= 1; dx = dx + 1) {
            let x = cx + dx;
            if (x < 0 || x >= i32(params.grid_w)) { continue; }
            let c = u32(y) * params.grid_w + u32(x);
            let count = min(atomicLoad(&cell_counts[c]), params.max_per_cell);
            for (var k = 0u; k < count; k = k + 1u) {
                let j = cell_particles[c * params.max_per_cell + k];
                if (j == i) { continue; }
                let pj = select(scratch[j], positions[j], from_positions);
                let d = pi - pj;
                let d2 = dot(d, d);
                if (d2 < contact_sq && d2 > 1e-12) {
                    let dist = sqrt(d2);
                    correction -= d * (params.relax * (dist - contact) / dist);
                } else if (d2 <= 1e-12) {
                    let sign = select(-1.0, 1.0, i < j);
                    if (((i ^ j) & 1u) == 0u) {
                        correction.x += sign * params.relax * contact;
                    } else {
                        correction.y += sign * params.relax * contact;
                    }
                }
            }
        }
    }

    var p = pi + correction;
    var op = old_positions[i];
    let velocity = (p - op) * 0.4;
    let salt = 1e-5 * f32(i & 15u);
    let low = params.ball_size + 1e-5 + salt;
    let high_x = params.width - params.ball_size - 1e-5 - salt;
    let high_y = params.height - params.ball_size - 1e-5 - salt;
    if (p.x <= params.ball_size) { p.x = low; op.x = p.x + velocity.x; }
    if (p.x >= params.width - params.ball_size) { p.x = high_x; op.x = p.x + velocity.x; }
    if (p.y <= params.ball_size) { p.y = low; op.y = p.y + velocity.y; }
    if (p.y >= params.height - params.ball_size) { p.y = high_y; op.y = p.y + velocity.y; }
    old_positions[i] = op;
    if (from_positions) { scratch[i] = p; } else { positions[i] = p; }
}

@compute @workgroup_size(256)
fn collision_a_to_b(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x < params.particle_count) { collide(gid.x, true); }
}

@compute @workgroup_size(256)
fn collision_b_to_a(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x < params.particle_count) { collide(gid.x, false); }
}
