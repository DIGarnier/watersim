// Shared compute + rendering shader for the zero-readback live path.

struct Particle {
    position: vec2<f32>,
    old_position: vec2<f32>,
    force: vec2<f32>,
    scratch: vec2<f32>,
};

struct Grid {
    counts: array<atomic<u32>, 32001>,
    particles: array<u32, 2048000>,
};

struct SimParams {
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

@group(0) @binding(0) var<storage, read_write> state: array<Particle>;
@group(0) @binding(1) var<storage, read_write> grid: Grid;
@group(0) @binding(2) var<uniform> sim: SimParams;

fn cell_id(p: vec2<f32>) -> u32 {
    let x = u32(clamp(i32(p.x / sim.grid_size), 0, i32(sim.grid_w) - 1));
    let y = u32(clamp(i32(p.y / sim.grid_size), 0, i32(sim.grid_h) - 1));
    return y * sim.grid_w + x;
}

@compute @workgroup_size(256)
fn clear_grid(@builtin(global_invocation_id) gid: vec3<u32>) {
    let count = sim.grid_w * sim.grid_h;
    if (gid.x <= count) { atomicStore(&grid.counts[gid.x], 0u); }
}

@compute @workgroup_size(256)
fn integrate_and_bin(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= sim.particle_count) { return; }
    let p = state[i].position;
    let next = 2.0 * p - state[i].old_position
        + (vec2<f32>(sim.gravity_x, sim.gravity_y) + state[i].force) * sim.accel_dt2;
    state[i].position = next;
    state[i].old_position = p;
    state[i].force = vec2<f32>(0.0);
    let cell = cell_id(next);
    let slot = atomicAdd(&grid.counts[cell], 1u);
    if (slot < sim.max_per_cell) {
        grid.particles[cell * sim.max_per_cell + slot] = i;
    } else {
        atomicAdd(&grid.counts[sim.grid_w * sim.grid_h], 1u);
    }
}

fn taper(d2: f32) -> f32 {
    let contact2 = 4.0 * sim.ball_size * sim.ball_size;
    let cutoff2 = sim.grid_size * sim.grid_size;
    let u = clamp((d2 - contact2) / (cutoff2 - contact2), 0.0, 1.0);
    return 1.0 - u * u * (3.0 - 2.0 * u);
}

@compute @workgroup_size(256)
fn force_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= sim.particle_count) { return; }
    let pi = state[i].position;
    let cell = cell_id(pi);
    let cx = i32(cell % sim.grid_w);
    let cy = i32(cell / sim.grid_w);
    let ball2 = sim.ball_size * sim.ball_size;
    let cutoff2 = sim.grid_size * sim.grid_size;
    var total = vec2<f32>(0.0);
    for (var dy = -1; dy <= 1; dy = dy + 1) {
        let y = cy + dy;
        if (y < 0 || y >= i32(sim.grid_h)) { continue; }
        for (var dx = -1; dx <= 1; dx = dx + 1) {
            let x = cx + dx;
            if (x < 0 || x >= i32(sim.grid_w)) { continue; }
            let c = u32(y) * sim.grid_w + u32(x);
            let count = min(atomicLoad(&grid.counts[c]), sim.max_per_cell);
            for (var k = 0u; k < count; k = k + 1u) {
                let j = grid.particles[c * sim.max_per_cell + k];
                let d = pi - state[j].position;
                let d2 = dot(d, d);
                if (d2 >= ball2 && d2 < cutoff2) {
                    total += d * (sim.scale_over_8 * taper(d2) / (sqrt(d2) * d2));
                }
            }
        }
    }
    state[i].force = total;
}

fn collide(i: u32, from_position: bool) {
    let pi = select(state[i].scratch, state[i].position, from_position);
    let cell = cell_id(pi);
    let cx = i32(cell % sim.grid_w);
    let cy = i32(cell / sim.grid_w);
    let contact = 2.0 * sim.ball_size;
    var correction = vec2<f32>(0.0);
    for (var dy = -1; dy <= 1; dy = dy + 1) {
        let y = cy + dy;
        if (y < 0 || y >= i32(sim.grid_h)) { continue; }
        for (var dx = -1; dx <= 1; dx = dx + 1) {
            let x = cx + dx;
            if (x < 0 || x >= i32(sim.grid_w)) { continue; }
            let c = u32(y) * sim.grid_w + u32(x);
            let count = min(atomicLoad(&grid.counts[c]), sim.max_per_cell);
            for (var k = 0u; k < count; k = k + 1u) {
                let j = grid.particles[c * sim.max_per_cell + k];
                if (j == i) { continue; }
                let pj = select(state[j].scratch, state[j].position, from_position);
                let d = pi - pj;
                let d2 = dot(d, d);
                if (d2 < contact * contact && d2 > 1e-12) {
                    let dist = sqrt(d2);
                    correction -= d * (sim.relax * (dist - contact) / dist);
                } else if (d2 <= 1e-12) {
                    let sign = select(-1.0, 1.0, i < j);
                    if (((i ^ j) & 1u) == 0u) { correction.x += sign * sim.relax * contact; }
                    else { correction.y += sign * sim.relax * contact; }
                }
            }
        }
    }
    var p = pi + correction;
    var op = state[i].old_position;
    let velocity = (p - op) * 0.4;
    let salt = 1e-5 * f32(i & 15u);
    let low = sim.ball_size + 1e-5 + salt;
    let high_x = sim.width - sim.ball_size - 1e-5 - salt;
    let high_y = sim.height - sim.ball_size - 1e-5 - salt;
    if (p.x <= sim.ball_size) { p.x = low; op.x = p.x + velocity.x; }
    if (p.x >= sim.width - sim.ball_size) { p.x = high_x; op.x = p.x + velocity.x; }
    if (p.y <= sim.ball_size) { p.y = low; op.y = p.y + velocity.y; }
    if (p.y >= sim.height - sim.ball_size) { p.y = high_y; op.y = p.y + velocity.y; }
    state[i].old_position = op;
    if (from_position) { state[i].scratch = p; } else { state[i].position = p; }
}

@compute @workgroup_size(256)
fn collision_ab(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x < sim.particle_count) { collide(gid.x, true); }
}
@compute @workgroup_size(256)
fn collision_ba(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x < sim.particle_count) { collide(gid.x, false); }
}
@compute @workgroup_size(256)
fn finalize_collision(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x < sim.particle_count) { state[gid.x].position = state[gid.x].scratch; }
}

struct RenderParams { width: f32, height: f32, radius: f32, _pad: f32 };
@group(0) @binding(0) var<storage, read> render_state: array<Particle>;
@group(0) @binding(1) var<uniform> render: RenderParams;

struct VertexOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) local: vec2<f32>,
    @location(1) speed: f32,
};

@vertex
fn particle_vertex(@builtin(vertex_index) vertex: u32,
                   @builtin(instance_index) instance: u32) -> VertexOut {
    // Four-vertex triangle strip. Avoid dynamically indexing a local array so
    // this also validates on ggez's older Naga/WGSL frontend.
    let local = vec2<f32>(
        select(-1.0, 1.0, vertex == 1u || vertex == 3u),
        select(-1.0, 1.0, vertex >= 2u));
    let p = render_state[instance].position + local * render.radius;
    var out: VertexOut;
    out.clip = vec4<f32>(p.x / render.width * 2.0 - 1.0,
                         1.0 - p.y / render.height * 2.0, 0.0, 1.0);
    out.local = local;
    out.speed = length((render_state[instance].position - render_state[instance].old_position) * 20.0);
    return out;
}

fn hue_rgb(h: f32) -> vec3<f32> {
    let k = vec3<f32>(0.0, 4.0, 2.0);
    return clamp(abs(fract(h + k / 6.0) * 6.0 - 3.0) - 1.0, vec3<f32>(0.0), vec3<f32>(1.0));
}

@fragment
fn particle_fragment(in: VertexOut) -> @location(0) vec4<f32> {
    let r2 = dot(in.local, in.local);
    if (r2 > 1.0) { discard; }
    let hue = fract((in.speed + 198.0) / 360.0);
    let rgb = (hue_rgb(hue) - 0.5) * 0.75 + 0.5;
    let edge = 1.0 - smoothstep(0.82, 1.0, sqrt(r2));
    return vec4<f32>(rgb, edge);
}
