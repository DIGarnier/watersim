// Massive-particle path: 64 bits/particle (16-bit normalized position and
// velocity), GPU-only initialization, and temporally interleaved updates.

struct MassiveParams {
    particle_count: u32,
    substeps: u32,
    update_stride: u32,
    update_phase: u32,
    width: f32,
    height: f32,
    time: f32,
    flow_strength: f32,
    pointer_x: f32,
    pointer_y: f32,
    impulse_x: f32,
    impulse_y: f32,
    render_stride: u32,
    render_phase: u32,
    _pad0: u32,
    _pad1: u32,
};

const PARTICLES_PER_SHARD: u32 = 16777216u;
@group(0) @binding(0) var<storage, read_write> state_a: array<vec2<u32>>;
@group(0) @binding(1) var<storage, read_write> state_b: array<vec2<u32>>;
@group(0) @binding(2) var<uniform> sim: MassiveParams;

fn load_particle(i: u32) -> vec2<u32> {
    if (i < PARTICLES_PER_SHARD) { return state_a[i]; }
    return state_b[i - PARTICLES_PER_SHARD];
}

fn store_particle(i: u32, value: vec2<u32>) {
    if (i < PARTICLES_PER_SHARD) { state_a[i] = value; }
    else { state_b[i - PARTICLES_PER_SHARD] = value; }
}

fn hash11(x: f32) -> f32 {
    return fract(sin(x * 12.9898) * 43758.5453);
}

@compute @workgroup_size(256)
fn init_particles(@builtin(global_invocation_id) gid: vec3<u32>,
                  @builtin(workgroup_id) workgroup: vec3<u32>) {
    // A single dispatch dimension is capped at 65,535 workgroups. Fold the Y
    // workgroup coordinate into a linear id for full 128 MiB buffer launches.
    let i = gid.x + workgroup.y * 65535u * 256u;
    if (i >= sim.particle_count) { return; }
    let columns = max(1u, u32(ceil(sqrt(f32(sim.particle_count) * sim.width / sim.height))));
    let rows = max(1u, (sim.particle_count + columns - 1u) / columns);
    let cell = vec2<f32>(
        (f32(i % columns) + 0.5) / f32(columns),
        (f32(i / columns) + 0.5) / f32(rows));
    let jitter = vec2<f32>(hash11(f32(i) + 3.1), hash11(f32(i) + 9.7)) - 0.5;
    let normalized = clamp(cell + jitter / vec2<f32>(f32(columns), f32(rows)) * 0.7,
                           vec2<f32>(0.0), vec2<f32>(1.0));
    store_particle(i, vec2<u32>(pack2x16unorm(normalized), pack2x16snorm(vec2<f32>(0.0))));
}

@compute @workgroup_size(256)
fn flow_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x * sim.update_stride + sim.update_phase;
    if (i >= sim.particle_count) { return; }

    let packed = load_particle(i);
    var p = unpack2x16unorm(packed.x) * vec2<f32>(sim.width, sim.height);
    var v = unpack2x16snorm(packed.y) * 4.0;
    let identity = f32(i);
    let phase = hash11(identity + 0.17) * 6.2831853;
    let drag = 0.965 + hash11(identity + 7.31) * 0.02;

    for (var tick = 0u; tick < sim.substeps; tick = tick + 1u) {
        let t = sim.time + f32(tick) * 0.008333333;
        let center = vec2<f32>(
            sim.width * (0.5 + 0.22 * sin(t * 0.37)),
            sim.height * (0.48 + 0.18 * cos(t * 0.29)));
        let delta = p - center;
        let radius = max(length(delta), 1.0);
        let tangent = vec2<f32>(-delta.y, delta.x) / radius;
        let vortex = tangent * (0.035 / (1.0 + radius * 0.003));
        let waves = vec2<f32>(
            sin(p.y * 0.012 + t * 1.7 + phase),
            cos(p.x * 0.009 - t * 1.3 + phase * 0.7)) * 0.006;

        let pointer = vec2<f32>(sim.pointer_x * sim.width, sim.pointer_y * sim.height);
        let pointer_delta = p - pointer;
        let pointer_falloff = max(0.0, 1.0 - length(pointer_delta) / 420.0);
        let pointer_force = vec2<f32>(sim.impulse_x, sim.impulse_y)
            * (0.18 * pointer_falloff * pointer_falloff);
        let buoyancy = vec2<f32>(0.0, 0.0035 + 0.002 * sin(phase + t));
        v = clamp(v * drag + (vortex + waves) * sim.flow_strength
                  + buoyancy + pointer_force, vec2<f32>(-4.0), vec2<f32>(4.0));
        p += v;

        if (p.x < 0.0) { p.x += sim.width; }
        if (p.x >= sim.width) { p.x -= sim.width; }
        if (p.y < 0.0) { p.y = 0.0; v.y = abs(v.y) * 0.72; }
        if (p.y >= sim.height) {
            p.y = sim.height - 0.001;
            v.y = -abs(v.y) * 0.72;
            v.x *= 0.985;
        }
    }

    let normalized = clamp(p / vec2<f32>(sim.width, sim.height),
                           vec2<f32>(0.0), vec2<f32>(1.0));
    store_particle(i, vec2<u32>(pack2x16unorm(normalized), pack2x16snorm(v / 4.0)));
}
