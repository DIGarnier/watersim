// One hardware point per packed particle. This avoids the 4x vertex cost and
// billboard overdraw of the detailed renderer once particles are subpixel.

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
@group(0) @binding(0) var<storage, read> state_a: array<vec2<u32>>;
@group(0) @binding(1) var<storage, read> state_b: array<vec2<u32>>;
@group(0) @binding(2) var<uniform> sim: MassiveParams;

fn load_particle(i: u32) -> vec2<u32> {
    if (i < PARTICLES_PER_SHARD) { return state_a[i]; }
    return state_b[i - PARTICLES_PER_SHARD];
}

struct VertexOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec3<f32>,
};

fn hue_rgb(h: f32) -> vec3<f32> {
    let k = vec3<f32>(0.0, 4.0, 2.0);
    return clamp(abs(fract(h + k / 6.0) * 6.0 - 3.0) - 1.0,
                 vec3<f32>(0.0), vec3<f32>(1.0));
}

@vertex
fn particle_vertex(@builtin(vertex_index) vertex: u32) -> VertexOut {
    let i = vertex * sim.render_stride + sim.render_phase;
    let packed = load_particle(i);
    let normalized = unpack2x16unorm(packed.x);
    let speed = length(unpack2x16snorm(packed.y));
    var out: VertexOut;
    out.clip = vec4<f32>(normalized.x * 2.0 - 1.0,
                         1.0 - normalized.y * 2.0, 0.0, 1.0);
    out.color = (hue_rgb(fract(0.55 + speed * 0.16)) - 0.5) * 0.75 + 0.5;
    return out;
}

@fragment
fn particle_fragment(in: VertexOut) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
}
