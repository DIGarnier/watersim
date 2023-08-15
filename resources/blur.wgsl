struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) color: vec4<f32>,
}

@group(1) @binding(0)
var t: texture_2d<f32>;

@group(1) @binding(1)
var s: sampler;


// @fragment
// fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
//     var avg = vec4<f32>(0.);

//     let hsamp: i32 = 6;
//     for (var i:i32 = -hsamp; i <= hsamp; i++) {
//         for (var j:i32 = -hsamp; j <= hsamp; j++) {
//             avg = avg + textureSample(t, s, in.uv +  vec2<f32>(f32(i), f32(j)) * 0.001);
//         }
//     }

//     avg = avg / f32(hsamp*hsamp);
//     return avg;
// }



// https://www.shadertoy.com/view/Xltfzj
@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {

    let tau = 6.28318530718;
    let directions = 16.0;
    let quality = 4.0;
    let size = 16.0/1200.0;

    var color = textureSample(t, s, in.uv);
    // return color;

    for (var d:f32 = 0.0; d < tau; d = d + tau/directions) {
        for (var i:f32 = 1.0/quality; i < 1.001; i = i + 1.0/quality) {
            color = color + textureSample(t, s, in.uv + vec2<f32>(cos(d), sin(d)) * size * i);
        }
    }
    return color / (quality * directions + 1.0);
}