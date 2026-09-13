# 29 — packed and time-sliced massive particles (2026-09-13)

## Result

The live capacity is now **33,554,432 stateful particles**, up from 4,194,304.
The standard launch starts with 16 million. At the ceiling, every particle has
its own persistent position and velocity, but dense-screen work is interleaved
over time: approximately four million particles advance per visual frame and
one third of the ceiling population renders each frame.

Apple M5 release results with no-vsync presentation:

| resident particles | observed live rate | notes |
|---:|---:|---|
| 8,000,000 | 120 FPS | display/compositor ceiling in this test |
| ≈16,000,000 | 75–80 FPS initially, 54–60 FPS sustained | full point set rendered; four simulation cohorts |
| 33,554,432 | about 80 FPS initially, **47–55 FPS sustained** | three render cohorts; nine simulation cohorts |

There were no wgpu shader or resource-validation errors in the successful
runs. CPU command encoding/submission generally remained below 0.4 ms; the
reported frame rate includes asynchronous GPU execution and presentation.

## Literature and design choices

### Keep state on the GPU

Crane et al.'s GPU fluid-particle system binds simulation data directly for
rendering and avoids bus transfers. The existing stage-28 architecture already
followed that rule, and this stage keeps initialization on GPU too—starting
32M particles no longer allocates or uploads a CPU particle vector.

- [Interactive Fluid-Particle Simulation using Translating Eulerian Grids](https://research.nvidia.com/publication/2010-02_interactive-fluid-particle-simulation-using-translating-eulerian-grids)

### Spend bits on screen-space accuracy, not general-purpose floats

The massive path stores normalized XY position with `pack2x16unorm` and bounded
XY velocity with `pack2x16snorm`: two `u32`s, or 8 bytes per particle. The WGSL
spec explicitly provides these packing functions to reduce storage and memory
bandwidth. In the 1600×1200 simulation domain, position quantization is about
0.024 px horizontally and 0.018 px vertically—well below one display pixel.

- [WGSL data packing and unpacking](https://www.w3.org/TR/WGSL/)

WebGPU guarantees only 128 MiB for one storage-buffer binding. One packed shard
therefore holds exactly 16,777,216 particles. Two bindings provide the 32M
capacity without asking for non-portable limits.

- [WebGPU `maxStorageBufferBindingSize`](https://gpuweb.github.io/gpuweb/)

### Stop rasterizing subpixel quads

GPU Gems identifies particle overdraw/fill rate as the central large-system
rendering problem. Above the detailed regime, particles are already subpixel,
so the renderer issues one hardware point vertex instead of a four-vertex
alpha-blended billboard. This cuts vertex work by 4× and removes billboard
fragment overdraw.

- [GPU Gems 3: High-Speed, Off-Screen Particles](https://developer.nvidia.com/gpugems/gpugems3/part-iv-image-effects/chapter-23-high-speed-screen-particles)

### Interleave work when samples outnumber pixels

Large particle visualization research uses stochastic particle subsets to
reduce accesses while preserving the aggregate appearance. Here, rotating
deterministic cohorts apply the same screen-density observation to both
rendering and the intentionally visual flow model. Simulation ticks are
accumulated for a cohort, so a particle advances the full elapsed interval when
its turn arrives; render cohorts rotate every frame.

- [Stochastic Volume Rendering of Multi-Phase SPH Data](https://doi.org/10.1111/cgf.14121)
- [Interactive Rendering of Giga-Particle Fluid Simulations](https://www.cs.cit.tum.de/cg/research/publications/2014/interactive-rendering-of-giga-particle-fluid-simulations/)

This is an explicit accuracy/performance trade: the 32M regime is a visual
fluid-flow simulation, not a 32M-particle incompressibility/contact solve. The
detailed grid/contact model remains available through 131,072 particles.

## Reproduce

```bash
# Standard 16M launch
cargo run --release

# Two-shard ceiling
WATERSIM_SEED_PARTICLES=33554432 WATERSIM_UNCAPPED=1 WATERSIM_DEBUG=1 cargo run --release
```
