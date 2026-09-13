# 27 — GPU-resident compute on Apple M5 (2026-09-13)

## Decision

Adopt `wgpu` compute as the standard-build default (`--sim gpu`). On macOS it
uses the native Metal backend; WGSL keeps the kernels portable to Vulkan/DX12
and WebGPU. We intentionally use the `wgpu` version already in ggez 0.9's
dependency graph, avoiding a second graphics stack.

CUDA is not available on Apple silicon. Metal is the native alternative, but
calling it directly would make the solver macOS-only and require an additional
Rust/Objective-C boundary. Metal Performance Shaders provides optimized
primitives, not the irregular neighbor gather/contact kernels needed here.

- [wgpu backends](https://wgpu.rs/doc/wgpu/enum.Backend.html)
- [wgpu shader formats](https://wgpu.rs/doc/wgpu/documentation/shaders/index.html)
- [Apple Metal compute overview](https://developer.apple.com/documentation/metal/performing-calculations-on-a-gpu)

## Architecture

The new path is GPU-resident, not a CPU solver with a GPU coprocessor call:

1. Clear atomic cell counters.
2. Verlet-integrate every particle and insert it into a fixed-capacity uniform
   grid bucket (`64` particles/cell, with an overflow diagnostic).
3. Run three ping-pong Jacobi contact passes.
4. Gather the compact-support force for the next substep.
5. Keep position, previous-position, force, grid, and scratch buffers on GPU.
6. In the headless comparison path, copy positions to the CPU only once per 8
   physics steps (480 Hz → 60 Hz). The live renderer added in stage 28 does
   not copy positions at all.

The standard capacity is 262,144 particles. Buffer use is about 17 MiB,
dominated by the 32,000 × 64 cell bucket table.

## M5 results

Command:

```bash
cargo run --release --no-default-features --features gpu --bin gpu_bench -- <N> 120
```

The strict column forces a CPU-visible snapshot after every substep. The live
column uses the shipped render cadence. Times are end-to-end wall time and
include queue submission and synchronization. All runs ended with zero NaN or
out-of-bounds particles.

| particles | optimized CPU | GPU strict sync | GPU live sync/8 | live speedup |
|---:|---:|---:|---:|---:|
| 1,000 | 54.1 µs | 1,562.3 µs | 218.5 µs | 0.25× |
| 15,000 | 470.9 µs | 1,619.3 µs | 415.7 µs | 1.13× |
| 50,000 | 1,201.9 µs | 1,675.2 µs | 637.7 µs | **1.88×** |

The result demonstrates why GPU residency is the core design constraint:
forcing a map/readback every 2.08 ms loses at every measured size. At render
cadence the M5 overtakes the heavily optimized Rayon/SIMD CPU path around the
existing 15k scale and gains with particle count. Even the 50k GPU path uses
only 31% of the 2,083 µs physics-step budget.

A longer 480-substep soak at 50k measured 1,364.5 µs CPU versus 780.9 µs GPU
(**1.75×**) and also finished with zero bucket overflow, NaNs, or escapes. The
GPU recomputes forces every substep, while the legacy CPU default uses its
four-step force cache, so the new path is doing more force work in this A/B.

## Next stage

Stage 28 replaces ggez for the default live application with a thin native
winit/wgpu loop. Its vertex shader consumes the simulation position buffer
directly, removing the last CPU particle round trip from the hot path. The
legacy CPU solvers continue to use their existing ggez UI.
