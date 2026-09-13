# 28 — zero-readback compute + rendering (2026-09-13)

## Result

The standard live application now simulates and renders from one shared GPU
particle buffer. A visual frame contains all due physics ticks and a single
instanced particle draw in one wgpu command buffer and one queue submission.
There is no position readback, CPU particle loop, or transform upload in the
hot path.

## Why the default window no longer uses ggez

ggez 0.9 exposes its wgpu device, but creates that device with WebGL2-downlevel
limits. In this version those requested limits advertise zero compute
workgroups, so a compute pipeline cannot legally be created on the renderer's
device. A second compute device would force cross-device copies through the
CPU and defeat the design.

The GPU mode therefore uses a small native `winit` + `wgpu` event loop. The
same adapter, device, queue, and command encoder own:

1. atomic-grid clear and particle binning;
2. Verlet integration and compact-support force gathering;
3. three ping-pong Jacobi contact passes;
4. procedural four-vertex particle billboards read directly from storage.

That neighbor-based pipeline remains the detailed regime through 131,072
particles. Above the threshold, a fused O(N) flow kernel advances all due 120
Hz ticks in one dispatch. Particle radius scales down with population to avoid
fragment overdraw. This is an intentional model change: at millions of
particles, preserving the old contact semantics would spend nearly all work
resolving multiple particles per pixel and cannot maintain real time.

The CPU granular, PBF, DFSPH, and MLS-MPM modes are still selectable and keep
the original ggez renderer and detailed HUD.

## Reproducing the live stress run

```bash
WATERSIM_SEED_PARTICLES=4000000 WATERSIM_DEBUG=1 cargo run --release
```

The window title reports frame rate, particle count, and CPU-side command
encoding/submission time. Add `WATERSIM_UNCAPPED=1` to request a no-vsync
present mode when measuring maximum throughput.

On the Apple M5 test machine, an uncapped one-million-particle release run
sustained 120 FPS for the sample with no wgpu validation errors. Four million
particles started around 75 FPS and settled around **64–70 FPS** during the
20-second run. CPU-side encoding plus the single queue submission was generally
below 0.4 ms. This is not presented as GPU kernel time: the queue executes
asynchronously, so observed frame rate is the end-to-end live check.

The headless solver validation and strict-vs-batched timing remain in stage
27. That harness deliberately supports readback because finite positions,
wall bounds, and grid overflow are correctness gates; the shipping live path
does not pay that cost.
