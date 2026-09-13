# watersim

A real-time particle water simulation built from techniques in the particle
simulation literature. The default engine is a GPU-resident compute pipeline
that runs through Metal on Apple silicon.

## The model(s)

Five selectable solver strategies (`--sim gpu|granular|pbf|dfsph|mlsmpm`,
default `gpu` in the standard build) cover four numerical models and two
granular backends. See [docs/solvers.md](docs/solvers.md) for the architecture
and the evaluation behind the choice.

- **Granular** (PBD) — local pressure-like repulsion with compact support
  (cutoff = 2.5 particle radii, smoothly tapered to zero), position-based
  contact projection, and Størmer–Verlet integration at 480 Hz substeps. It
  stacks and slumps like a pile of balls. O(n) per step via a counting-sort CSR
  grid whose cell size equals the interaction cutoff, making the one-cell
  stencil provably exact (`--validate` checks it against the O(n²) sum).
- **GPU flow** — the next-generation default. Up to 131k particles use the
  detailed granular grid/contact model; larger populations switch to a fused
  O(N) procedural flow kernel designed for millions. The detailed regime uses
  instanced billboards; the massive regime uses hardware points and rotating
  cohorts directly from two packed state buffers. On this Mac, `wgpu` maps to
  Metal; the live path has no particle readback or CPU transform upload.
- **PBF** (PBD) — Position Based Fluids (Macklin & Müller, SIGGRAPH 2013): the
  same position-projection solver enforces a *density* constraint instead of
  non-penetration, so the particles pour, splash, and slosh as an incompressible
  liquid. Stable but inherently dissipative. See [docs/pbf.md](docs/pbf.md).
- **DFSPH** (pressure-SPH) — Divergence-Free SPH (Bender & Koschier 2015/17):
  incompressibility from a *pressure force* (a constant-density and a
  divergence-free velocity solve) rather than geometric projection, so it's
  crisp and low-dissipation — sharper splashes and livelier sloshing than PBF.
- **MLS-MPM** (hybrid grid+particle) — Moving-Least-Squares Material Point
  Method (Hu et al. 2018): particles carry state, a background grid does the
  momentum solve (APIC transfers). A weakly-compressible **liquid** by default;
  swap the constitutive model for an elastic **jelly** — the one model here that
  does elastic solids, not just fluids.

## Optimization techniques (all measured; see docs/benchmarks)

- **Zero-copy GPU pipeline** - integration, atomic spatial bins, force gather,
  Jacobi contacts, and instanced rendering in one WGSL/wgpu command buffer
- **Massive-particle compression** - 64-bit normalized position/velocity,
  two 128 MiB storage shards, GPU-side initialization, one-vertex points, and
  rotating update/render cohorts
- **CSR counting-sort grid** - one flat index array, built once per substep
- **Verlet neighbor lists** - displacement-triggered lazy rebuilds
- **Multiple time stepping (r-RESPA)** - smooth forces refreshed every 4th substep
- **Two solver engines** - serial Gauss-Seidel; packed SoA Jacobi with clustered SIMD gathers at scale
- **Small Steps substepping** - runtime knob (Macklin et al. 2019)
- **Rayon parallelism, SFC particle reordering, hardware rsqrt kernels**
- **Adaptive time-stepping** - dynamic timestep adjustment for stability

## Controls

- **Mouse drag**: Add particles in detailed mode; steer the flow field at scale
- **W/S**: Increase/decrease force scale
- **V**: Toggle Verlet neighbor lists (legacy CPU granular mode)
- **A**: Toggle adaptive time-stepping (legacy CPU modes)

Launch flag: `--sim gpu|granular|pbf|dfsph|mlsmpm` selects the fluid model
(default GPU; use `--sim granular` for the previous CPU implementation).

## Performance

The simulation displays real-time performance metrics in the window title:
- FPS and particle count
- GPU frame encoding/submission time

Legacy CPU strategies retain the detailed in-window phase HUD.

Measured performance and the full optimization history live in
[docs/benchmarks/](docs/benchmarks/README.md); the techniques and the papers
behind them are surveyed in [docs/literature.md](docs/literature.md).

## Building

```bash
cargo build --release
cargo run --release
```

The direct GPU app starts with 16 million particles and supports up to
33,554,432. It executes simulation and point rendering in a single command
buffer per visual frame. Override the starting population with:

```bash
WATERSIM_SEED_PARTICLES=33554432 cargo run --release
```

Record five seconds of the real GPU framebuffer to H.264 (requires `ffmpeg`):

```bash
WATERSIM_SEED_PARTICLES=33554432 \
WATERSIM_RENDER_PARTICLES=1000000 \
WATERSIM_RECORD=renders/33m-particles.mp4 \
cargo run --release
```

Only the finished framebuffer is read back for recording; particle state stays
GPU-resident. `WATERSIM_RENDER_PARTICLES` controls the rotating visual sample,
which is useful when the particle population greatly exceeds the pixel count.

Measure both strict per-step GPU synchronization and the live render-cadence path:

```bash
cargo run --release --no-default-features --features gpu --bin gpu_bench -- 50000 240
```

## Benchmarking

A headless benchmark drives the real physics engine (no graphics dependencies
needed) across force paths and particle counts:

```bash
cargo bench --no-default-features --bench nbody          # full suite
cargo bench --no-default-features --bench nbody -- --quick
```

## Architecture

- Default: one native `winit` + `wgpu` device for Metal compute and direct
  instanced rendering; particle state never leaves VRAM
- Legacy modes: the original multi-threaded CPU solvers and ggez renderer
- Modular solver system and headless CPU/GPU comparison harnesses

## References

Based on research from:
- Müller et al. (2003) / Macklin & Müller (2013) / Clavet et al. (2005): local particle water models
- Bender & Koschier (2015, 2017): Divergence-Free SPH (DFSPH)
- Hu et al. (2018) / Jiang et al. (2015): MLS-MPM and the APIC transfer
- Verlet (1967): neighbor lists for molecular dynamics
- Green (2010) / Hoetzlein (2014): counting-sort uniform grids (CSR layout)
- Tuckerman, Berne & Martyna (1992): r-RESPA multiple time stepping
- Macklin et al. (2014, 2019): constraint relaxation; "Small Steps in Physics Simulation"

Full annotated bibliography with links: [docs/literature.md](docs/literature.md);
the solver survey and design notes: [docs/solvers.md](docs/solvers.md).
