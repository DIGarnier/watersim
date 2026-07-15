# watersim

A real-time particle water simulation built from techniques in the particle
simulation literature, with every optimization measured before adoption.

## The model(s)

Four selectable particle-fluid models (`--sim granular|pbf|dfsph|mlsmpm`,
default granular), each a self-contained *strategy* chosen once at startup.
They span three method families; see [docs/solvers.md](docs/solvers.md) for the
architecture and the evaluation behind the choice.

- **Granular** (PBD) — local pressure-like repulsion with compact support
  (cutoff = 2.5 particle radii, smoothly tapered to zero), position-based
  contact projection, and Størmer–Verlet integration at 480 Hz substeps. It
  stacks and slumps like a pile of balls. O(n) per step via a counting-sort CSR
  grid whose cell size equals the interaction cutoff, making the one-cell
  stencil provably exact (`--validate` checks it against the O(n²) sum).
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

- **CSR counting-sort grid** - one flat index array, built once per substep
- **Verlet neighbor lists** - displacement-triggered lazy rebuilds
- **Multiple time stepping (r-RESPA)** - smooth forces refreshed every 4th substep
- **Two solver engines** - serial Gauss-Seidel; packed SoA Jacobi with clustered SIMD gathers at scale
- **Small Steps substepping** - runtime knob (Macklin et al. 2019)
- **Rayon parallelism, SFC particle reordering, hardware rsqrt kernels**
- **Adaptive time-stepping** - dynamic timestep adjustment for stability

## Controls

- **Mouse drag**: Add particles (cannon)
- **W/S**: Increase/decrease force scale
- **V**: Toggle Verlet neighbor lists
- **A**: Toggle adaptive time-stepping

Launch flag: `--sim granular|pbf|dfsph|mlsmpm` selects the fluid model
(default granular).

## Performance

The simulation displays real-time performance metrics:
- FPS and particle count
- Integration and collision timings
- Active optimization status
- Current adaptive timestep

Measured performance and the full optimization history live in
[docs/benchmarks/](docs/benchmarks/README.md); the techniques and the papers
behind them are surveyed in [docs/literature.md](docs/literature.md).

## Building

```bash
cargo build --release
cargo run --release
```

## Benchmarking

A headless benchmark drives the real physics engine (no graphics dependencies
needed) across force paths and particle counts:

```bash
cargo bench --no-default-features --bench nbody          # full suite
cargo bench --no-default-features --bench nbody -- --quick
```

## Architecture

- Rust-based physics engine with ggez for rendering
- Multi-threaded physics simulation
- Radial blur shader for visual effects
- Modular optimization system for easy testing

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
