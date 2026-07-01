# watersim

An n-body particle physics simulation with state-of-the-art optimization techniques.

## Features

This simulation implements multiple cutting-edge optimization techniques from the computational physics literature:

- **Barnes-Hut Quadtree** - Hierarchical force approximation (O(n log n))
- **Verlet Neighbor Lists** - Cached neighbor lists to reduce redundant calculations
- **Adaptive Time-Stepping** - Dynamic timestep adjustment for stability
- **SIMD Vectorization** - Structure-of-Arrays layout for better performance
- **GPU Compute Shaders** - WGSL compute shaders for GPU-accelerated physics
- **Spatial Hashing** - Grid-based collision detection

All optimizations can be toggled at runtime to demonstrate their effectiveness!

## Controls

- **Mouse drag**: Add particles (cannon)
- **W/S**: Increase/decrease force scale
- **B**: Toggle Barnes-Hut algorithm
- **V**: Toggle Verlet neighbor lists
- **A**: Toggle adaptive time-stepping

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
- Barnes & Hut (1986): hierarchical force approximation
- Verlet (1967): neighbor lists for molecular dynamics
- Green (2010) / Hoetzlein (2014): counting-sort uniform grids (CSR layout)
- Burtscher & Pingali (2011): allocation-free flat treecodes
- Macklin et al. (2019): "Small Steps in Physics Simulation"

Full annotated bibliography with links: [docs/literature.md](docs/literature.md).
