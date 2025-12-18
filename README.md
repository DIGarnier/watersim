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

See [OPTIMIZATIONS.md](OPTIMIZATIONS.md) for detailed documentation of all techniques.

## Building

```bash
cargo build --release
cargo run --release
```

## Architecture

- Rust-based physics engine with ggez for rendering
- Multi-threaded physics simulation
- Radial blur shader for visual effects
- Modular optimization system for easy testing

## References

Based on research from:
- Barnes & Hut (1986): Barnes-Hut algorithm
- Verlet (1967): Neighbor lists for molecular dynamics
- Recent GPU implementations (2025): 4 trillion interactions in 8ms
- SIMD optimization techniques: 2-4x speedup with AVX2/AVX-512

See OPTIMIZATIONS.md for complete references and implementation details.
