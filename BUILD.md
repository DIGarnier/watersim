# Build Instructions

## System Dependencies

The simulation requires the following system libraries for audio and device access:

```bash
# Ubuntu/Debian
sudo apt-get install libasound2-dev libudev-dev pkg-config

# Fedora/RHEL
sudo dnf install alsa-lib-devel systemd-devel

# Arch Linux
sudo pacman -S alsa-lib systemd
```

## Building

The default build includes the `gpu` feature. On macOS, wgpu selects Metal;
on Windows/Linux it selects the native DX12/Vulkan backend. A CPU-only library
build remains available with `--no-default-features`.

### Debug Build (for development)
```bash
cargo build
./target/debug/lolballs
```

### Release Build (optimized, recommended for benchmarking)
```bash
cargo build --release
./target/release/lolballs
```

The release build is **highly recommended** for testing the optimizations as it includes:
- Full compiler optimizations (-O3 equivalent)
- Auto-vectorization (SIMD)
- Inlining of hot functions
- Link-time optimization

## Running

Once built, you can run the simulation:

```bash
# Debug mode (slower, easier to debug)
cargo run

# Release mode (fast, for benchmarking)
cargo run --release
```

## Controls

- **Mouse drag**: Add 20 particles in detailed mode; steer massive-scale flow
- **W/S**: Increase/decrease force scale
- **V**: Toggle Verlet neighbor lists ON/OFF (CPU granular)
- **A**: Toggle adaptive time-stepping ON/OFF

## Performance Testing

For a repeatable CPU/GPU comparison on the current machine:

```bash
cargo run --release --no-default-features --features gpu --bin gpu_bench -- 50000 480
```

Run `cargo run --release` for the GPU default or add `-- --sim granular` for
the legacy optimized CPU solver.

The default GPU launch seeds 16 million particles. Override it up to the
33,554,432-particle hard capacity with `WATERSIM_SEED_PARTICLES`.

The GPU window title shows:
- Current FPS
- Particle count
- GPU frame encoding/submission time

Set `WATERSIM_DEBUG=1` to mirror those live statistics to stderr, and set
`WATERSIM_UNCAPPED=1` to disable vsync for throughput measurements. The legacy
CPU strategies retain the detailed in-window phase HUD.

## Troubleshooting

### "could not find system library 'alsa'"
Install `libasound2-dev` (Ubuntu/Debian) or equivalent for your distro.

### "could not find system library 'udev'"
Install `libudev-dev` (Ubuntu/Debian) or equivalent for your distro.

### Low FPS in debug mode
This is expected. Use `cargo run --release` for realistic performance.

### GPU fallback
The direct app fails clearly if wgpu cannot acquire a compatible adapter. The
headless `gpu_bench` output has a `GPU active` column so its CPU fallback cannot
be mistaken for a GPU measurement. Use `--sim granular` to explicitly run the
legacy CPU engine.

### Display/window issues
The simulation requires Metal on macOS or an X11/Wayland display server on Linux.

## Next Steps

See [docs/benchmarks/27-gpu-metal.md](docs/benchmarks/27-gpu-metal.md) for the
compute design and M5 results, and
[docs/benchmarks/28-zero-copy-render.md](docs/benchmarks/28-zero-copy-render.md)
for the direct rendering architecture, and
[docs/benchmarks/29-massive-particles.md](docs/benchmarks/29-massive-particles.md)
for the packed 32M design and literature review.
