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

- **Mouse drag**: Add 20 particles at once (cannon)
- **W/S**: Increase/decrease force scale
- **B**: Toggle Barnes-Hut algorithm ON/OFF
- **V**: Toggle Verlet neighbor lists ON/OFF
- **A**: Toggle adaptive time-stepping ON/OFF

## Compilation Status

✅ **All compilation errors fixed**
- Fixed ambiguous type error for `max_velocity`
- Removed unused timing variables
- Code compiles cleanly with zero errors
- Only 2 harmless warnings about unused utility methods

## Binary Size

Release binary: ~13MB (includes all ggez graphics dependencies)

## Performance Testing

To benchmark the optimizations:

1. Start with all optimizations OFF (press B, V, A to disable)
2. Add particles by dragging mouse
3. Note FPS when you have ~5000 particles
4. Enable Barnes-Hut (press B) and observe FPS improvement
5. Enable Verlet lists (press V) for additional speedup
6. Enable adaptive dt (press A) for stability in dense regions

The on-screen HUD shows:
- Current FPS
- Particle count
- Optimization status (ON/OFF for each)
- Integration and collision timings in microseconds
- Current timestep value

## Troubleshooting

### "could not find system library 'alsa'"
Install `libasound2-dev` (Ubuntu/Debian) or equivalent for your distro.

### "could not find system library 'udev'"
Install `libudev-dev` (Ubuntu/Debian) or equivalent for your distro.

### Low FPS in debug mode
This is expected. Use `cargo run --release` for realistic performance.

### Display/window issues
The simulation requires X11 or Wayland display server. It won't run in headless environments.

## Next Steps

See [OPTIMIZATIONS.md](OPTIMIZATIONS.md) for detailed documentation on all implemented techniques and expected performance characteristics.
