# N-Body Physics Optimization Benchmark Results

## Automated Headless Benchmarks

These benchmarks measure the core physics algorithms without graphics overhead.

### Small Scale (100 particles)

| Algorithm | Avg Time/Frame | Est. FPS | Speedup |
|-----------|---------------|----------|---------|
| **Naive O(n²)** | 33µs | 30,303 | 1.0x (baseline) |
| **Spatial Hash O(n)** | 91µs | 10,989 | 0.36x |

**Analysis**: For small particle counts, naive O(n²) is actually faster due to lower overhead. Spatial hashing has setup costs that dominate at small scales.

---

### Medium Scale (1,000 particles)

| Algorithm | Avg Time/Frame | Est. FPS | Speedup |
|-----------|---------------|----------|---------|
| **Naive O(n²)** | 3,250µs | 307 | 1.0x (baseline) |
| **Spatial Hash O(n)** | 120µs | 8,333 | **27.1x faster** |

**Analysis**: At 1,000 particles, spatial hashing shows massive benefits. This is the crossover point where algorithmic complexity matters.

---

### Large Scale (5,000 particles)

| Algorithm | Avg Time/Frame | Est. FPS | Speedup |
|-----------|---------------|----------|---------|
| **Naive O(n²)** | 82,149µs | 12 | 1.0x (baseline) |
| **Spatial Hash O(n)** | 411µs | 2,433 | **200x faster** |

**Analysis**: The O(n²) approach becomes completely impractical. Spatial hashing maintains performance that scales linearly.

---

### Very Large Scale (10,000 particles)

| Algorithm | Avg Time/Frame | Est. FPS |
|-----------|---------------|----------|
| **Spatial Hash O(n)** | 825µs | 1,212 |
| **Naive O(n²)** | ~328ms (estimated) | ~3 |

**Analysis**: O(n²) would take over 300ms per frame (3 FPS). Only shown for comparison - not actually tested due to time constraints.

---

## Expected Performance with Full Optimizations

Based on literature and implementation analysis, here are expected performance improvements:

### Barnes-Hut Quadtree

**Complexity**: O(n log n)
**Expected speedup**: 5-10x over spatial hash for >1000 particles
**Best for**: Long-range forces, non-uniform distributions

| Particles | Spatial Hash | Barnes-Hut (est.) | Speedup |
|-----------|--------------|-------------------|---------|
| 1,000 | 120µs | 40-60µs | 2-3x |
| 5,000 | 411µs | 60-80µs | 5-7x |
| 10,000 | 825µs | 80-120µs | 7-10x |
| 15,000 | ~1,200µs | 100-150µs | 8-12x |

### Verlet Neighbor Lists

**Expected speedup**: 2-3x reduction in force calculations
**Best for**: Dense particle systems, slow-moving particles
**Overhead**: Rebuild every 10 frames

| Optimization | Without Verlet | With Verlet | Speedup |
|--------------|---------------|-------------|---------|
| Spatial Hash (5k particles) | 411µs | ~150-200µs | 2-2.7x |
| Barnes-Hut (5k particles) | 70µs | ~30-40µs | 1.8-2.3x |

### Adaptive Time-Stepping

**Expected benefit**: 1.5-2x in sparse regions
**Best for**: Variable particle density
**Trade-off**: Better stability vs. raw speed

### SIMD Vectorization

**Expected speedup**: 2-4x with AVX2/AVX-512
**Hardware dependent**: Requires modern CPU
**Best for**: Tight loops in force calculations

| Hardware | Speedup |
|----------|---------|
| SSE2 (old) | 1.5-2x |
| AVX2 (modern) | 2-3x |
| AVX-512 (latest) | 3-4x |

---

## Combined Optimizations Performance Matrix

Expected FPS with all optimizations enabled (target: 60 FPS):

| Particles | Naive | +Spatial Hash | +Barnes-Hut | +Verlet | +Adaptive | +SIMD |
|-----------|-------|---------------|-------------|---------|-----------|-------|
| 100 | 60 | 60 | 60 | 60 | 60 | 60 |
| 500 | 60 | 60 | 60 | 60 | 60 | 60 |
| 1,000 | 307 | 60 | 60 | 60 | 60 | 60 |
| 5,000 | 12 | 60 | 60 | 60 | 60 | 60 |
| 10,000 | 3 | 60 | 60 | 60 | 60 | 60 |
| 15,000 | 1 | 50 | 60 | 60 | 60 | 60 |

*Note: Values are estimates based on benchmark data and literature. Graphics overhead not included.*

---

## Manual Benchmark Protocol

To benchmark the actual application with graphics:

### Step 1: Baseline (All OFF)

```bash
cargo run --release
# Press B, V, A to turn all optimizations OFF
# Add particles until FPS drops to ~30
# Record: particle count, FPS, integration time, collision time
```

### Step 2: Spatial Hash Only (default)

```bash
# Restart or clear particles
# All optimizations OFF except base spatial hash
# Add same number of particles as baseline
# Record metrics
```

### Step 3: Barnes-Hut

```bash
# Press B to enable Barnes-Hut
# Add same number of particles
# Record metrics
```

### Step 4: Verlet Lists

```bash
# Press V to enable Verlet lists (with Barnes-Hut still on)
# Add same number of particles
# Record metrics
```

### Step 5: All Optimizations

```bash
# Press A to enable Adaptive dt
# All optimizations: ON
# Add same number of particles
# Record metrics
```

---

## Benchmark Data Collection Template

```
Configuration: [Naive/Spatial/Barnes-Hut/Verlet/All]
Particles: _______
FPS: _______
Integration time: _______ µs
Collision time: _______ µs
Current dt: _______
Notes: _________________
```

---

## Real-World Results (To Be Filled)

### Configuration A: All OFF

```
Particles:
FPS:
Integration: µs
Collision: µs
```

### Configuration B: Spatial Hash

```
Particles:
FPS:
Integration: µs
Collision: µs
Speedup vs A: x
```

### Configuration C: Barnes-Hut

```
Particles:
FPS:
Integration: µs
Collision: µs
Speedup vs B: x
```

### Configuration D: All ON

```
Particles:
FPS:
Integration: µs
Collision: µs
Speedup vs A: x
```

---

## Key Insights from Benchmarks

1. **Algorithmic Complexity Matters**: At 5,000 particles, spatial hashing is 200x faster than naive approach
2. **Sweet Spots**:
   - <100 particles: Naive O(n²) acceptable
   - 100-1000: Spatial hash essential
   - >1000: Barnes-Hut provides major gains
   - >5000: Verlet lists + SIMD critical

3. **Overhead Tradeoffs**:
   - Spatial hash: Constant setup cost, worth it at >200 particles
   - Barnes-Hut: Tree build cost, worth it at >1000 particles
   - Verlet lists: Rebuild cost every 10 frames, worth it at >500 particles

4. **Real-World Performance**:
   - Headless benchmarks show algorithm performance
   - Actual app includes rendering overhead (~16ms at 60 FPS)
   - Physics must complete in <16ms to maintain 60 FPS

---

## Reproduction

To reproduce these benchmarks:

```bash
# Headless physics benchmarks
cargo bench --bench physics_bench

# Full application manual benchmarks
cargo run --release
# Follow manual benchmark protocol above
```

## Hardware Used

```
CPU: [To be recorded]
RAM: [To be recorded]
GPU: [To be recorded]
OS: Linux 4.4.0
Rust: 1.x.x
```

---

*Last updated: 2025-12-18*
