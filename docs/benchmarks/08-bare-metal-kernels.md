# Benchmark 08 — Bare-metal kernel pass (rsqrt, branch/store discipline, dead work)

- Date: 2026-07-01
- Changes (`src/physics.rs`), all in the per-pair/per-particle hot kernels:
  1. **Hardware rsqrt** — `fast_rsqrt()` = SSE `rsqrtss` (~12 bit) + one
     Newton-Raphson step (~22 bit). Replaces `length()`+`normalize()` in the pair
     force kernel (two sqrts + divides → one rsqrt) and `sqrt().recip()` in both
     Barnes-Hut interaction branches. Contact normals and 1/r² force directions
     don't need the last bit of precision.
  2. **Contact projector** — `ball_collides()` + `length()` + `try_normalize()`
     (two sqrts) folded into one rsqrt with the same degenerate-pair semantics.
  3. **Store discipline** — the projector only writes positions when the pair
     actually overlaps. (First attempt returned a ZERO correction and stored it
     unconditionally — a consistent, reproducible **~4% regression**, since most
     checked pairs don't collide. Caught by A/B with 2 runs per build.)
  4. **Dead work removal** — the integrator computed `arbitrary_vector_field()`
     (one sin + one cos per particle per 480 Hz substep) only to multiply the
     result by 0.0, which the compiler cannot fold under IEEE rules. Removed.
     Particle colors (one sqrt + fmod each) are consumed by the 60 Hz renderer
     but were recomputed on all 8 substeps per frame; now computed on every 8th
     substep. Max-velocity tracking for adaptive dt uses squared lengths with a
     single sqrt at the end.
  5. `cell_id`: divide by grid size → multiply by constant reciprocal.
- Methodology note: these deltas are near the shared-VM noise band (±10%), so this
  stage was validated with alternating A/B (2 full runs per build) rather than a
  single pair of runs.
- Compare against: `docs/benchmarks/07-spatial-reorder.md`

## Result

| particles | force path | steps | mean µs/step | median | p95 | min | max | 480 Hz real-time? | max pen % | sane? |
|---|---|---|---|---|---|---|---|---|---|---|
| 1000 | barnes-hut | 300 | 511 | 502 | 644 | 322 | 1158 | yes | 86.6 | ok |
| 1000 | verlet-lists | 300 | 107 | 104 | 141 | 77 | 180 | yes | 21.9 | ok |
| 1000 | spatial-hash | 300 | 104 | 102 | 132 | 78 | 175 | yes | 18.0 | ok |
| 3000 | barnes-hut | 200 | 1786 | 1753 | 2086 | 1105 | 2342 | yes | 9.6 | ok |
| 3000 | verlet-lists | 200 | 314 | 304 | 387 | 254 | 495 | yes | 0.0 | ok |
| 3000 | spatial-hash | 200 | 313 | 303 | 391 | 254 | 431 | yes | 0.0 | ok |
| 6000 | barnes-hut | 120 | 3566 | 3633 | 4307 | 2346 | 4746 | NO | 14.5 | ok |
| 6000 | verlet-lists | 120 | 592 | 587 | 709 | 480 | 851 | yes | 0.0 | ok |
| 6000 | spatial-hash | 120 | 586 | 589 | 631 | 501 | 657 | yes | 0.0 | ok |
| 12000 | barnes-hut | 60 | 6656 | 6489 | 8752 | 5148 | 9696 | NO | 19.9 | ok |
| 12000 | verlet-lists | 60 | 1104 | 1091 | 1253 | 929 | 1588 | yes | 0.1 | ok |
| 12000 | spatial-hash | 60 | 1161 | 1123 | 1598 | 939 | 1744 | yes | 0.1 | ok |
| 24000 | barnes-hut | 40 | 15275 | 14758 | 18095 | 12747 | 21950 | NO | 90.2 | ok |
| 24000 | verlet-lists | 40 | 4459 | 4185 | 7384 | 3207 | 8695 | NO | 95.1 | ok |
| 24000 | spatial-hash | 40 | 4470 | 4348 | 6203 | 3244 | 7973 | NO | 93.8 | ok |

## A/B verdict vs benchmark 07 (2 runs per build, medians)

| config | before (2 runs) | after (2 runs) | delta |
|---|---|---|---|
| verlet 3k | 316 / 319 | 302 / 300 | **−5%** |
| verlet 12k | 1194 / 1203 | 1077 / 1078 | **−10%** |
| spatial 12k | 1212 / 1192 | 1077 / 1115 | **−8%** |
| barnes-hut 3k | 1787 / 1892 | 1719 / 1748 | **−5%** |
| verlet 24k | 4668 | 4185 | ~−10% |

Simulation behavior: force/normal directions now carry ~1e-6 relative error from the
refined rsqrt (visually and statistically indistinguishable; penetration metrics
unchanged), and colors update at the renderer's own 60 Hz instead of 480 Hz.
