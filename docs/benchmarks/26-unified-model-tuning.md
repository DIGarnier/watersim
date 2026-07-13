# 26 — Pass 3: re-profile the unified model, re-tune, deflate the packed engine

With the model unified (stage 25) the cost structure is simpler and worth
re-profiling from scratch. Phase means (µs/step, cutoff model, defaults):

| particles | path | integrate | grid build | forces | solver | total |
|---|---|---|---|---|---|---|
| 12000 | verlet | 29 | 108 | 131 | 704 | 979 |
| 12000 | spatial | 28 | 107 | 108 | 654 | 904 |
| 24000 | verlet | 68 | 208 | 226 | 1329 | 1900 |
| 24000 | spatial | 82 | 236 | 196 | 1111 | 1700 |

The **solver still dominates** (65–70 %), grid build is the clear #2
(~12 %), forces a distant #3 (the cutoff pruned them). Three changes, all
attacking those two phases, all bit-exactness-preserving:

## 1. Engine crossover re-swept: 22 000 → 14 000

The clustered solver gather (stage 23) and the cutoff model (stage 25) both
made the packed Jacobi engine cheaper per element, so its break-even against
the serial Gauss-Seidel engine moved down. Fresh `--engine-sweep`:

| particles | verlet serial→packed | spatial serial→packed |
|---|---|---|
| 8000 | **670** / 740 | 984 / **834** |
| 12000 | **922** / 957 | 919 / 922 (tie) |
| 16000 | 1242 / **1074** | 1165 / **1006** |
| 20000 | 1523 / **1339** | **1518** / 1604 |
| 24000 | **1939** / 2138 | 1978 / **1557** |

Packed wins clearly from 16k on both paths and ties at 12k; the 20k/24k
verlet reversals are inside the noise band. Threshold set to **14 000** —
routes 16k–20k (previously serial) to the packed engine. Still
runtime-tunable via `set_par_min_particles`; on a machine with more cores
the packed crossover drops further.

## 2. Packed solver: double-buffered, no accumulate-then-apply

The packed contact iteration was gather-corrections-into-`acc`, then a
serial `for k: px[k] += acc[k]` apply sweep. Now the gather writes the
*corrected positions* straight into `acc` and the buffers are swapped in —
the per-iteration serial O(n) apply sweep is gone (3 saved per substep at
it=3), and since every element is overwritten, the buffer no longer needs
zeroing either. Bit-identical (the arithmetic is unchanged; only where the
`+` lands moved); confirmed by the 24k full run (max pen 0.0 %, sane) and a
2400-step soak (mean pen / deep pairs match stage 25 exactly: 12k 3.72 %/1,
24k 10.38 %/99).

## 3. Grid build: parallel cell-id pass

The counting-sort's first pass computes each particle's cell id (fp mul +
two `as usize` conversions + clamps) then increments a per-cell counter.
The cell-id half is embarrassingly parallel; it now runs over rayon chunks
for `n ≥ par_min`, with the count increment kept serial (n scattered
writes). Applied only at the packed sizes where the region overhead pays.

## Measured (same-window A/B/A, stage-25 vs pass-3 defaults, mean µs/step)

| particles | verlet-lists | spatial-hash |
|---|---|---|
| 6000 | 486 → 427 | 473 → 422 |
| 12000 | 857 → 851 | 912 → 797 |
| 24000 | **1369 → 1009 (−26 %)** | **1136 → 1003 (−12 %)** |

- 24k is the headline: the packed engine shed the apply sweeps and got the
  parallel grid build. Verlet −26 %, spatial −12 %.
- 12k (below the 14k crossover, still serial) is unchanged — the pass-3
  changes touch only the packed path, so no regression where they don't
  apply, exactly as intended.
- 16k–20k gains come from the crossover re-route (§1), measured in the
  engine sweep above rather than the full run (which skips those sizes).

Contact quality unchanged everywhere (max pen 0.0 % ≥ 3k; soak deep-pair
counts identical). Every path comfortably real-time through 24 000.
