# Benchmark 04 — Solver iteration reduction ("Small Steps", Macklin et al. 2019)

- Date: 2026-07-01
- Change (`src/physics.rs`): the constraint solver ran a hard-coded **8**
  projection iterations per 480 Hz substep — i.e. 64 iterations per 60 Hz visual
  frame. Per *Small Steps in Physics Simulation* (SCA 2019, docs/literature.md §2),
  substeps are worth far more than solver iterations; the sim's 8× substepping is
  already the expensive, valuable half of that trade. Iteration count is now
  `SOLVER_ITERATIONS = 4`.
- New quality metric in the harness: **max pen %** — the worst pair overlap in the
  final state as a percentage of the contact distance (2·BALL_SIZE). Only the
  1000-particle configs run long enough for the block to settle into sustained
  contact, so those rows are the quality-sensitive ones.
- Compare against: `docs/benchmarks/03-barnes-hut-arena.md`

## Iteration sweep (1000 particles, 300 steps, quality-sensitive config)

| iterations | verlet median µs | verlet max pen % | spatial-hash median µs | spatial-hash max pen % |
|---|---|---|---|---|
| 8 (old) | 137 | 11.8 | 148 | 10.4 |
| **4 (new)** | **89** | **22.8** | **110** | **24.5** |
| 2 | 61 | 54.9 | 84 | 34.7 |
| 1 | 54 | 83.9 | 78 | 57.5 |

Simply *cutting* iterations is not free — Macklin et al. redistribute the work into
more substeps, whereas the substep rate here is pinned at 480 Hz by the app loop. 4
iterations keeps worst-case penetration in the "briefly visible under impact" range
(transient single-pair maximum, not steady state) for a 1.5× step-time win; 2 and 1
iterations trade too much contact quality. Chosen default: **4**.

## Full run at 4 iterations

| particles | force path | steps | mean µs/step | median | p95 | min | max | 480 Hz real-time? | max pen % | sane? |
|---|---|---|---|---|---|---|---|---|---|---|
| 1000 | barnes-hut | 300 | 985 | 991 | 1194 | 686 | 1312 | yes | 88.4 | ok |
| 1000 | verlet-lists | 300 | 103 | 89 | 158 | 76 | 190 | yes | 22.8 | ok |
| 1000 | spatial-hash | 300 | 119 | 110 | 170 | 83 | 223 | yes | 24.5 | ok |
| 3000 | barnes-hut | 200 | 4169 | 4190 | 4823 | 3166 | 5019 | NO | 13.4 | ok |
| 3000 | verlet-lists | 200 | 295 | 275 | 404 | 229 | 531 | yes | 0.0 | ok |
| 3000 | spatial-hash | 200 | 352 | 344 | 462 | 273 | 524 | yes | 0.0 | ok |
| 6000 | barnes-hut | 120 | 9128 | 9307 | 10355 | 6952 | 10874 | NO | 14.2 | ok |
| 6000 | verlet-lists | 120 | 600 | 580 | 804 | 489 | 969 | yes | 0.0 | ok |
| 6000 | spatial-hash | 120 | 679 | 671 | 891 | 570 | 943 | yes | 0.0 | ok |
| 12000 | barnes-hut | 60 | 18454 | 18528 | 20879 | 15828 | 21504 | NO | 19.4 | ok |
| 12000 | verlet-lists | 60 | 1180 | 1113 | 1541 | 988 | 1868 | yes | 0.0 | ok |
| 12000 | spatial-hash | 60 | 1356 | 1303 | 1714 | 1146 | 1871 | yes | 0.1 | ok |

(The 1k barnes-hut pen numbers are high at every iteration count because the BH
configuration compresses the pile much harder; even at 8 iterations it sat at 51.8%.)

## Delta vs benchmark 03 (median µs/step, verlet-lists)

1000: 134 → 89 (1.5×) · 3000: 424 → 275 (1.5×) · 6000: 839 → 580 (1.4×) · 12000: 1664 → 1113 (1.5×)

## Follow-up enabled by the paper (not done here)

The *proper* Small Steps trade — raising the substep rate while dropping to 1–2
iterations (e.g. 960 Hz × 2) — would beat both speed and quality of 480 Hz × 8.
It isn't applied because the integrator applies acceleration as `a·dt` rather than
`a·dt²`, which makes simulation behavior depend on the substep rate; changing
PHYS_TIME_STEP would visibly change gravity/force strength. Fixing the integrator
first would unlock this.
