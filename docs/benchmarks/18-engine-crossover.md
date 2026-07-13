# 18 — Re-sweeping the serial ↔ packed solver-engine crossover

`PAR_MIN_PARTICLES` (16 000, tuned in stage 10) picks between the serial
Gauss-Seidel engine and the packed parallel Jacobi engine on the grid paths.
Stages 13–17 changed everything that tuning rested on: forces refresh every
4th substep (fewer parallel regions matter less), 3 solver iterations instead
of 4, and smaller/denser particles. Per the series methodology, the threshold
gets re-measured, not trusted: new `--engine-sweep` bench mode
(`set_par_min_particles` forces the engine each way, everything else at the
adopted defaults).

## Measured, mean µs/step (serial vs packed)

At BALL=3, GRID=10 (stage-17 state):

| particles | verlet serial→packed | spatial serial→packed |
|---|---|---|
| 8000 | **805** / 968 | **828** / 935 |
| 12000 | **1200** / 1336 | **1289** / 1397 |
| 16000 | **1598** / 1712 | 1674 / **1671** |
| 20000 | **2075** / 2297 | 2111 / **2065** |
| 24000 | 2899 / **2783** | 2843 / **2543** |

Re-verified after the stage-19 grid re-tune (cells 7.5 px — shorter packed
rows could have moved it again):

| particles | verlet serial→packed | spatial serial→packed |
|---|---|---|
| 16000 | **1399** / 1498 | **1420** / 1478 |
| 20000 | **1723** / 1881 | 1937 / **1805** |
| 24000 | 2425 / **1992** | 2451 / **2272** |

## Decision

The crossover moved up: packed now wins clearly only from ~24k (spatial
flips a little earlier, verlet a little later; the 16–20k margins are inside
the ±10 % VM noise band). Default `PAR_MIN_PARTICLES` **16 000 → 22 000**,
the midpoint of the disagreement zone — it routes every measured size to the
winning engine except 20k-spatial, where it gives up ~7 % rather than
costing 20k-verlet ~9 % the other way. Runtime-tunable for machines with
more cores, where the packed engine's crossover will sit lower.
