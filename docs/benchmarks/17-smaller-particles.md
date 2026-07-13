# 17 — Smaller water particulates: BALL_SIZE 4 → 3

Stage 12 flagged that the 24k benchmark scenario over-fills the box: 160 rows
at 9 px spacing ≈ 1440 px in a 1200 px-tall box. Overflow rows wall-clamp
onto identical coordinates, mass-producing exactly-coincident pairs that the
solver's degenerate-pair mask can never separate — so every 24k quality
number measured the scenario, not the physics.

Fix at the source: smaller particulates. `BALL_SIZE` 4 → 3 shrinks the
contact distance to 6 px; the bench scenario spacing scales with it
(2.25·BALL = 6.75 px), so 24k particles now stand 1080 px tall and fit. This
also reads better as *water* — finer droplets.

## Quality effect at 24k (defaults, full-run pen column)

| path | max pen %, BALL=4 | max pen %, BALL=3 |
|---|---|---|
| barnes-hut | 96.0 | 29.9 |
| verlet-lists | 100.0 | **0.0** |
| spatial-hash | 91.3 | **0.5** |

The absorbing coincident-pair state is gone from the scenario; 24k quality
numbers are now meaningful.

## Performance effect (defaults, mean µs/step, same-day A/B)

| particles | barnes-hut | verlet-lists | spatial-hash |
|---|---|---|---|
| 6000 | 884 → 993 | 441 → 561 | 465 → 564 |
| 12000 | 1737 → 2022 | 805 → 1075 | 798 → 1094 |
| 24000 | 4234 → 4084 | 3037 → 2360 | 2992 → 2394 |

Two opposite effects, both expected:

- **Mid sizes get ~15–35 % slower**: contact radius shrank but the grid cell
  stayed 10 px, so each cell now holds ~(10/7.5)² ≈ 1.8× more candidate
  pairs per stencil relative to the interaction area. This is the motivation
  for stage 19's grid re-tune, not an accepted loss.
- **24k gets ~20 % faster on the grid paths**: the scenario is no longer a
  wall of permanently jammed contacts.

Comparisons across this boundary must be same-constant A/B; the stage-16
baseline→final table is unaffected (both sides ran BALL_SIZE = 4).
