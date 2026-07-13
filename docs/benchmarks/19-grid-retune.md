# 19 — Neighbor-grid cell size re-tuned to the smaller particles

Stage 17 shrank the contact distance to 6 px but left the neighbor grid at
10 px cells, so every 3×3 stencil scanned (30 px)² of candidates for a
(6 px + 1.5 px skin) interaction — the measured 15–35 % mid-size regression.
The exactness condition for the one-cell stencil is
`cell ≥ contact + skin = 7.5 px`; taking the cell to exactly 7.5 px cuts the
candidate area per stencil to (22.5/30)² = 0.56× and the box tiles evenly
(200 × 160 cells, up from 150 × 120).

Cost side: more cells → the counting-sort prefix pass and border-cell wall
sweep touch 1.78× more cells. Measured below — the pair-work savings dwarf it
(grid build +15–40 µs at 24k, solver −800–1050 µs).

## Full run (defaults: θ=0.9, K=4, it=3, ω=1.0, PAR_MIN=22k), same-day A/B vs stage 17

Mean µs/step:

| particles | barnes-hut | verlet-lists | spatial-hash |
|---|---|---|---|
| 3000 | 450 → 419 | 302 → 267 | 307 → 268 |
| 6000 | 993 → 894 | 561 → 455 | 564 → 477 |
| 12000 | 2022 → 1773 | 1075 → 790 | 1094 → 774 |
| 24000 | 4084 → 3703 | 2360 → **1815** | 2394 → **1708** |

Phase view at 24k (spatial): solver 2198 → 1460 µs, forces 161 → 105 µs,
grid build 174 → 214 µs — exactly the predicted trade.

**Milestone: every path now holds the 480 Hz budget (median ≤ 2083 µs)
through 24 000 particles** — at the start of this pass no path did at 24k,
and Barnes-Hut fell off at 6k. Honest caveat: for the BH path at 24k that's
the *median* (1961 µs); MTS makes the distribution bimodal and the mean
(3703 µs) still exceeds a single-substep budget — the refresh spikes
amortize across a 60 Hz frame rather than fitting every substep.

The stencil-exactness margin is now 1.5 px (cell − contact); particles move
a small fraction of that per 480 Hz substep, and the grid is rebuilt every
substep, so the one-cell stencil stays valid. Validation at this final
state: full-run max pen ≤ 1.1 % on the grid paths at every size ≥ 3k, and
the 5 s soak (2400 steps) stayed sane with mean pen 2.4–5.0 % and deep-pair
counts of 0 / 2 / 16 (12k BH / 12k verlet / 16k spatial) — long-compression
levels comparable to the pre-change reference sweeps.
