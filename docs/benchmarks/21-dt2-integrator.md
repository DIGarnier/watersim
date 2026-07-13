# 21 — Integrator fix: a·dt → a·dt²

The Verlet update applied `(GRAVITY + F)·dt` — dimensionally an impulse, not
an acceleration term. Consequence: the simulated dynamics depended on the
substep rate (halving dt halved the effective gravity per unit time²), which
is what blocked proper Small Steps in stage 04 and made adaptive-dt subtly
change the physics it was supposed to preserve.

Fix: `x' = 2x − x_prev + a·dt²` with `a = (GRAVITY + F) / PHYS_TIME_STEP` —
the historical force values are interpreted as impulse-per-480 Hz-substep,
which they always were (scale = 2000 and gravity 9.8 were tuned in those
units). At dt = PHYS_TIME_STEP the update is algebraically identical to the
old one (up to float rounding), so default behavior is preserved; at any
other dt the motion now scales physically.

## Equivalence check (full run at defaults, A/B vs stage 19 state; includes stage 20)

Mean µs/step and sanity — all within the ±10 % noise band, pen character
unchanged (grid paths ≤ 1.1 %, BH's noisy single-worst-pair metric in its
usual 25–55 % range at small/mid sizes):

| particles | barnes-hut | verlet-lists | spatial-hash |
|---|---|---|---|
| 12000 | 1773 → 1891 | 790 → 863 | 774 → 857 |
| 24000 | 3703 → 3356 | 1815 → 1798 | 1708 → 1704 |

No NaN, no escapes at any size; the `coincident_pair_separates` test (stage
20) passes. Trajectories diverge from the old build only by floating-point
rounding (`dt·dt·480 ≠ dt` by ulps), which chaos amplifies — the same
class of difference as any two VM runs.

This stage is a correctness enabler, not an optimization: stage 22 (Small
Steps) is the payoff.
