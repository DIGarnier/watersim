# 14 — Multiple time stepping: far-field forces on a slow clock (RESPA)

The stage-12 phase profile says the force pass dominates the BH path (75–82 %
of the step) and is the #2 term on the grid paths. But the 1/r² repulsion
field is *smooth*: at 480 Hz a particle moves a tiny fraction of a grid cell
per substep, so recomputing that field every substep buys almost nothing.
Classic molecular dynamics answer (r-RESPA, Tuckerman/Berne/Martyna 1992;
docs/literature.md §7): evaluate the slow long-range force every K substeps
and hold it, while the fast stiff part — here the contact projection —
keeps the full rate.

Implementation: `Physics` caches the whole force-pass output (`c_farfield`)
and replays it for K−1 substeps (`FORCE_INTERVAL`, `set_force_interval`).
The cache is invalidated when particles are added and permuted alongside the
periodic spatial reordering. Applies to every force path — BH tree build +
traversal, verlet gather, grid gather — since all of them only produce the
smooth far field; contacts are handled by the solver.

## Measured (sweep, 240 steps from a 1 s-settled state, vs stage-11 reference)

Timing note: with MTS the step-time distribution is bimodal (K−1 cheap steps,
1 refresh step), so the **mean** is the honest cost metric — the median only
sees cheap steps. Both are reported; p95 shows the refresh spikes.

| particles | path | variant | mean µs | Δmean | mean pen % | deep pairs | drift % |
|---|---|---|---|---|---|---|---|
| 12000 | barnes-hut | reference | 8908 | +0% | 1.51 | 0 | 0.00 |
| 12000 | barnes-hut | θ=0.5 K=4 | 3997 | -55% | 1.50 | 0 | 9.07 |
| 12000 | barnes-hut | θ=0.9 K=1 | 6488 | -27% | 1.55 | 0 | 10.84 |
| 12000 | barnes-hut | θ=0.9 K=4 | 3318 | -63% | 1.51 | 0 | 10.74 |
| 12000 | barnes-hut | θ=0.9 K=8 | 3000 | -66% | 1.52 | 0 | 11.12 |
| 12000 | verlet-lists | K=4 | 2744 | -17% | 5.50 | 14 | 29.03 |
| 16000 | spatial-hash | K=4 | 2058 | -28% | 5.92 | 0 | 6.84 |
| 24000 | spatial-hash | K=4 | 3172 | -24% | 14.10 | 727 | 17.65 |

(Full tables incl. 3000-particle BH and all metrics: stage-15 file and the
series README. Grid-path reference rows: verlet 12k = 3318 µs / 5.00 % /
20 deep; spatial 16k = 2864 µs / 5.78 % / 0 deep; spatial 24k = 4196 µs /
14.03 % / 732 deep — the 24k pen/deep numbers are scenario pollution, see
stage 12.)

## Reading the quality columns

- **Contact quality is untouched by K=4 on every path** — mean penetration
  and deep-pair counts match the reference within noise. This is the RESPA
  prediction: the stiff part (contacts) never left the fast clock.
- **Density drift** must be read against the chaos-floor control (reference
  physics + a 1e-3 px nudge on one particle): the grid-path states fully
  decorrelate from that nudge alone (28.7 % at 12k verlet, 17.3 % at 24k),
  so their drift column says "different trajectory", not "worse physics".
  The BH states are stable (floor 0.3–0.8 %), and there *any* knob change —
  including the most conservative θ=0.7 K=1 — lands at the same ~9–15 %
  decorrelation plateau; K contributes nothing beyond it.
- K=8 starts to show single-pair max-pen outliers (38 % vs ~24 %) for a
  marginal further gain, so the default stops at **K=4**.

## Decision

`FORCE_INTERVAL = 4` adopted as default. One caveat for real-time use: with
MTS the *refresh* substeps still cost the full force pass (p95 column), so
the per-substep 480 Hz budget is only met in the mean, amortized across each
60 Hz visual frame (8 substeps = 2 refreshes at K=4).
