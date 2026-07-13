# 13 — Barnes-Hut: a measured θ, and a force-sign bug the instrument caught

Two outcomes in this stage:

1. a new `--force-error` bench mode that measures the tree's force error
   against the exact O(n²) sum — the standard treecode accuracy metric
   (docs/literature.md §6);
2. that instrument immediately exposed that the BH traversal had computed
   **attraction** while every exact path (grid, verlet, direct kernel)
   computes **repulsion** — a sign inversion present since BH was first
   introduced (commit 208a446), *before* the 2026-07-01 optimization pass.

## The sign bug

First run of the error instrument, before the fix:

| particles | θ | rel RMS force error |
|---|---|---|
| 3000 / 12000 | 0.3 … 1.1 | **1.93e0 – 2.00e0** |

A relative RMS error pinned at ≈ 2.0 for *every* θ is the signature of a
flipped vector (|F − (−F)| = 2|F|), not of approximation error. The traversal
used `center_of_mass − pos` (toward the mass = attraction); the exact kernel
`force()` uses `pos_a − pos_b` (repulsion). Both the far-field and the packed
leaf near-field loop were flipped, so the "Barnes-Hut" toggle silently swapped
the simulation between a pressure-like repulsive fluid and a self-gravitating
collapse. It also explains the baseline quality gap: BH runs showed max pen of
9–81 % where grid paths showed ~0 % — attraction squeezes the pile into the
contact solver.

Fixed by flipping both subtractions (`pos − center_of_mass`, `pos − leaf`).
BH now approximates the same physical system the exact paths compute, which
is the precondition for θ meaning anything at all.

## Measured θ error/cost curve (after the fix)

State: dense block settled 60 steps under reference physics; error is
relative RMS vs the exact all-pairs sum, sqrt(Σ|ΔF|²/Σ|F|²). Traversal time
is single-thread, same tree, timing only the force loop (relative scaling is
what matters).

| particles | θ | rel RMS force error | serial traversal ms | speedup vs θ=0.5 |
|---|---|---|---|---|
| 3000 | 0.3 | 0.38 % | 8.2 | 0.54× |
| 3000 | 0.5 | 1.40 % | 4.4 | 1.0× |
| 3000 | 0.7 | 2.98 % | 3.0 | 1.5× |
| 3000 | 0.9 | 6.05 % | 2.2 | 2.0× |
| 3000 | 1.1 | 9.71 % | 1.7 | 2.6× |
| 12000 | 0.3 | 0.42 % | 43.0 | 0.44× |
| 12000 | 0.5 | 1.33 % | 18.8 | 1.0× |
| 12000 | 0.7 | 2.83 % | 12.2 | 1.5× |
| 12000 | 0.9 | 5.84 % | 8.7 | 2.2× |
| 12000 | 1.1 | 8.45 % | 6.9 | 2.7× |

Error grows ≈ θ² as monopole theory predicts; cost falls ≈ 1/θ². For
reference, production astrophysics codes run θ = 0.7–1.0 at ~0.4–1 % rms
error on *gravity* benchmarks; this sim's error is higher at equal θ because
the bounded box keeps far-field contributions from averaging out as cleanly.

## Decision

Default θ raised 0.5 → **0.9**: ≈ 2.2× cheaper traversal for ~6 % force
error on a force whose job here is visual (spreading pressure). End-to-end
(stage-14 sweep): contact quality is unchanged at θ=0.9 (deep pairs 0, mean
pen within noise of reference), and its density drift sits on the same
~10–15 % trajectory-decorrelation plateau as the most conservative variant
measured (θ=0.7, K=1) — i.e. no measurable systematic damage beyond "a
different but equally valid trajectory" of this chaotic system. θ stays
runtime-tunable (`set_bh_theta`) — anything needing the old accuracy sets
0.5 back and pays the old price.
