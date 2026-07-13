# 15 — Contact solver: iterations × over-relaxation, measured

The solver is 60–73 % of the grid-path step (stage 12). It ran a fixed 4
iterations of an under-relaxed projection (per-pair factor 0.375 = 0.75
stiffness × ½ per particle). Position-based-dynamics practice (Macklin et
al. 2014, docs/literature.md §8) treats the relaxation factor ω and the
iteration count as one budget: the per-pair factor is now 0.375·ω with both
`solver_iterations` and `solver_omega` runtime-tunable.

The sim has **two solver engines** (stage 10): serial Gauss-Seidel below
16k particles, packed parallel Jacobi at ≥16k — and SOR theory says they
tolerate ω very differently. They do.

## Measured (sweep, 240 steps from a 1 s-settled state)

Serial Gauss-Seidel engine (12k, verlet path; reference = it=4 ω=1.0,
3318 µs, mean pen 5.00 %, 20 deep pairs):

| variant | mean µs | Δmean | mean pen % | deep pairs |
|---|---|---|---|---|
| it=3 ω=1.0 | 2735 | -18% | 6.28 | 18 |
| it=2 ω=1.0 | 2233 | -33% | 9.87 | 82 |
| it=2 ω=1.25 | 2125 | -36% | 7.45 | 71 |
| it=2 ω=1.5 | 2106 | -37% | 11.68 | 95 |
| it=1 ω=1.5 | 1512 | -54% | 17.42 | 183 |
| it=1 ω=2.0 | 1521 | -54% | 22.46 | 403 |

Packed Jacobi engine (16k, spatial path — the clean read; the 24k scenario
over-fills the box, see stage 12. Reference = 2864 µs, mean pen 5.78 %,
0 deep pairs):

| variant | mean µs | Δmean | mean pen % | deep pairs | drift % |
|---|---|---|---|---|---|
| it=3 ω=1.0 | 1947 | -32% | 6.89 | 0 | 7.2 |
| it=2 ω=1.0 | 1721 | -40% | 13.22 | 0 | 15.0 |
| it=2 ω=1.25 | 1684 | -41% | 12.73 | 50 | 10.5 |
| it=2 ω=1.5 | 1808 | -37% | 16.64 | **564** | **50.6** |
| it=1 ω=1.5 | 1154 | -60% | 21.09 | 849 | 49.2 |

## What the surface says

- **Dropping 4 → 3 iterations is almost free on quality** on both engines
  (mean pen +1.1–1.3 pp, deep pairs unchanged) for −18 %/−32 % solver-path
  time. That's the new default.
- **Mild SOR helps at 2 iterations on Gauss-Seidel** (ω=1.25 beats ω=1.0:
  pen 7.45 vs 9.87 at equal cost), exactly the literature's promise — but
  quality is still visibly below 3 plain iterations (deep pairs ~70 vs ~18),
  and one run produced an exactly-coincident pair (which the solver's
  degenerate-pair mask can never re-separate).
- **The Jacobi engine cannot take ω ≥ 1.5**: 564 deep pairs and a 50 %
  density drift at 16k — textbook Jacobi over-relaxation divergence. ω=1.25
  is its edge (deep pairs appear: 0 → 50).
- it=1 is not a serious configuration on either engine.

## Decision

Defaults: **`SOLVER_ITERATIONS = 3`, `SOLVER_OMEGA = 1.0`** — stage-11
contact quality within noise at 18–32 % less solver time, safe on both
engines. The measured faster/mushier point (it=2 ω=1.25, another ~10 %
step time, ~2× mean penetration, small deep-pair counts) is one
`set_solver_iterations`/`set_solver_omega` call away for callers that want
it; ω ≥ 1.5 is documented as unsafe with the packed Jacobi engine.
