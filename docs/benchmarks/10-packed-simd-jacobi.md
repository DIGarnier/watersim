# Benchmark 10 — Phase profiling + packed-SoA Jacobi solver (hybrid engine)

- Date: 2026-07-02
- **Drift warning:** the shared VM is measurably slower this session (stage-09 code
  re-run today: BH 3k = 2001–2067 µs vs 1753 recorded yesterday). All conclusions
  below come from same-session A/B runs, never from cross-day comparison.

## Step 0: phase instrumentation (`--phases` bench mode)

`Physics` now fills the per-phase `PerformanceStats` fields. Mean µs/step before
this change:

| particles | path | integrate | grid build | tree build | forces | solver | total |
|---|---|---|---|---|---|---|---|
| 12000 | barnes-hut | 42 | 100 | 482 | 4741 | 1409 | 6296 |
| 12000 | verlet-lists | 32 | 100 | 0 | 327 | 633 | 1097 |
| 24000 | verlet-lists | 95 | 220 | 0 | 1155 | 2843 | 4316 |
| 24000 | spatial-hash | 99 | 227 | 0 | 905 | 2977 | 4210 |

**The solver, not the force pass, dominates the grid paths (~60–70%)** — so the
"SIMD the force kernel" plan was re-aimed at the solver.

## The change

Positions are packed once per step into CSR-ordered SoA arrays (`px`/`py`), where a
cell's 3×3 neighborhood is **3 contiguous ranges** (row-major adjacency). On the
packed arrays:
- the spatial-hash force pass becomes a branchless masked gather over those rows;
- the contact solver becomes **Jacobi**: gather all half-corrections read-only,
  apply in one sweep. Order-independence removes the 6-color machinery — a step is
  5 parallel regions (1 force + 4 solver) instead of up to 25.

## Negative result → hybrid

The pure packed engine **regressed 2–2.5× at serial sizes** (12k verlet 1091 →
2766 µs): with ~1–2 particles per 10 px cell, the "vectorizable" rows are 3–6
elements — no SIMD payoff — while the symmetric gather does every pair twice and
pays an extra apply pass. The GROMACS cluster-pair trick presumes dense clusters;
this system is too sparse per cell for it below the parallel regime.

Final form: **hybrid dispatch on the parallel threshold** — scalar Gauss-Seidel
half-stencil sweep (stage-08 engine) below 16k particles, packed Jacobi at ≥16k.
Threshold verified: at 12k the packed engine measured 1288–1447 vs 1185–1188
serial (2 runs each); at 24k it wins decisively.

## Result (same-session A/B, 24k medians)

| config | stage 09 (today) | hybrid | delta |
|---|---|---|---|
| verlet 24k | 4689 | 3396 / 3450 | **−27%** |
| spatial 24k | 4204 | 3096 / 3148 | **−26%** |
| barnes-hut 24k | 16551 | 15854 / 15891 | −4% (solver share) |
| 1k–12k, all paths | — | unchanged (serial engine) | 0 |

Quality: Jacobi's per-iteration pair correction is identical to Gauss-Seidel's
(each side gets its half); convergence is slightly softer in theory, but the max-pen
metric at 24k reads ~100% under both engines (the 160-row column compresses hard
mid-fall) and serial sizes keep their stage-08 values. Sanity gates pass everywhere.
