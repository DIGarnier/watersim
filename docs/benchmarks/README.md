# Benchmark series

Every optimization is measured before and after with the same harness;
nothing is assumed. One file per stage, numbered in the order the changes
landed (each stage includes all previous ones).

## Second pass — 2026-07-13 (algorithmic accuracy/speed trades)

Final report with baseline→final tables:
[16-second-pass-report.md](16-second-pass-report.md).

| # | file | change | headline effect |
|---|---|---|---|
| 12 | [12-baseline-2026-07-13.md](12-baseline-2026-07-13.md) | none (fresh baseline of stage-11 code) | BH path: forces 75–82 % of step; grid paths: solver 60–73 % |
| 13 | [13-bh-theta.md](13-bh-theta.md) | BH force-**sign fix** (attraction → repulsion, bug since BH introduction) + measured θ, default 0.5 → 0.9 | 2.2× cheaper traversal at 6 % measured force error |
| 14 | [14-mts-farfield.md](14-mts-farfield.md) | multiple time stepping (r-RESPA): far field every 4 substeps, contacts every substep | −55…−66 % BH-path step; −17…−28 % grid paths; contact quality unchanged |
| 15 | [15-solver-relaxation.md](15-solver-relaxation.md) | solver iterations 4 → 3, SOR ω measured on both engines | −18/−32 % solver time at reference quality; ω ≥ 1.5 unsafe on Jacobi engine |
| 16 | [16-second-pass-report.md](16-second-pass-report.md) | — (report for stages 12–15) | BH path 2.5–4.2×, real-time through 12k (was 3k) |
| 17 | [17-smaller-particles.md](17-smaller-particles.md) | smaller particulates: BALL_SIZE 4 → 3 | 24k scenario no longer over-fills: max pen 100 % → 0–0.5 % on grid paths; mid sizes −15…−35 % pending stage 19 |
| 18 | [18-engine-crossover.md](18-engine-crossover.md) | re-swept serial↔packed solver threshold | PAR_MIN_PARTICLES 16k → 22k (crossover moved with the new cost profile) |
| 19 | [19-grid-retune.md](19-grid-retune.md) | grid cell 10 → 7.5 px (= contact + skin) | stencil candidates ×0.56: grid paths −25…−29 % at 12–24k |

Net (mean µs/step, defaults, same-day A/B chains): BH path 2.5–4.2× faster;
grid paths ~1.4× at solver-dominated sizes, then another ~1.3× from stages
17+19 at equal physics quality. **Every path holds the 480 Hz median budget
through 24 000 particles** (at pass start: no path at 24k, BH fell off at
6k; BH at 24k is median-real-time — its MTS refresh spikes amortize across
the frame, mean 3.7 ms). New harness modes: `--force-error`, `--sweep`
(quality metrics + chaos-floor control), `--engine-sweep`, `--soak`.

## First pass — 2026-07-01 (data structures & parallelism)

| # | file | change | headline effect |
|---|---|---|---|
| 00 | [00-baseline.md](00-baseline.md) | none (baseline) | nothing met the 480 Hz budget |
| 01 | [01-build-config.md](01-build-config.md) | thin LTO + target-cpu=native | ~2–5%; `codegen-units=1` rejected (−15–20%) |
| 02 | [02-csr-grid.md](02-csr-grid.md) | CSR counting-sort grid, built once/step | **3–22×** on grid paths; real-time to 12k |
| 03 | [03-barnes-hut-arena.md](03-barnes-hut-arena.md) | arena BH tree, sqrt-free θ, bucketed leaves | 1.3× on BH path |
| 04 | [04-small-steps.md](04-small-steps.md) | 8 → 4 solver iterations (Macklin 2019) | 1.5× with quantified quality trade |
| 05 | [05-rayon-parallel.md](05-rayon-parallel.md) | rayon: BH traversal, force gather, colored solver | BH 1.9–3×; grid ~2× at 24k |
| 06 | [06-csr-verlet-lists.md](06-csr-verlet-lists.md) | flat CSR neighbor lists, lazy displacement rebuild | correctness fix; ≈ direct-pass cost in motion, free at rest |
| 07 | [07-spatial-reorder.md](07-spatial-reorder.md) | periodic SFC-style particle reordering | neutral ≤24k (kept: free, scales) |
| 08 | [08-bare-metal-kernels.md](08-bare-metal-kernels.md) | hardware rsqrt, store discipline, dead-work removal | 5–10% grid, ~5% BH |
| 09 | [09-bh-hot-cold-split.md](09-bh-hot-cold-split.md) | BH node hot/cold split + prefetch | ~3% BH where visible |
| 10 | [10-packed-simd-jacobi.md](10-packed-simd-jacobi.md) | phase profiling; packed-SoA Jacobi solver ≥16k (hybrid) | **−26/−27% at 24k**; solver shown to dominate grid paths |
| 11 | [11-bh-packed-leaves.md](11-bh-packed-leaves.md) | contiguous BH leaf members | neutral (kept: structure, no pointer-chase) |

## Method

- Harness: `benches/nbody.rs` — drives the **real** `lolballs::physics::Physics`
  engine headlessly (the pre-existing benchmarks simulated a reimplemented copy and
  were removed). Run: `cargo bench --no-default-features --bench nbody`.
- Deterministic dense-block scenario, adaptive dt off, fixed dt = 1/480 s;
  per-step wall times reported as mean/median/p95/min/max.
- **480 Hz real-time?** = median ≤ 2083 µs, the budget for one physics substep.
- **max pen %** = worst pair overlap at the end (contact-quality metric).
- Sanity gates on every run: no NaN positions, no particles outside the walls.
- Machine: 4-core Intel Xeon @ 2.10 GHz (shared cloud VM; ±10% noise band —
  regressions/wins near that band were re-run A/B before being believed).

## End-to-end: baseline → final (median µs/step, stages 00 → 09)

> Stages 10–11 were measured on a visibly slower VM day (stage-09 code re-ran
> ~10–15% above its recorded numbers), so they are excluded from this table's
> absolute values; their same-session deltas are in their own files. Net stage-10
> effect at 24k: verlet 4689 → 3450, spatial 4204 → 3096 (same-day A/B).

| particles | barnes-hut | verlet-lists | spatial-hash |
|---|---|---|---|
| 1000 | 4608 → **505** (9.1×) | 3038 → **104** (29×) | 3517 → **102** (34×) |
| 3000 | 9729 → **1690** (5.8×) | 3437 → **304** (11×) | 3933 → **303** (13×) |
| 6000 | 17556 → **3650** (4.8×) | 4065 → **587** (6.9×) | 4638 → **589** (7.9×) |
| 12000 | 29098 → **6030** (4.8×) | 5369 → **1091** (4.9×) | 6033 → **1123** (5.4×) |
| 24000 | — | (serial baseline ~6800) → **4185** | — → **4348** |

At baseline, **no configuration** could hold the 480 Hz substep rate at any size.
Now the grid paths hold it beyond 12 000 particles and Barnes-Hut holds it to ~3 000.
Stage-06 verlet numbers are not directly comparable to stage-05's: 05 reused stale
neighbor lists during fast motion; 06 fixed that (see its write-up).

## Where the remaining time goes / follow-ups

- **Barnes-Hut** is θ-limited arithmetic: at θ = 0.5 with a global 1/r² force each
  particle legitimately touches hundreds of nodes. Options: raise θ (accuracy trade),
  fast multipole, or the GPU compute path (`resources/physics_compute.wgsl` exists
  but was never wired up).
- **Proper Small Steps**: raising the substep rate while dropping to 1 iteration
  needs the integrator's `a·dt` term fixed to `a·dt²` first (see 04).
- **SFC particle reordering** is now in (stage 07), measured neutral ≤24k as
  predicted; its value is headroom past ~50k particles.
- Explicit SIMD batching of pair kernels was attempted in stage 10: it pays only
  in the parallel regime (≥16k) — at this sim's density (~1–2 particles per cell)
  packed rows are 3–6 elements, too short for SIMD to beat the scalar
  Gauss-Seidel sweep at small N. Denser packing (GROMACS-style 4×4 clusters
  independent of cell size) is the remaining width lever.
- Per-phase timings (`cargo bench --no-default-features --bench nbody -- --phases`)
  now show where each configuration spends its step; the solver dominates grid
  paths, traversal dominates BH.
