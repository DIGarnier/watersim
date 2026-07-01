# Benchmark series — 2026-07-01 optimization pass

Every optimization in this pass was measured before and after with the same
harness; nothing was assumed. One file per stage, numbered in the order the
changes landed (each stage includes all previous ones):

| # | file | change | headline effect |
|---|---|---|---|
| 00 | [00-baseline.md](00-baseline.md) | none (baseline) | nothing met the 480 Hz budget |
| 01 | [01-build-config.md](01-build-config.md) | thin LTO + target-cpu=native | ~2–5%; `codegen-units=1` rejected (−15–20%) |
| 02 | [02-csr-grid.md](02-csr-grid.md) | CSR counting-sort grid, built once/step | **3–22×** on grid paths; real-time to 12k |
| 03 | [03-barnes-hut-arena.md](03-barnes-hut-arena.md) | arena BH tree, sqrt-free θ, bucketed leaves | 1.3× on BH path |
| 04 | [04-small-steps.md](04-small-steps.md) | 8 → 4 solver iterations (Macklin 2019) | 1.5× with quantified quality trade |
| 05 | [05-rayon-parallel.md](05-rayon-parallel.md) | rayon: BH traversal, force gather, colored solver | BH 1.9–3×; grid ~2× at 24k |

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

## End-to-end: baseline → final (median µs/step)

| particles | barnes-hut | verlet-lists | spatial-hash |
|---|---|---|---|
| 1000 | 4608 → **529** (8.7×) | 3038 → **87** (35×) | 3517 → **110** (32×) |
| 3000 | 9729 → **1812** (5.4×) | 3437 → **241** (14×) | 3933 → **306** (13×) |
| 6000 | 17556 → **3665** (4.8×) | 4065 → **477** (8.5×) | 4638 → **600** (7.7×) |
| 12000 | 29098 → **6113** (4.8×) | 5369 → **1019** (5.3×) | 6033 → **1177** (5.1×) |

At baseline, **no configuration** could hold the 480 Hz substep rate at any size.
Now the grid paths hold it beyond 12 000 particles and Barnes-Hut holds it to ~3 000.

## Where the remaining time goes / follow-ups

- **Barnes-Hut** is θ-limited arithmetic: at θ = 0.5 with a global 1/r² force each
  particle legitimately touches hundreds of nodes. Options: raise θ (accuracy trade),
  fast multipole, or the GPU compute path (`resources/physics_compute.wgsl` exists
  but was never wired up).
- **Proper Small Steps**: raising the substep rate while dropping to 1 iteration
  needs the integrator's `a·dt` term fixed to `a·dt²` first (see 04).
- **SFC particle reordering** (docs/literature.md §4) may pay off past ~50k
  particles; at ≤24k the CSR grid order was measured neutral.
