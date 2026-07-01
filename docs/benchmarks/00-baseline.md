# Benchmark 00 — Baseline (unmodified physics)

- Date: 2026-07-01
- Commit: ca66922 (plus benchmark-harness refactor, no physics changes)
- Machine: Intel Xeon @ 2.10 GHz, 4 cores, 15 GiB RAM, Linux 6.18.5
- Toolchain: rustc 1.94.1, `cargo bench --no-default-features --bench nbody`
- Profile: default `release` (no LTO, default codegen-units)
- Methodology: dense resting particle block, adaptive dt disabled, fixed dt = 1/480 s.
  Warmup = steps/10 + 5, then per-step wall time collected individually.
  "480 Hz real-time?" = median step time ≤ 2083 µs (the physics thread's substep budget).
  Sanity = no NaN positions, no particles escaped the walls.

| particles | force path | steps | mean µs/step | median | p95 | min | max | 480 Hz real-time? | sane? |
|---|---|---|---|---|---|---|---|---|---|
| 1000 | barnes-hut | 300 | 4590 | 4608 | 4835 | 3918 | 6288 | NO | ok |
| 1000 | verlet-lists | 300 | 3065 | 3038 | 3302 | 2940 | 3824 | NO | ok |
| 1000 | spatial-hash | 300 | 3527 | 3517 | 3774 | 3300 | 4012 | NO | ok |
| 3000 | barnes-hut | 200 | 9584 | 9729 | 10062 | 7180 | 10353 | NO | ok |
| 3000 | verlet-lists | 200 | 3461 | 3437 | 3636 | 3355 | 3818 | NO | ok |
| 3000 | spatial-hash | 200 | 4022 | 3933 | 4408 | 3796 | 5082 | NO | ok |
| 6000 | barnes-hut | 120 | 16831 | 17556 | 18363 | 12576 | 19140 | NO | ok |
| 6000 | verlet-lists | 120 | 4106 | 4065 | 4375 | 3960 | 4584 | NO | ok |
| 6000 | spatial-hash | 120 | 4646 | 4638 | 4871 | 4399 | 5048 | NO | ok |
| 12000 | barnes-hut | 60 | 29990 | 29098 | 36053 | 24625 | 36336 | NO | ok |
| 12000 | verlet-lists | 60 | 5454 | 5369 | 5951 | 5283 | 6078 | NO | ok |
| 12000 | spatial-hash | 60 | 6034 | 6033 | 6160 | 5735 | 6274 | NO | ok |

## Observations

- **No configuration meets the 480 Hz real-time budget**, at any particle count.
- The default configuration (Barnes-Hut) is the **slowest** path: 4.6 ms/step at 1k particles,
  30 ms/step at 12k. The pointer-based quadtree is rebuilt every step with one `Box`
  allocation per subdivided node, and force traversal is recursive.
- There is a large **fixed overhead (~3 ms/step)** independent of particle count: the
  collision solver runs 8 iterations per step, and each iteration rebuilds the spatial
  hash (`Vec<Vec<usize>>`, 18 000 cells) and sweeps *all* cells — including empty ones —
  copying neighbor-cell contents into a scratch `others` buffer.
- Verlet-lists is the fastest existing path but scales poorly per particle
  (~0.2 µs/particle marginal) on top of that fixed cost.
