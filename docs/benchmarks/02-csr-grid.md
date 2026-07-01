# Benchmark 02 — CSR counting-sort grid, built once per step

- Date: 2026-07-01
- Change (`src/physics.rs`):
  - Replaced the `Vec<Vec<usize>>` spatial hash (18 000 heap vectors, cleared and
    refilled up to 9× per step) with a flat CSR grid built by counting sort
    (Green 2010; Hoetzlein 2014 — see `docs/literature.md` §1).
  - The grid is built **once per step** and reused by the force pass and all 8
    constraint-solver iterations (particles move ≪ one 10 px cell per 1/480 s step,
    so the one-cell stencil doubles as a Verlet skin).
  - Sweeps visit only **occupied** cells, via a forward half-stencil so each
    unordered cell pair is processed once; forces apply Newton's third law per pair.
  - Verlet-list rebuilds gather from the same CSR grid.
- Behavior notes: each colliding pair is now projected once per solver iteration
  instead of twice (the old full-stencil sweep hit every pair from both sides), and
  the spatial-hash force path now applies equal-and-opposite forces (the old code
  only accumulated onto one particle of each same-cell pair). Both changes make the
  solver *more* symmetric/correct; sanity checks (no NaN, no escapes) pass.
- Compare against: `docs/benchmarks/01-build-config.md`

## Result

| particles | force path | steps | mean µs/step | median | p95 | min | max | 480 Hz real-time? | sane? |
|---|---|---|---|---|---|---|---|---|---|
| 1000 | barnes-hut | 300 | 1421 | 1429 | 1640 | 1017 | 1990 | yes | ok |
| 1000 | verlet-lists | 300 | 136 | 131 | 167 | 114 | 200 | yes | ok |
| 1000 | spatial-hash | 300 | 150 | 146 | 179 | 119 | 312 | yes | ok |
| 3000 | barnes-hut | 200 | 6018 | 6134 | 6542 | 4080 | 7220 | NO | ok |
| 3000 | verlet-lists | 200 | 410 | 403 | 501 | 364 | 540 | yes | ok |
| 3000 | spatial-hash | 200 | 467 | 462 | 521 | 408 | 584 | yes | ok |
| 6000 | barnes-hut | 120 | 12518 | 13131 | 13605 | 9195 | 14446 | NO | ok |
| 6000 | verlet-lists | 120 | 841 | 814 | 1008 | 767 | 1193 | yes | ok |
| 6000 | spatial-hash | 120 | 939 | 931 | 1007 | 830 | 1040 | yes | ok |
| 12000 | barnes-hut | 60 | 24315 | 23210 | 29053 | 20567 | 29324 | NO | ok |
| 12000 | verlet-lists | 60 | 1660 | 1616 | 2043 | 1568 | 2051 | yes | ok |
| 12000 | spatial-hash | 60 | 1822 | 1815 | 1962 | 1633 | 2111 | yes | ok |

## Delta vs benchmark 01 (median µs/step)

| particles | verlet-lists | spatial-hash | barnes-hut |
|---|---|---|---|
| 1000 | 2931 → **131** (22.4×) | 3467 → **146** (23.7×) | 4483 → 1429 (3.1×) |
| 3000 | 3267 → **403** (8.1×) | 3841 → **462** (8.3×) | 9493 → 6134 (1.5×) |
| 6000 | 3882 → **814** (4.8×) | 4599 → **931** (4.9×) | 17009 → 13131 (1.3×) |
| 12000 | 5020 → **1616** (3.1×) | 5649 → **1815** (3.1×) | 27562 → 23210 (1.2×) |

**The grid-based paths are now real-time at 480 Hz for all tested sizes (≤ 12 000
particles).** The dominant fixed cost (full-grid sweeps + per-cell allocations, 8×
per step) is gone. Barnes-Hut improved only from the collision-solver share of its
step; its pointer-chasing per-step tree build now dominates — addressed next.
