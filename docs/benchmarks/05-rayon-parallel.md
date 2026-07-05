# Benchmark 05 — Parallelism with rayon (4-core machine)

- Date: 2026-07-01
- Change (`src/physics.rs`, `Cargo.toml`):
  1. **Barnes-Hut force loop** — traversal is read-only, so particles are processed
     with `par_iter` using a per-thread scratch stack (`for_each_init`). Always
     parallel: one region of heavy independent work.
  2. **Grid force paths** — the Newton's-third-law scatter (thread-unsafe writes to
     both particles) becomes a symmetric *gather*: each particle sums its full
     neighbor list. 2× the arithmetic, zero write conflicts (the trade GROMACS/LAMMPS
     make on wide machines; docs/literature.md §5).
  3. **Constraint solver** — parallel projection via **cell coloring**: occupied
     cells are bucketed into 6 color classes `(x mod 3) + 3·(y mod 2)`; with the
     forward half-stencil {E, SW, S, SE} two same-color cells always touch disjoint
     particles, so classes run sequentially and cells within a class in parallel.
  4. 2 and 3 only engage at ≥ 16 000 particles (`PAR_MIN_PARTICLES`); below that the
     serial paths win — see the threshold study.
- The 24 000-particle configuration was added to the harness to measure past the
  crossover. (Its max-pen numbers run ~100% for *all* configurations because a
  160-row column of particles compresses hard mid-fall; compare like against like.)
- Compare against: `docs/benchmarks/04-small-steps.md`

## Result (final configuration)

| particles | force path | steps | mean µs/step | median | p95 | min | max | 480 Hz real-time? | max pen % | sane? |
|---|---|---|---|---|---|---|---|---|---|---|
| 1000 | barnes-hut | 300 | 539 | 529 | 683 | 332 | 2108 | yes | 88.4 | ok |
| 1000 | verlet-lists | 300 | 92 | 87 | 120 | 79 | 152 | yes | 22.8 | ok |
| 1000 | spatial-hash | 300 | 115 | 110 | 155 | 88 | 200 | yes | 24.5 | ok |
| 3000 | barnes-hut | 200 | 1829 | 1812 | 2250 | 1116 | 3027 | yes | 13.4 | ok |
| 3000 | verlet-lists | 200 | 256 | 241 | 342 | 223 | 494 | yes | 0.0 | ok |
| 3000 | spatial-hash | 200 | 314 | 306 | 361 | 264 | 388 | yes | 0.0 | ok |
| 6000 | barnes-hut | 120 | 3609 | 3665 | 4347 | 2395 | 6483 | NO | 14.2 | ok |
| 6000 | verlet-lists | 120 | 498 | 477 | 686 | 454 | 711 | yes | 0.0 | ok |
| 6000 | spatial-hash | 120 | 604 | 600 | 653 | 518 | 838 | yes | 0.0 | ok |
| 12000 | barnes-hut | 60 | 6204 | 6113 | 7882 | 4805 | 8278 | NO | 19.4 | ok |
| 12000 | verlet-lists | 60 | 1061 | 1019 | 1403 | 932 | 1566 | yes | 0.0 | ok |
| 12000 | spatial-hash | 60 | 1179 | 1177 | 1251 | 1044 | 1305 | yes | 0.1 | ok |
| 24000 | barnes-hut | 40 | 15249 | 14823 | 18847 | 12075 | 21642 | NO | 98.5 | ok |
| 24000 | verlet-lists | 40 | 3746 | 3744 | 5958 | 1859 | 8875 | NO | 96.5 | ok |
| 24000 | spatial-hash | 40 | 4739 | 4707 | 6141 | 3470 | 6444 | NO | 100.0 | ok |

## Delta vs benchmark 04 (median µs/step)

| particles | barnes-hut | verlet-lists |
|---|---|---|
| 1000 | 991 → **529** (1.9×) | 89 → 87 (—) |
| 3000 | 4190 → **1812** (2.3×, now real-time) | 275 → 241 (—) |
| 6000 | 9307 → **3665** (2.5×) | 580 → 477 (—) |
| 12000 | 18528 → **6113** (3.0×) | 1113 → 1019 (—) |
| 24000 | ~35000 (est.) → **14823** | 6799 (serial, measured) → **3744** (1.8×) |

## Threshold study (why 16 000)

A/B at 24k (verlet median): all-serial 6799 · forces-parallel only 6867 (flat) ·
forces+solver parallel **3498–3744** (≈1.9×) · solver-parallel+serial-forces 3980.
At 12k every combination lands within the ±10% noise band of this shared VM; at
3–6k full parallelism was a clear **regression** (3k: 275 → 605 µs — 24 parallel
regions per step of overhead). Hence: BH unconditional, everything else ≥ 16k.
