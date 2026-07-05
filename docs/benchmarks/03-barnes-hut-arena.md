# Benchmark 03 — Barnes-Hut rewrite: arena tree, sqrt-free θ test, bucketed leaves

- Date: 2026-07-01
- Change (`src/physics.rs`, Barnes-Hut path only):
  1. **Arena tree** — `QuadNode` (one `Box<[QuadNode;4]>` per subdivision, recursive
     traversal) replaced by a flat `Vec<BhNode>` arena with index-based children,
     iterative insertion, and explicit-stack traversal. Arena/stack capacity persists
     across frames → steady-state builds are allocation-free (Burtscher & Pingali
     2011; docs/literature.md §3).
  2. **Spatial-order force loop** — particles are walked in CSR-grid order so
     consecutive traversals reuse hot upper-tree nodes.
  3. **sqrt-free acceptance** — `size/dist < θ` evaluated as `size² < θ²·dist²`
     with `size²` cached per node; sqrt only on accepted nodes.
  4. **Bucketed leaves** — leaves hold up to `BH_LEAF_CAP` particles (intrusive
     per-particle lists); near-field work becomes exact particle-particle loops.
     Cap swept over {8, 16, 32, 64}: **8 was fastest** (6k quick-run median:
     7.5 / 9.4 / 9.5 / 10.9 ms), so BH_LEAF_CAP = 8.
- Accuracy: same θ = 0.5 and same force law. Bucketed leaves make near-field forces
  *exact* instead of aggregated, so the approximation is slightly better than before.
- Compare against: `docs/benchmarks/02-csr-grid.md`

## Result

| particles | force path | steps | mean µs/step | median | p95 | min | max | 480 Hz real-time? | sane? |
|---|---|---|---|---|---|---|---|---|---|
| 1000 | barnes-hut | 300 | 1056 | 1077 | 1194 | 721 | 1327 | yes | ok |
| 1000 | verlet-lists | 300 | 138 | 134 | 170 | 112 | 209 | yes | ok |
| 1000 | spatial-hash | 300 | 150 | 147 | 183 | 117 | 249 | yes | ok |
| 3000 | barnes-hut | 200 | 4444 | 4571 | 4859 | 3147 | 5723 | NO | ok |
| 3000 | verlet-lists | 200 | 432 | 424 | 522 | 383 | 549 | yes | ok |
| 3000 | spatial-hash | 200 | 496 | 486 | 578 | 434 | 618 | yes | ok |
| 6000 | barnes-hut | 120 | 9425 | 9855 | 10193 | 7067 | 11950 | NO | ok |
| 6000 | verlet-lists | 120 | 862 | 839 | 1049 | 794 | 1129 | yes | ok |
| 6000 | spatial-hash | 120 | 957 | 950 | 1044 | 842 | 1066 | yes | ok |
| 12000 | barnes-hut | 60 | 18331 | 17609 | 22000 | 15736 | 23837 | NO | ok |
| 12000 | verlet-lists | 60 | 1706 | 1664 | 2088 | 1620 | 2152 | yes | ok |
| 12000 | spatial-hash | 60 | 1897 | 1870 | 2109 | 1682 | 2169 | yes | ok |

## Delta vs benchmark 02 (barnes-hut median µs/step)

| particles | before | after | speedup |
|---|---|---|---|
| 1000 | 1429 | 1077 | 1.33× |
| 3000 | 6134 | 4571 | 1.34× |
| 6000 | 13131 | 9855 | 1.33× |
| 12000 | 23210 | 17609 | 1.32× |

Grid paths unchanged (within noise), as expected.

## Attribution and a negative result

Measured incrementally: the arena rewrite alone was **neutral** (≈0%) — the
bottleneck was never the tree build but the per-particle traversal. Spatial-order
iteration alone was also ≈0% (the 12k tree is only ~1.5 MB; it largely fits in L2).
The wins came from the sqrt-free θ test (~10–12%) and bucketed leaves (~20%).
The traversal is now arithmetic-bound; the remaining lever for this path is
parallelism (benchmark 05).
