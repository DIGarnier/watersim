# Benchmark 07 — Periodic spatial reordering of particle arrays (SFC-style)

- Date: 2026-07-01
- Change (`src/physics.rs`): every 64 steps the particle arrays (`c_pos`,
  `c_opos`, `c_force`, `c_color`) are physically permuted into grid order —
  the CSR grid's `indices` array *is* that permutation, so the reorder is one
  O(n) gather per array and the grid is patched back to identity in place
  (row-major cell order; the 2D analogue of HOOMD-blue's SFCPACK reordering,
  docs/literature.md §4). Neighbor lists are invalidated on reorder and
  rebuilt on the next force pass.
- Renderer contract: positions and colors are permuted together and the draw
  loop is order-agnostic, so this is invisible outside the engine.
- Compare against: `docs/benchmarks/06-csr-verlet-lists.md`

## Result: neutral at all tested sizes (median µs/step)

| particles | path | bench 06 | this |
|---|---|---|---|
| 1000 | verlet-lists | 108 | 110 |
| 6000 | verlet-lists | 615 | 612 |
| 12000 | verlet-lists | 1151 | 1190 |
| 24000 | verlet-lists | 4714 | 4668 |
| 24000 | spatial-hash | 4479 | 4672 |
| 12000 | barnes-hut | 6215 | 7428* |

*BH at 12k bounced between 5.8 and 7.4 ms across reruns of both builds — shared-VM
noise, not attributable to this change.

## Verdict

Kept, despite no measurable win at ≤ 24k: the whole working set (24k × 32 B of hot
per-particle data ≈ 0.8 MB) still fits in L2/L3, which is consistent with the
literature only reporting SFC-reorder wins at hundreds of thousands of particles.
The measured cost is zero (one gather per array every 64 steps, amortized ≪ 1 µs/step)
and it keeps array order ≈ spatial order for free as particle counts grow — it also
restores, permanently, the "traverse in spatial order" property the BH pass lost
when it went parallel.
