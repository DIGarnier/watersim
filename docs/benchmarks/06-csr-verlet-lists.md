# Benchmark 06 — Flat CSR neighbor lists + displacement-triggered ("lazy") rebuilds

- Date: 2026-07-01
- Change (`src/physics.rs`, verlet-lists path only):
  1. **CSR layout** — `Vec<NeighborList>` (one heap `Vec<usize>` per particle) becomes
     one flat `neighbors: Vec<u32>` with per-particle `start` offsets, built in a
     single append pass from the step-level grid. No per-particle allocations,
     contiguous force-pass reads.
  2. **Displacement-triggered rebuild** (standard MD policy; docs/literature.md §1):
     lists are only rebuilt when some particle has drifted > skin/2 from its position
     at build time — replacing the fixed every-10-frames cadence, which silently
     computed forces from stale lists during fast motion.
  3. **Lazy fallback**: naive "rebuild whenever stale" rebuilt every step under fast
     motion and was measured at 8454 µs (24k median) — 1.8× *worse* than the direct
     cell pass, because a rebuild gather costs as much as computing the forces
     directly. So when lists are stale the engine takes the direct cell pass and only
     re-attempts list mode every 16 steps.
- Compare against: `docs/benchmarks/05-rayon-parallel.md`

## Result (verlet-lists path; other paths unchanged within noise)

| particles | bench 05 median µs | naive displacement rebuild | **lazy (final)** |
|---|---|---|---|
| 1000 | 87 | 126 | 108 |
| 3000 | 241 | 350 | 317 |
| 6000 | 477 | 742 | 615 |
| 12000 | 1019 | 1442 | 1151 |
| 24000 | 3744 | 8454 | 4714 |

## Honest verdict

In this benchmark's worst case — a block in continuous fast fall, where *every*
step exceeds the skin — the lazy path converges to the direct cell pass
(24k: 4714 vs 4479 for spatial-hash; the small gap is the periodic re-attempt).
That is ~10–25% **slower** than benchmark 05's numbers, because 05 was reusing
up-to-10-frame-old neighbor lists and therefore computing forces against stale
neighborhoods; matching that speed by accepting stale forces is a correctness
regression, not an optimization. Where the sim actually spends most of its life
(settled or slowly moving water), the new policy does **zero** rebuilds instead
of one every 10 frames and keeps the 3× shorter pair lists — the settled-pile
phase of the 1k run shows lists staying valid across long stretches.

Layout note: the CSR flattening itself (measured before the policy change) was
worth a few percent on the force pass; the policy dominates the delta.
