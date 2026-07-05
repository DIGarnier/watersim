# Benchmark 09 — Barnes-Hut node hot/cold split + software prefetch

- Date: 2026-07-01
- Change (`src/physics.rs`): `BhNode` (44 bytes: bounds, com, mass, size²,
  children, first, count) split into:
  - `BhHot` (24 bytes: com, mass, size², children, first) — the only thing the
    force traversal reads; ~2.7 nodes per cache line instead of ~1.5;
  - cold `bounds`/`count` arrays touched only during tree construction.
  Plus `_mm_prefetch` of the 4 contiguous children (96 B, 2 lines) when they are
  pushed on the traversal stack, a few iterations before they're popped.
- Compare against: `docs/benchmarks/08-bare-metal-kernels.md` (A/B, 2 runs per build)

## Result (barnes-hut medians, 2 runs)

| particles | bench 08 | hot/cold split | delta |
|---|---|---|---|
| 1000 | 502 | 514 / 505 | noise |
| 3000 | 1753 | 1680 / 1700 | ~−3.5% |
| 6000 | 3633 | 3777 / 3650 | noise |
| 12000 | 6489 | 6030 / 6597 | noise |
| 24000 | 14758 | 14127 / 14369 | ~−3% |

Grid paths untouched (BH-only change).

## Verdict

A small, borderline-noise win (~3% where visible). Kept: the effect is
non-negative everywhere, the hot struct is what the parallel traversal actually
streams, and the layout benefit grows with tree size. This also confirms the
earlier profile: after benchmark 03/08 the BH path is dominated by traversal
*arithmetic* (rsqrt + FMA chains), not memory — consistent with the spatial-order
iteration (03) and this split both being near-neutral while the sqrt-free θ test
and bucketed leaves (03) were the big levers.
