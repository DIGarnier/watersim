# Benchmark 11 — Contiguous Barnes-Hut leaf members

- Date: 2026-07-02
- Change (`src/physics.rs`): after the tree build, a finalize pass copies each
  leaf's members into contiguous `leaf_x`/`leaf_y` arrays and stamps
  (offset, count) into the hot nodes. The exact near-field loop becomes a
  branchless masked sweep over that slice instead of chasing the intrusive
  `next[]` list through scattered `ppos` reads (a dependent-load chain).
  The intrusive list still exists but is now build-only (cold).
- Compare against: `docs/benchmarks/10-packed-simd-jacobi.md` (same-session A/B,
  2 runs per build)

## Result (barnes-hut medians)

| particles | before (2 runs) | after (2 runs) | delta |
|---|---|---|---|
| 3000 | 1954 / 2026 | 2016 / 2009 | neutral |
| 6000 | 4137 / 4214 | 4259 / 4197 | neutral |
| 12000 | 7659 (noisy day) | 7225 / 7529 | neutral |

Max-pen values shift by ~0.1–0.7 points from floating-point reordering of the
near-field sums; sanity gates pass.

## Verdict

Kept, measured neutral: with θ = 0.5 the traversal's time is dominated by
upper-tree aggregate evaluations (one θ-test + rsqrt per visited node), and
near-leaf exact interactions — the only thing this change touches — are a small
slice, with rows of ≤ 8 members that are too short for meaningful SIMD (the same
lesson as benchmark 10's serial regression). The structure is still strictly
better: the O(n) finalize is amortized noise, the hot path loses its last
pointer-chase, and a future explicit-SIMD near-field would need exactly this
layout. This closes the cheap levers on the BH path — remaining options are the
θ/accuracy trade, fast multipole, or the GPU compute path.
