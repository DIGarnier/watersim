# 23 — Cluster gather: the SIMD width lever, cashed for the packed solver

Stage 10's honest verdict was that explicit SIMD couldn't beat the scalar
kernels because packed stencil rows hold only 3–6 candidates (~1–2 particles
per cell) — the lanes were empty. The GROMACS answer (literature §4) is to
make work width independent of cell occupancy: process an *i-cluster* of
consecutive particles against a shared candidate set.

Implementation, packed Jacobi correction gather only:

- Packed order is cell order, so 4 consecutive packed particles usually sit
  on one grid row within a few cells of each other. Such a cluster shares
  one **union stencil** (`stencil_rows_span`): x from first.x−1 to last.x+1,
  same 3 row-lines.
- `gather_correction4` walks that candidate set once, broadcasting each
  partner against 4 lanes — the inner body is 4-wide SIMD with full lanes.
  Candidate loads drop ~4×.
- Clusters that straddle a grid row, span > 3 cells (sparse regions), or the
  tail fall back to the scalar kernel.

**Exactness**: any partner within CONTACT of a member is inside that
member's own 3×3 stencil (CONTACT ≤ cell size), so the union's extra
candidates fail the distance mask and contribute exact zero terms; per-lane
candidate order is unchanged. Results are bit-identical to the scalar
gather — confirmed by the A/B: penetration numbers at *every* size matched
the previous run to the printed digit. The packed *force* gather is left
scalar: its 1/r² kernel has no distance cutoff, so a union stencil would
change (slightly widen) the effective interaction set rather than mask out.

## Measured (defaults, full run + phase view, A/B vs stage 21 state)

The packed engine runs at ≥ 22k; smaller sizes are the untouched serial
engine (their rows moved < noise, pen identical — the control).

| metric @24k | verlet-lists | spatial-hash |
|---|---|---|
| solver phase µs | 1291 → **751** (−42 %) | 1460 → **854** (−42 %) |
| mean µs/step | 1798 → **1276** (−29 %) | 1704 → **1467** (−14 %) |

24k Barnes-Hut also dropped 3356 → 2916 µs (its solver is the same packed
engine at that size). With the solver deflated, the grid paths at 24k are
now bounded by the force pass + grid build again.

## Decision

Adopted (it is the packed engine's default gather). The same treatment for
the packed force gather was tried with a cutoff-preserving index-mask
formulation and measured ~15 % *slower* — that kernel is FLOP-bound, not
load-bound; see stage 24 for the rejection record.
