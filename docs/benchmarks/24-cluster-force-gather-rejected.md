# 24 — Clustered *force* gather: measured, rejected

Stage 23's remaining follow-up: apply the i-cluster union-stencil treatment
to the packed force gather. The force kernel has no distance cutoff, so
instead of a distance mask the exactness mechanism was a **per-lane
index-range mask** — candidate k contributes to cluster member l only when
k lies inside l's own packed row subrange. Same candidate sets, same order,
bit-exact; each candidate loaded once against 4 lanes.

## Measured — and why it lost

First measurement attempt was invalidated by the VM: the untouched 12k
serial-engine rows moved +40 % between runs, so a cross-run comparison was
worthless. Re-measured as an interleaved same-window A/B/B/A (stage-23
build in a git worktree vs the stage-24 tree), 24k spatial-hash force phase
(the only phase the change touches), with the verlet force phase — code
untouched by the change — as the noise control:

| leg | spatial force µs (touched) | verlet force µs (control) |
|---|---|---|
| A (23) | 182 | 204 |
| B (24) | 206 | 220 |
| B2 (24) | 226 | 245 |
| A2 (23) | 194 | 184 |

Stage-24 mean 216 µs vs stage-23 mean 188 µs on the touched phase: **~15 %
slower**, control noise ±15 %. At best it's a wash; it is certainly not a
win.

The mechanism, clear in hindsight: the *correction* kernel won 42 % because
most union candidates fail its distance mask — the scalar loop was
load-bound with mostly-idle arithmetic, exactly the slack clustering
reclaims. The *force* kernel has no cutoff: every candidate in range does
full-cost arithmetic already, so the loop is FLOP-bound. Clustering added
per-cluster setup (4 stencil computations + union + mask bounds) and made
edge lanes compute-then-discard union extras, while reclaiming no idle
lanes — there were none.

## Decision

**Rejected and reverted** (the tree keeps the scalar force gather; the
clustered correction gather from stage 23 stays). Recorded like stage 01's
`codegen-units=1` and stage 10's first SIMD attempt: a measured dead end is
part of the map. Width gains for the force pass would need a different
lever — e.g. an actual cutoff for the grid-path repulsion (a physics
change) or the GPU compute path (no GPU in this environment; `/dev/dri`
absent, software rasterizers pointless for a perf pass).
