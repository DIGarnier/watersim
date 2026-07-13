# 25 — One model: compact-support repulsion (and the tree goes away)

The model decision (docs/literature.md §9). The sim had a split personality:

- **Barnes-Hut mode** summed the 1/r² repulsion *globally* — an
  inverted-gravity n-body problem that needed an O(n log n) tree and was
  the slowest path at every size;
- **grid modes** truncated the same kernel at stencil geometry — an
  accidental cutoff that was *anisotropic and position-dependent*
  (guaranteed only to 7.5 px, reaching up to ~21 px diagonally), and the
  Verlet-list variant silently truncated at yet another radius.

Particle water models are local (SPH kernels, PBF neighborhoods, Clavet's
double-density — pressure is a local phenomenon), so the sim now commits to
one model: **1/r² repulsion with compact support**, cutoff at
`FORCE_CUTOFF = 2.5 ball radii = one grid cell`, smoothstep-tapered to zero
between the contact distance and the cutoff so the force is C¹ (no popping
as pairs cross it). Cutoff ≤ cell size makes the 3×3 stencil **provably
complete**; the new `--validate` bench mode confirms the engine's grid
gather equals the exact O(n²) sum — measured relative RMS difference:
exactly 0. The Verlet-list build range was fixed to cutoff + skin (5×5
build stencil), so list mode now computes the identical interaction set too.

## Removed (dead once the model is local)

- The entire Barnes-Hut machinery: arena quadtree, hot/cold node split,
  packed leaves, θ knob, tree phase stats, the [B] toggle — ~330 lines.
- `simd_physics.rs` (unused since before this series) and
  `resources/physics_compute.wgsl` (a GPU shader that was never wired up;
  no GPU exists in this environment).

## Measured (same-window A/B, HEAD worktree vs cutoff model, defaults)

Mean µs/step, grid paths (the only paths that exist on both sides):

| particles | verlet-lists | spatial-hash | max pen % (A → B) |
|---|---|---|---|
| 3000 | 342 → 268 | 324 → 257 | 0.0/0.1 → 0.0/0.0 |
| 6000 | 637 → 487 | 553 → 460 | 0.7/0.5 → 0.0/0.0 |
| 12000 | 1088 → 899 | 1067 → 905 | 0.1/1.1 → 0.0/0.0 |
| 24000 | 1497 → 1361 | 1573 → 1323 | 0.3/0.2 → 0.0/0.0 |

- **10–20 % faster** everywhere: the explicit cutoff prunes the 7.5–21 px
  pairs that previously did full kernel work, and the pruning more than
  pays for the taper arithmetic.
- **Contact quality improved to 0.0 % max pen at every size ≥ 3k** — the
  tapered local field is gentler at pile boundaries than the old
  anisotropic truncation.
- The *default* configuration (what the app runs) was previously the BH
  path; it is now the verlet path: **12k: 2079 → 899 µs (2.3×), 24k:
  3516 → 1361 µs (2.6×)** for default-mode users.
- 5 s soak at 12k/16k/24k: sane, deep pairs 1/14/99 (the 24k scenario fills
  ~90 % of the box; heavy-pile compression, no coincidences).

Behavior note: this intentionally changes what the old BH mode looked like —
there is no long-range spreading anymore, piles behave like liquid under
local pressure. That is the model decision, not a regression.
