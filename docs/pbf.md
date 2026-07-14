# Two fluid models: granular vs. PBF

The engine can run either of two particle-fluid models, chosen at launch:

```bash
cargo run --release -- --sim granular   # default: repulsion + contact projection
cargo run --release -- --sim pbf         # Position Based Fluids (Macklin & Müller 2013)
```

They are deliberately different physics, not two tunings of one model:

| | **Granular** (default) | **PBF** |
|---|---|---|
| What's enforced | non-penetration (pairs kept ≥ contact distance) + a local 1/r² repulsion | a *density* constraint: local density = rest density ρ0 |
| Behavior | stacks and slumps like sand / a pile of balls | pours, splashes, and sloshes like an incompressible liquid |
| Solver | Jacobi/Gauss-Seidel contact projection | the *same* Jacobi position-projection machinery, applied to the density constraint |

The point of the exercise (see the module header in `src/physics.rs`) was to go
from "piles like balls" to "actually behaves like water" by swapping the
constraint the position solver enforces — exactly the framing in
`docs/literature.md` §§8–9.

## What PBF does per substep

Macklin & Müller, *Position Based Fluids*, SIGGRAPH 2013 — algorithm 1:

1. **Predict** positions under gravity (symplectic Euler, scaled to match the
   granular Verlet integrator so both models fall identically).
2. **Neighbor search** on a uniform grid at cell size `h` (the smoothing
   radius). PBF wants a much wider stencil than the granular contact model —
   `h` ≈ 2.5 rest spacings here — so it keeps its own grid (`PbfGrid`) rather
   than reusing the tuned one-cell granular grid.
3. **Solver iterations** (Jacobi), each computing per particle:
   - density `ρᵢ = Σⱼ W_poly6(‖xᵢ−xⱼ‖, h)`
   - constraint `Cᵢ = ρᵢ/ρ0 − 1`
   - `λᵢ = −Cᵢ / (Σ‖∇C‖² + ε)`  (ε = CFM regularization)
   - `Δxᵢ = (1/ρ0) Σⱼ (λᵢ + λⱼ + s_corr) ∇W_spiky(xᵢ−xⱼ, h)`
   - `s_corr` is the artificial-pressure term (anti-clumping / surface tension).
4. **Velocity** `vᵢ = (xᵢ − x_prevᵢ)/Δt`.
5. **XSPH viscosity** (coherent flow) and **vorticity confinement** (re-inject
   the swirl the constraint solve damps out).

The density solve is a pure gather (each particle's correction is computed from
read-only neighbor data), so it parallelizes with rayon just like the granular
force gather.

## Tuning note: the unit system rescales every coefficient

The textbook PBF coefficients (ε≈600, s_corr k≈0.1) assume ρ0≈1000 in SI-ish
units. This codebase's kernels live in a very different unit system — ρ0 ≈ 0.028
and gradients ≈ 1e-3 — so the natural correction magnitudes are tiny and every
coefficient has to be re-derived. Two findings from the `--stats` diagnostic in
the render tool (`src/bin/render.rs`):

- **ε must stay ≪ Σ‖∇C‖² (≈7e-3 here).** The first cut used ε=20, which
  swamped the density term entirely: λ≈0, no correction, and the fluid clumped
  into pairs under gravity. Dropping ε to 1e-4 restored the density solve.
- **The density constraint alone does not prevent local clumping.** Measured
  ρ/ρ0 was 0.99 (incompressible!) while particles still paired up at ~2 px,
  because poly6 density is a smooth large-radius sum that many uneven layouts
  satisfy. Breaking that degeneracy is precisely the job of `s_corr`, and in
  this unit system it needs `k ≈ 6` (not 0.1) to hold the rest spacing. Above
  `k ≈ 8` the solve overshoots and destabilizes — a sharp cliff.

All PBF coefficients are runtime-tunable via `PbfParams` / `set_pbf_params` so
the sweep can be repeated (`cargo run --features render --bin render -- --stats`).

## Seeing the difference

`src/bin/render.rs` is a headless comparison renderer (the dev box has no
display). It runs three scenarios under both models from identical initial
states and composites them side by side into an animated GIF plus a still PNG:

```bash
cargo run --release --no-default-features --features render --bin render -- out_dir
```

- **dam_break** — a column released against a wall. PBF surges across the floor
  and runs up the far wall as a coherent wave; granular slumps flatter and
  sheds a ballistic spray.
- **drop_splash** — a blob dropped into a shallow pool. PBF throws a central
  rebound jet and ripples the whole pool; granular just dents and stays calm.
- **slosh_tank** — a liquid layer with gravity tilted left/right on a period.
  PBF sloshes up the wall and curls into a breaking wave; granular shifts as a
  heap and sheds loose grains.

The GIF/PNG encoders are hand-rolled (LZW GIF, stored-block PNG) to keep the
crate dependency-free; neither the default graphics build nor the headless CI
build compiles the `render` bin (it is gated behind the off-by-default `render`
feature).
