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
  this unit system it needs `k` of order a few (not 0.1) to hold the rest
  spacing — `k ≈ 6` at this stage, later lowered to 3 once the settling pass
  below reduced the clumping by other means. Above `k ≈ 8` the solve overshoots
  and destabilizes — a sharp cliff.

All PBF coefficients are runtime-tunable via `PbfParams` / `set_pbf_params` so
the sweep can be repeated (`cargo run --features render --bin render -- --stats`).

## Making it settle like real water

Once the model was stable it still didn't *look* right: a settled pool kept
churning (mean particle speed ≈ 470 px/s that never decayed) with a few
particles flung out of it. Real water settles to still, so this was excess
energy the model wasn't dissipating. The `--tune` diagnostic (drop a block,
measure residual mean/max speed after it settles) drove two fixes:

- **XSPH viscosity was missing its `1/ρ0` volume weight.** The smoothing sum
  `Σ (vⱼ−vᵢ) W` is scaled by the raw kernel value (≈ρ0 ≈ 0.028 here), so
  without dividing by ρ0 the viscosity was ~35× too weak — raising `c` from
  0.08 to 0.4 barely changed anything. With the weight, `c` is a real dial and
  a full neighborhood of coherent motion actually damps.
- **ε=1e-4 was too *stiff*.** A near-zero CFM makes the density projection so
  hard it overshoots each substep and *rings* — the churn that no amount of
  viscosity removed. Softening ε to 2e-3 (still ≪ the ~7e-3 gradient sum, so
  ρ/ρ0 stays ≈ 1) let the solve come to rest.

Net effect on a settling block: residual mean-speed **20 → 2.5** (granular-pile
calm), max-speed **120 → 4** (no flung strays), 0 % clumping, ρ/ρ0 = 1.00 — and
the dynamic scenes still throw a splash jet and run a breaking wave up the wall,
because the density solve is untouched during active motion. The adopted
defaults are therefore `eps_cfm=2e-3, xsph_c=0.05, scorr_k=3, vorticity=0,
max_corr=0.12h` (vorticity confinement is off: it re-injects energy and fought
the settling; `xsph_c` was later trimmed 0.1 → 0.05 by the sloshing benchmark
below). Reproduce the sweep with
`cargo run --features render --bin render -- --tune`, and see a settled-pool
before/after with `--settle-still`.

The earlier `k ≈ 6` / `ε = 1e-4` values (before this pass) held the rest density
but produced the churn; they survive only in `--settle-still`'s left panel for
comparison.

## Stability: drive a fixed timestep

Both integrators (granular Størmer–Verlet and PBF's predict/velocity step)
assume a **constant** dt. The headless render tool always steps at a fixed
`dt = PHYS_TIME_STEP`, which is why it is rock-stable. The live app originally
fed the raw, variable wall-clock dt straight in, with adaptive-dt on top — two
distinct failures:

- **Granular "breathing".** With Verlet, the velocity is encoded implicitly as
  `x − x_prev`. Change dt between steps and that same displacement is
  reinterpreted as a different velocity, injecting or removing energy. The
  adaptive-dt controller then chased velocity by oscillating dt, producing the
  visible limit cycle where particles float, then drop.
- **PBF exploding when sparse.** A far-under-dense fluid (few particles,
  C ≈ −0.8) drives `λ = −C/(Σ‖∇C‖²+ε)` huge because ε is tiny, and a large
  variable dt turns that into a blow-up.

The fix (`src/main.rs`): a standard fixed-timestep accumulator advances the sim
in `PHYS_TIME_STEP` chunks to track real time, with adaptive-dt off
(`set_adaptive_dt(false)`). The `--stats` sweep confirms that under a fixed dt
even a 5×5 blob stays sane. Two cheap safety nets remain in the solver: `λ` is
capped (`PbfParams::lambda_max`, default 30 — keeps ρ/ρ0 ≈ 0.99 while bounding
the sparse pathology) and per-substep displacement is clamped to half a
smoothing radius.

## Debugging instability

The engine now exposes coarse aggregate diagnostics in `PerformanceStats`
(`mean_speed`, `max_speed`, and for PBF `pbf_density_ratio` = mean ρ/ρ0). The
live HUD shows them, and setting `WATERSIM_DEBUG=1` logs them each second:

```
WATERSIM_DEBUG=1 cargo run --release -- --sim pbf
# [Pbf] n=1240 mean_speed=3.1 max_speed=48.0 rho/rho0=0.99 solve=820us
```

A density ratio climbing well above 1, or a max_speed running away, is the
signature of a blow-up — far easier to see than staring at particles.

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

## Validation against literature benchmarks

The standard water-sim benchmark suite has *quantitative* expectations, not just
visual ones, so `--validate-water` checks the PBF defaults against theory
(effective gravity `a = g/PHYS_TIME_STEP` is confirmed by a free-fall probe):

- **Dam break** (Koshizuka & Oka 1996; Martin & Moyce 1952). A width-`a`,
  height-`2a` column collapses on a dry bed. The measured surge-front speed is
  **0.56×** the analytical Ritter dry-bed tip speed `2√(gH)` — right in the band
  expected for a finite column (Ritter assumes a semi-infinite reservoir, so a
  collapsing column runs slower), the front reaches the far wall, and volume is
  conserved to a few %.
- **Sloshing** (SPHERIC-style). A still layer is kicked and left to oscillate;
  the free-oscillation period is **0.95×** linear potential theory
  `T = 2π/√(g·k·tanh(k·h))`, `k = π/L` — within ~5 %.
- **Hydrostatic tank.** A resting block stays still (residual speed ≈ 1),
  keeps a flat free surface (RMS ≈ 3 px), and conserves volume.

Two honest findings came out of building these, and drove real fixes:

- **PBF's raw λ is not a clean pressure probe.** The idea was to check that
  pressure ∝ depth (hydrostatic), reading PBF's Lagrange multiplier as pressure.
  It doesn't work here: the λ field is a U-shape dominated by **SPH particle
  deficiency** at the free surface *and* the floor (both have missing
  neighbors), which swamps the weak bulk gradient — a well-documented SPH
  artifact. So the hydrostatic tank is validated on its observable behavior
  (rest / flatness / volume), which is what "well-behaved" means for it.
- **Sloshing exposed over-damping.** The frequency was right all along, but the
  amplitude decayed ×0.27 per half-period — a slosh died in one cycle where real
  water swings many times. The `--damp-sweep` diagnostic traced it to `xsph_c`:
  the settling pass had cranked it to 0.1, which over-damped free oscillation
  *without even improving the settle*. Trimming to 0.05 roughly doubles slosh
  persistence (decay ×0.40), keeps the period at 0.95×, and settles just as calm
  (residual ≈ 0.6). The residual damping below that is the density solve itself —
  PBF is inherently dissipative, an accepted trade for unconditional stability.

Reproduce with `--validate-water` (the benchmark suite) and `--damp-sweep` (the
viscosity/dissipation trade-off table).
