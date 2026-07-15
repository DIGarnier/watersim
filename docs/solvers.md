# Solvers — the strategy pattern and the method zoo

This sim can run more than one particle-fluid method. Each is a **strategy**:
a numerical model selected once at startup (`--sim <name>`) and kept for the
whole run. This note documents the architecture and evaluates the methods —
the two we have, and the ones we are adding.

## Architecture: one boxed strategy, chosen once

`src/physics/` is organized around a single trait:

```rust
pub trait FluidSolver: Send {
    fn name(&self) -> &'static str;
    fn substep(&mut self, dt: f32, gravity: Vec2, share: &mut ShareData, c_opos: &mut Vec<Vec2>);
    // + no-op-defaulted knob setters (add_scale, set_pbf_params, …)
}
```

`Physics` is a thin **coordinator**. It owns only the state shared across
*every* method — the particle positions/colors (`ShareData`), the
Størmer–Verlet previous-position array (`c_opos`, which also encodes velocity),
gravity, the substep count, and the adaptive-timestep controller — and holds
the active method as one `Box<dyn FluidSolver>`. Each substep it makes exactly
one dynamic call into the boxed strategy.

Two design constraints shaped this (both from the original request):

- **No bundling.** Each method keeps *its own* scratch inside its own struct
  (the granular CSR grid + packed SoA + Verlet lists; PBF's h-grid + velocity
  and λ buffers; DFSPH's α/stiffness fields; MPM's background grid + per-particle
  deformation gradient). Nothing forces one method's data into another's.
- **Few indirections.** Because the strategy is chosen once and never swapped
  per step, the single `dyn` call per substep is free relative to the
  O(n·neighbors) work it wraps — no per-particle virtual dispatch, no enum
  match in the hot loop. The knob setters that only one method understands
  (force scale, PBF coefficients) are defaulted to no-ops on the trait so the
  coordinator can forward them blindly.

Adding a method = one new file implementing `FluidSolver`, one `Strategy`
variant, one arm in `Strategy::make_solver`. The renderer, cannon, HUD, event
loop, and benchmark harness are untouched — they only ever see `Physics`.

## What we have

### Granular — PBD non-penetration + local repulsion  (`granular.rs`)

Short-range 1/r² repulsion with compact support (cutoff = one grid cell) plus a
hard non-penetration contact projection, integrated with Størmer–Verlet at
480 Hz. It **stacks and slumps like a pile of balls / sand**, not a liquid.
This is the historical engine and carries every optimization in
`docs/benchmarks/`: counting-sort CSR grid, packed SIMD Jacobi contact solver,
Verlet neighbor lists, RESPA multiple-time-stepping, SFC reordering.

- Family: Position-Based Dynamics (geometric projection).
- Character: dissipative, rock-stable, granular. Not incompressible.

### PBF — Position Based Fluids  (`pbf.rs`)

Macklin & Müller (SIGGRAPH 2013). The same Jacobi position-projection enforces
a **density** constraint instead of non-penetration, so particles pour, splash,
and slosh as an incompressible liquid. Self-contained: its own h-sized grid,
velocities, XSPH viscosity, and (optional) vorticity confinement.

- Family: Position-Based Dynamics (density constraint).
- Character: incompressible-*ish* but **soft and inherently dissipative** — the
  density constraint is satisfied iteratively/geometrically, which bleeds
  energy. Great stability, muted liveliness. See `docs/pbf.md`.

Both existing methods live in the **same corner** of the design space: PBD,
stable, energy-losing. That is exactly the gap the new methods target.

## The candidates (evaluated for *this* sim)

This engine is 2D, real-time, and renders every particle as a circle, so the
methods that fit are **Lagrangian** (particles carry position). The bar for
"worth adding" is *different observable character*, not just a different
acronym.

| Method | Family | Would add | Fit here |
|---|---|---|---|
| **DFSPH** | Pressure-projection SPH | Crisp, low-dissipation, splashy incompressible water; density error <0.1%; big timesteps | **Excellent** — reuses SPH h-grid, particle-native. **Adopted.** |
| **MLS-MPM** | Hybrid grid+particle (APIC) | Elastic jelly, snow, sand-with-friction — a whole different constitutive world | **Excellent** — particles carry state, grid is scratch. **Adopted.** |
| IISPH | Pressure-projection SPH | Implicit pressure-Poisson; very incompressible | Good, but *same family* as DFSPH — largely redundant once DFSPH is in. |
| VSLSPH | — | — | Not a standard, citable method; skipped pending a concrete reference. |
| Cumulant LBM | Lattice Boltzmann (Eulerian) | State-of-the-art collision operator | **Poor fit** — a field on a fixed lattice, no Lagrangian particles. Rendering it as moving circles needs free-surface VOF + tracers bolted on: a separate visualization, not a drop-in strategy. Deferred. |

### Decision

Add two methods that each open a *new* corner of the design space:

1. **DFSPH** (Bender & Koschier 2015/17) — the pressure-SPH answer to "PBF but
   crisp." Two solves per step (constant-density **and** divergence-free) give
   true incompressibility with low numerical dissipation, so it stays lively:
   sharp splashes, persistent sloshing. The direct contrast to PBF's mushiness.
2. **MLS-MPM** (Hu et al. 2018) — a hybrid Eulerian–Lagrangian method. Particles
   carry mass, velocity (APIC affine state), and a deformation gradient; a
   background grid does the momentum solve. Swapping the constitutive model
   turns the *same* solver from fluid to elastic jelly to granular sand. Nothing
   else here can do elastoplastic solids.

Coverage after these land: PBD (Granular, PBF), pressure-SPH (DFSPH), and hybrid
MPM (MLS-MPM) — four methods spanning three distinct families.

## References

- **DFSPH** — Bender & Koschier, *Divergence-Free Smoothed Particle
  Hydrodynamics*, SCA 2015; *Divergence-Free SPH for Incompressible and Viscous
  Fluids*, IEEE TVCG 2017.
- **IISPH** — Ihmsen, Cornelis, Solenthaler, Horvath & Teschner, *Implicit
  Incompressible SPH*, IEEE TVCG 2014.
- **MLS-MPM** — Hu, Fang, Ge, Qu, Zhu, Pradhana, Jiang, *A Moving Least Squares
  Material Point Method with Displacement Discontinuity and Two-Way Rigid Body
  Coupling*, SIGGRAPH 2018. Background: Stomakhin et al., *A Material Point
  Method for Snow Simulation*, SIGGRAPH 2013; Jiang et al., *The Affine
  Particle-In-Cell Method*, SIGGRAPH 2015.
- **LBM (cumulant)** — Geier, Schönherr, Pasquali & Krafczyk, *The cumulant
  lattice Boltzmann equation in three dimensions*, 2015.
