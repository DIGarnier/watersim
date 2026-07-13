# Literature notes — optimizing this n-body / particle sim

Survey done 2026-07-01 to ground the optimization work in current practice.
Each technique below maps to a concrete change in this codebase; benchmarks for
each live in `docs/benchmarks/`.

## 1. Neighbor search: counting-sort cell lists (CSR layout)

The standard for fixed-radius neighbor search in particle codes (MD, SPH, PBD) is a
uniform grid stored as one flat index array plus per-cell start offsets (CSR), built
with a counting sort in O(n) — not per-cell dynamic arrays.

- S. Green, *Particle Simulation using CUDA*, NVIDIA whitepaper (2010) — grid via sort,
  contiguous cell contents.
- R. Hoetzlein, *Fast Fixed-Radius Nearest Neighbors* (GTC 2014) — counting sort +
  prefix sums beats radix-sort grids on modern hardware.
- [Fast and exact fixed-radius neighbor search based on sorting (arXiv:2212.07679)](https://arxiv.org/pdf/2212.07679)
- [GPU-Native Compressed Neighbor Lists with a Space-Filling-Curve Data Layout (arXiv:2602.19873)](https://arxiv.org/pdf/2602.19873) —
  2026; confirms SFC-ordered CSR neighbor structures are still state of the art.

**Applies here:** `Physics::table` is `Vec<Vec<usize>>` (18 000 heap vectors), rebuilt
and fully swept — including empty cells — 8× per step by the collision solver, with
neighbor contents copied into a scratch buffer. Replace with CSR grid built once per
solver pass, iterate only over occupied cells, use half-neighborhood traversal with
Newton's third law.

## 2. Substeps beat solver iterations ("Small Steps")

- [Macklin, Storey, Lu, Terdiman, Chentanez, Jeschke, Müller — *Small Steps in Physics
  Simulation*, SCA 2019](https://mmacklin.com/smallsteps.pdf): for position-based
  solvers, n substeps × 1 constraint iteration is more accurate *and* stiffer than
  1 step × n iterations at the same cost. See also the
  [ACM version](https://dl.acm.org/doi/10.1145/3309486.3340247).

**Applies here:** the sim already substeps at 480 Hz (8× a 60 Hz frame), yet *also*
runs 8 constraint-projection iterations inside every substep — effectively 64
iterations per visual frame. Per Small Steps, the high substep rate is what buys
stability; the inner iteration count can be cut drastically.

## 3. Barnes-Hut on CPU: flat, allocation-free trees

Pointer-chasing trees with per-node heap allocation are the known anti-pattern;
production BH codes use contiguous node arrays (arena/linear quadtrees, often
Morton-ordered) and non-recursive traversal.

- Burtscher & Pingali, *An Efficient CUDA Implementation of the Tree-Based Barnes Hut
  n-Body Algorithm* (GPU Computing Gems, 2011) — [PDF](https://iss.oden.utexas.edu/Publications/Papers/burtscher11.pdf)
- [Accelerating Barnes-Hut t-SNE by Efficient Parallelization (arXiv:2212.11506)](https://arxiv.org/pdf/2212.11506) —
  Morton-code quadtree build, SIMD-friendly layout, big single-thread wins.
- [Stochastic Barnes-Hut Approximation for Fast Summation on the GPU (arXiv:2506.02219)](https://arxiv.org/pdf/2506.02219) — 2025, current research direction.

**Applies here:** `QuadNode` boxes 4 children per subdivision, rebuilt every step,
recursive force traversal. Replace with an index-based arena reused across frames and
an explicit-stack traversal.

## 4. Data layout: SoA, SIMD, and spatial reordering

- [GROMACS heterogeneous parallelization (arXiv:2006.09167)](https://arxiv.org/pdf/2006.09167) —
  SoA layouts and cluster pair lists for SIMD.
- Space-filling-curve particle reordering (HOOMD-blue's SFCPACK; Hilbert-sorted
  particles reported ~2× over random order) — cache locality for neighbor gathers.

**Applies here:** force kernels recompute `length()` then `normalize()` (two sqrt and
a division chain per pair); rewrite with one inverse-sqrt. Particle arrays are AoS
`Vec2`, which is acceptable for 2D (8 bytes/particle ≈ SoA); the bigger win is
iterating pairs in cell order (the CSR grid already gives near-sorted access).
*Status: applied — hardware rsqrt kernels (benchmark 08), SFCPACK-style periodic
array reordering (benchmark 07), and a hot/cold node split for the BH tree
(benchmark 09). Lazy displacement-triggered Verlet rebuilds landed in benchmark 06.*

## 5. Multithreading

Classic domain decomposition: per-particle force gather parallelizes trivially if each
thread writes only its own particle's force (skip Newton's-third-law sharing across
threads — recompute instead of sync). GROMACS/LAMMPS both take this trade on wide
machines.

**Applies here:** physics runs on one thread; the machine has 4 cores. Use `rayon`
for the force gather and integration loops.

---

Sections below added 2026-07-13 for the second optimization pass
(docs/benchmarks/12+), focused on *controlled* accuracy/speed trades — the
realtime-vs-accuracy compromise knobs used across water/particle simulation
practice.

## 6. Barnes-Hut opening angle: error is a dial, not a constant

The multipole acceptance criterion s/d < θ makes θ the canonical
accuracy/cost dial of treecodes: monopole-only force error grows ~θ² for
near-uniform distributions, and traversal cost falls roughly as 1/θ².
Production astrophysics codes run θ = 0.7–1.0 and report the *measured* rms
force error rather than assuming one.

- Barnes & Hut, *A hierarchical O(N log N) force-calculation algorithm*, Nature 324 (1986).
- Wadsley, Stadel & Quinn, *Gasoline: an adaptable implementation of TreeSPH*
  ([arXiv:astro-ph/0303521](https://arxiv.org/pdf/astro-ph/0303521)) — θ = 0.7 typical,
  rms relative force error ~0.4% reported for clustered states.
- Interactive error/cost exploration: [The Barnes-Hut Approximation (Heer)](https://jheer.github.io/barnes-hut/) —
  θ ≈ 1 gives ~1% errors for gravity benchmarks.

**Applies here:** θ was hard-coded at 0.5 with no error measurement. The bench
now has a `--force-error` mode comparing tree forces against the exact O(n²)
sum; docs/benchmarks/13-bh-theta.md has the measured curve for this sim
(error ≈ 1.3% at θ=0.5, 6% at θ=0.9; traversal 2.2× faster at 0.9). Also
uncovered by that instrument: the BH traversal had the force *sign inverted*
relative to every exact path since its introduction (rel. RMS "error" of 2.0
at all θ = a flipped vector). Fixed; BH now approximates the same repulsive
system the exact paths compute.

## 7. Multiple time stepping (r-RESPA): slow forces on a slow clock

For systems with a stiffness separation — fast local interactions, smooth
long-range field — evaluating the long-range force every n substeps and
holding it in between is the classic MD answer, formally derived from a
Trotter factorization (reversible, stable):

- Tuckerman, Berne & Martyna, *Reversible multiple time scale molecular
  dynamics*, J. Chem. Phys. 97, 1990 (1992) —
  [PDF](https://www.columbia.edu/cu/chemistry/groups/berne/papers/jcp_97_1990_1992.pdf):
  "the short range force is computed after each time step and the long range
  force is computed every n time steps", with "considerable speedups … with
  no loss of accuracy".
- Games/graphics land routinely does the same thing as "force caching" /
  staggered updates; Small Steps (Macklin et al. 2019, §2 above) is the
  complementary result that the *constraint* part wants the fastest clock.

**Applies here:** the sim substeps at 480 Hz; per substep a particle moves a
tiny fraction of a grid cell, so the smooth 1/r² repulsion field barely
changes between substeps — yet it was recomputed (tree build + traversal, the
dominant cost on the BH path) every substep. Now the far-field pass refreshes
every K substeps (`FORCE_INTERVAL`, default 4) and is held piecewise-constant;
the stiff contact projection still runs at 480 Hz, exactly the RESPA split.
Measured in docs/benchmarks/14-mts-farfield.md.

## 8. Constraint solver: over-relaxation instead of iterations

Position-based solvers converge geometrically, so the per-pair correction
factor and the iteration count are interchangeable currencies. PBD/PBF
practice exposes an SOR factor ω (1 ≤ ω ≤ 2) on Jacobi/Gauss-Seidel constraint
projection, with constraint averaging keeping Jacobi stable:

- Macklin, Müller, Chentanez & Kim, *Unified Particle Physics for Real-Time
  Applications*, SIGGRAPH 2014 —
  [preprint](https://mmacklin.com/uppfrta_preprint.pdf): global SOR parameter,
  1 ≤ ω ≤ 2 recommended.
- Macklin & Müller, *Position Based Fluids*, SIGGRAPH 2013 — parallel Jacobi
  density-constraint solver for realtime water, the framing this sim's
  contact solver already follows at scale (stage 10).

**Applies here:** the historical solver ran 4 iterations of an under-relaxed
projection (0.75 stiffness, half per particle → effective per-pair factor
0.375). The factor is now 0.375·ω with ω runtime-tunable; the measured
(iterations × ω) surface (docs/benchmarks/15-solver-relaxation.md) shows
where fewer, stronger iterations buy solver time at equal penetration
quality — the solver dominates the grid paths, so this converts directly to
step time.

## 9. Compact support: water forces are local (the stage-25 model decision)

Every particle water method gives its interaction kernels compact support —
pressure in a liquid is a local phenomenon:

- Müller, Charypar & Gross, *Particle-Based Fluid Simulation for Interactive
  Applications*, SCA 2003 — SPH smoothing kernels with support radius h,
  identically zero beyond it.
- Macklin & Müller, *Position Based Fluids*, SIGGRAPH 2013 — density
  constraints over an h-neighborhood, found with a uniform grid of cell
  size h.
- Clavet, Beaudoin & Poulin, *Particle-based Viscoelastic Fluid Simulation*,
  SCA 2005 — the classic realtime 2D water recipe: short-range
  double-density relaxation, nothing long-range.

**Applies here:** the sim historically ran a *split* model — the Barnes-Hut
path summed 1/r² repulsion globally (an inverted-gravity n-body, needing an
O(n log n) tree) while the grid paths truncated the same kernel at stencil
geometry (an accidental, anisotropic cutoff). Stage 25 commits to the local
model: explicit cutoff at 2.5 ball radii = one grid cell (making the 3×3
stencil provably exact — validated against the O(n²) sum by the bench's
`--validate` mode), with a smoothstep taper from the contact distance so
the force is C¹ at the cutoff (no popping). The Barnes-Hut machinery,
having no system left to approximate, was removed.

## Sources

- https://arxiv.org/pdf/2212.07679
- https://arxiv.org/pdf/2602.19873
- https://mmacklin.com/smallsteps.pdf
- https://dl.acm.org/doi/10.1145/3309486.3340247
- https://iss.oden.utexas.edu/Publications/Papers/burtscher11.pdf
- https://arxiv.org/pdf/2212.11506
- https://arxiv.org/pdf/2506.02219
- https://arxiv.org/pdf/2006.09167
- https://developer.download.nvidia.com/assets/cuda/files/particles.pdf (Green 2010)
- https://on-demand.gputechconf.com/gtc/2014/presentations/S4117-fast-fixed-radius-nearest-neighbor-gpu.pdf (Hoetzlein 2014)
