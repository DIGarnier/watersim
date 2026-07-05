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
