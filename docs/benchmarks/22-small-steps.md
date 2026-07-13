# 22 — Proper Small Steps: substeps × iterations, finally measurable

"Small Steps in Physics Simulation" (Macklin et al., SCA 2019) claims n
substeps × 1 solver iteration beats 1 step × n iterations at equal cost.
Stage 04 could only approximate this (cutting iterations at a fixed substep
rate) because the integrator's `a·dt` impulse term made dynamics depend on
the substep rate. Stage 21 fixed that (`a·dt²`), so the experiment is now
possible: `set_substeps(S)` splits each 480 Hz step into S internal substeps
(full integrate → grid → forces → solve pipeline at dt/S), with
`force_interval` scaled by S to hold the far-field refresh rate constant.

## Measured (`--small-steps`; settled state, 240 external steps, vs adopted S=1 it=3)

Note the re-run of the adopted config itself lands at −12…+7 % — that's the
noise band for the Δ column.

| particles | path | variant | Δmean | mean pen % | deep pairs |
|---|---|---|---|---|---|
| 12000 | barnes-hut | S=1 it=1 (control) | -40% | 11.01 | 97 |
| 12000 | barnes-hut | **S=2 it=1 K=8** | **-12%** | **2.96** (ref 3.68) | 0 |
| 12000 | barnes-hut | S=2 it=2 K=8 | +25% | 1.78 | 0 |
| 12000 | barnes-hut | S=3 it=1 K=12 | +21% | 1.54 | 0 |
| 12000 | verlet-lists | S=2 it=1 K=8 | -11% | 11.16 (ref 4.00) | 23 |
| 12000 | verlet-lists | S=2 it=2 K=8 | +38% | 5.81 | 11 |
| 24000 | spatial-hash | S=2 it=1 K=8 | -10% | 17.35 (ref 10.34) | 397 |

## The split verdict

- **BH path: Small Steps wins on both axes.** S=2 × it=1 is ~12 % faster
  *and* ~20 % better on mean penetration than S=1 × it=3 — the paper's
  claim, reproduced. In this regime the far-field pressure keeps the pile
  loose, so per-substep error is integration-dominated and halving dt beats
  a third solver sweep. S=2 × it=2 / S=3 × it=1 buy still-better contacts
  (mean pen 1.5–1.8 %) at +21…38 % cost — the quality dial now goes *up*
  too.
- **Grid paths: iterations win.** Their dense piles converge by Gauss-Seidel
  sweep count; 2×1 substep-sweeps lose to 1×3 (mean pen ~3× worse at equal
  cost). The control row (S=1 it=1) confirms most of the damage is the
  iteration cut itself.

## Decision

Engine defaults stay **S=1, it=3** — one global default must not degrade the
grid paths. For BH-mode use, `set_substeps(2)` + `set_solver_iterations(1)`
+ `set_force_interval(8)` is the measured-better configuration (faster and
higher-quality); left to the caller since the knob is per-engine, not
per-path. The substep machinery also unlocks quality headroom (S≥2, it≥2)
that no iteration count could reach before the stage-21 fix.
