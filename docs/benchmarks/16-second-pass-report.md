# 16 — Second optimization pass: final report (2026-07-13)

This pass started from the merged stage-11 code and asked: with the data
structures already tight, what do the *algorithms* still leave on the table
for a realtime-vs-accuracy compromise? Method as before — same harness, fresh
same-day baseline (stage 12), one change at a time, every accuracy trade
measured, not assumed. New in this pass: the harness measures **quality**
alongside time (force error vs exact summation, contact penetration
statistics, end-state density drift with a chaos-floor control).

## What changed

| stage | change | knob (default) | headline |
|---|---|---|---|
| 13 | **BH force-sign fix** + measured θ | `set_bh_theta` (0.9) | sign bug since BH's introduction: the BH toggle simulated *attraction*, every exact path repulsion; θ=0.9 ≈ 2.2× cheaper traversal for a measured 6 % RMS force error |
| 14 | **Multiple time stepping** (r-RESPA): far-field force refreshed every K substeps, contacts every substep | `set_force_interval` (4) | −55 %…−66 % mean step on the BH path alone; −17 %…−28 % on grid paths; contact quality unchanged |
| 15 | **Solver iterations × SOR** measured on both engines | `set_solver_iterations` (3), `set_solver_omega` (1.0) | 4→3 iterations ≈ free on quality (−18/−32 % solver time); ω≥1.5 proven unsafe on the packed Jacobi engine |

The three defaults together are the measured "combo-safe" point of the
stage-14/15 sweeps: −63 %…−69 % mean step time on the BH path, −34 %…−42 %
on the grid paths, with deep-pair counts and mean penetration at reference
levels. The faster/mushier "combo-fast" point (it=2, ω=1.25; another ~10 %)
is documented in stage 15 and reachable at runtime.

## Baseline → final, mean µs/step (same day, same VM, defaults)

| particles | barnes-hut | verlet-lists | spatial-hash |
|---|---|---|---|
| 1000 | 506 → **205** (2.5×) | 99 → **96** | 102 → **92** |
| 3000 | 1807 → **447** (4.0×) | 303 → **258** | 301 → **304** |
| 6000 | 3678 → **884** (4.2×) | 615 → **441** (1.4×) | 666 → **465** (1.4×) |
| 12000 | 6697 → **1737** (3.9×) | 1144 → **805** (1.4×) | 1134 → **798** (1.4×) |
| 24000 | 14043 → **4234** (3.3×) | 3600 → **3037** | 3136 → **2992** |

- **The Barnes-Hut path is real-time (480 Hz median) through 12 000
  particles** — it previously fell out of budget at 6 000. At 24k it's
  3.3× faster but still over budget (and the 24k scenario itself over-fills
  the box; see stage 12).
- Grid paths were already real-time to 12k; they gain another ~1.4× where
  the solver dominates. At 24k the mean is pulled by refresh-step spikes;
  amortized per 60 Hz frame the paths are borderline rather than clearly out.
- The BH sign fix is also a *quality* win: max pen at 1k dropped 80.9 % →
  17.8 % because BH no longer squeezes the pile into the contact solver.

## Quality at the final defaults

- Force field: 5.8–6.1 % relative RMS error vs exact summation (θ=0.9,
  measured at 3k and 12k) — against a reference that was, until this pass,
  pointing the wrong way on the BH path entirely.
- Contacts (240-step sweep vs stage-11 reference): deep pairs 0 → 0 (BH,
  16k packed), 20 → 21 (12k Gauss-Seidel); mean penetration +0.3–1.8 pp.
- Soak (2 400 steps = 5 s at defaults): no NaN, no escapes, deep-pair counts
  and mean penetration flat vs the short runs — no slow degradation mode.
- Density drift vs reference sits at each configuration's
  trajectory-decorrelation plateau (chaos-floor control rows in the sweep),
  i.e. the end states differ the way any two runs of a chaotic system
  differ, with no systematic bias the contact metrics can see.

## Follow-ups (unchanged from stage 11, minus what this pass took)

- The solver now dominates every path again (stage-16 phase profile) —
  GROMACS-style cluster pair lists remain the SIMD width lever, and the
  16k `PAR_MIN_PARTICLES` threshold deserves a re-sweep now that the
  packed engine's per-step cost profile changed.
- Proper Small Steps (substep-rate ↑, iterations → 1) still wants the
  integrator's `a·dt` → `a·dt²` fix first (stage 04).
- The 24k benchmark scenario over-fills the box and manufactures coincident
  pairs at the top wall (stage 12) — worth redesigning before trusting any
  absolute quality number at that size.
- Coincident pairs are invisible to the solver's degenerate-pair mask
  forever; a deterministic tie-break separation direction would remove that
  absorbing state.
