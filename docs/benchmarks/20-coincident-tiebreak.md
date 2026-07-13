# 20 — Un-gluing coincident pairs

Follow-up from stage 15/16: an exactly-coincident pair (distance < 1e-6 px)
was invisible to every solver kernel — the degenerate-pair mask that guards
against dividing by zero also meant *no iteration could ever separate the
pair again*. Coincidence was an absorbing state; the stage-12 scenario
manufactured thousands of them at the top wall, and unsafe solver settings
(ω ≥ 1.5) produced them organically.

Two changes:

1. **Deterministic tie-break in the serial Gauss-Seidel engine**
   (`project_pair`): a coincident pair has no collision axis, so one is
   chosen from the pair's indices (X or Y by parity) and the pair is
   projected apart as a fully-overlapped contact. Next iteration they have a
   real axis and the normal projection takes over. The packed Jacobi gather
   kernel keeps its mask (a per-lane tie-break would cost the hot loop; both
   sides of a gathered pair would also need opposite directions, which a
   symmetric gather can't express cheaply) — instead, coincidence is now
   prevented at its one organic source:
2. **Anti-stacking salt in the wall clamp**: every particle pushed past the
   same wall corner in one substep used to clamp onto the *identical*
   (x, y). A deterministic per-particle-id offset of 0–0.15 µm breaks exact
   ties without visible effect.

Regression test: `physics::tests::coincident_pair_separates` — two particles
spawned on the same point must reach contact distance within 20 substeps
(fails on the pre-stage-20 solver, which leaves them glued forever).

Performance: the tie-break lives in the already-rare degenerate branch and
the salt is one multiply-add per wall contact — the full-run A/B (see stage
21's table, which includes both changes) is flat within the noise band.
