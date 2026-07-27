# Task List

> **Document hierarchy:** Tasks are organized by the level of the design
> chain they affect. Each item should cite the relevant document section.

---

## VISION

<!-- Tasks related to goals and principles. -->

---

## ARCHITECTURE

<!-- Tasks related to layout, modules, build. -->

---

## DESIGN

<!-- Tasks related to algorithms and data structures, mathematical
foundations, interaction rules. -->

`DESIGN.md` is complete: all fourteen sections are drafted, and its
Contents table marks every one "written". A `/refine` pass reconciled it
with ARCHITECTURE (see below). This list now records only the milestone
work that follows.

**All DESIGN sections are written and committed**, §1–§14, including the
§6.1 integrator-signature reconciliation that the PSEUDOCODE pass produced.

**`/refine` outcomes (this pass):**

- [x] Finding 1 — retention buffer had two homes. Fixed DESIGN §12 to
      name `dynamics/trajectory.py` as the buffer (ARCH §3.3), with the
      sink boundary reserved for recording/spill; corrected the illegal
      "buffer forwards to hdf5_sink" (dynamics/ may not import sinks/).
- [x] Finding 2 — scenario format. Chose **TOML**; DESIGN §11.7 now names
      it, and ARCHITECTURE §9.1/§9.2 align (`.toml`, `tomli-w` writer,
      `tomllib`/`tomli` reader).
- [x] Finding 3 — the `ui/controls.py` layer had no DESIGN prose. Added
      DESIGN §14 (interaction and controls), per the chain division: ARCH
      names the tool, DESIGN gives the prose, PSEUDOCODE the algorithm.

**Milestone — DESIGN closed, PSEUDOCODE opened and completed:**

- [x] Commit §14 and the refine edits.
- [x] Tag `v0.3-design` (ARCHITECTURE §10) — created; the design baseline
      is marked.
- [x] `PSEUDOCODE.md` drafted in full — see the PSEUDOCODE section below.

**Open items carried forward:**

- The §8.4 elliptic solution is numerically verified
  (`dev/spikes/free_top_elliptic.py`, passing). PSEUDOCODE §9 reproduces
  the forms it certifies; the code for `analytic_solutions.py` must do the
  same when written.
- DESIGN forward-obligation still open for the code level:
  `numerical_inertia.py` must reproduce §3.2 to a stated tolerance (§3.7).
  (The TOML key layout §11.7 deferred to PSEUDOCODE is discharged in §12.3.)

---

## PSEUDOCODE

<!-- Tasks related to algorithm specifications. -->

`PSEUDOCODE.md` is **complete**: all fifteen sections are drafted, and its
Contents table marks every one "written". It transcribes the algorithmic
subset of DESIGN into language-agnostic form. The record below is kept for
reference; the next milestone is the source in `src/`.

- [x] §1 The simulation loop (DESIGN §4.2, §6, §7, §12, §14; ARCH §6).
- [x] §2 Orientation mathematics (DESIGN §1.3, §2.2, §2.4, §2.6) — the
      quaternion foundation §1 and everything else call into.
- [x] §3 State and derived quantities (DESIGN §2.1, §2.5) — the state
      record and the momentum/energy read-outs, with the torque-free
      invariants the monitor and Poinsot lean on.
- [x] §4 Inertia and body construction (DESIGN §3) — the Body record,
      the three physical assertions, the closed-form/provider seam, the
      diagonalization with handedness and degeneracy fixes, the pivot
      shift, classification, and the build_body constructor.
- [x] §5 Equations of motion (DESIGN §4) — Euler's equations, the pure
      state_derivative, the additive body-frame torque total, the
      intermediate-axis growth rate with its interface guard, and the
      conservation rate laws the monitor leans on.
- [x] §6 Torque models (DESIGN §5) — the pure torque_body interface,
      torque-free as the empty list, the gravity-through-a-pivot and
      viscous-damping models, internal dissipation as a non-torque state
      modifier, and the fixed-order composition rule.
- [x] §7 Integrators (DESIGN §6) — the selectable-strategy interface
      with the state arithmetic, fixed-step RK4, the post-step quaternion
      renormalization, the two error regimes, and the structure-preserving
      (symplectic) path designed for the long regime.
- [x] §8 Conservation monitor (DESIGN §7) — the live balance-not-constancy
      residual against the §5.5 rate laws, the fixed-scale/per-unit-time
      reporting with the magnitude/direction split, the secular-trend
      tracker, the report-never-correct discipline, and the Poinsot
      identity offered as a test oracle only.
- [x] §9 Analytic solutions (DESIGN §8) — steady principal rotation, the
      symmetric-top precession rates, the torque-free asymmetric top in
      Jacobi elliptic functions (both branches, reproducing exactly the
      forms `free_top_elliptic.py` certifies), and the heavy-top
      steady-precession quadratic with its existence threshold.
- [x] §10 Poinsot geometry (DESIGN §9) — the momental ellipsoid and
      contact point, the invariable plane, and the polhode/herpolhode
      pair (analytic where §9 supplies it, numerical quadric intersection
      otherwise), with the separatrix stability story.
- [x] §11 Reference frames (DESIGN §10) — the one mapping applied both
      ways (express_in_space/express_in_body), the frame-view choice of
      what holds still, the frame-free invariants, L in both frames with
      its polhode-shadowing companion curve, and the Poinsot pairing.
- [x] §12 Scenario load and save (DESIGN §11) — the two-zone plain-data
      scenario, the full concrete TOML key layout (§11.7's deferred
      deliverable, now given), the units/precision/versioning constraints,
      and load/save with the resolved-vs-recomputed body consistency check.
- [x] §13 Trajectory retention (DESIGN §12) — retention as a read-only
      cache over the recomputable Markovian ground truth: the bounded ring
      buffer, keyframe-plus-re-integration for long runs, the sink spill
      for whole-history preservation, and why the limit is recorded.
- [x] §14 Scene description (DESIGN §13) — the quantity/primitive/pixel
      pipeline, the named-drawable inventory assembled by build_scene, the
      role-to-encoding palette with the redundant-encoding rule, the frame
      coding, the labeled ellipsoid-scale choice, and the telemetry overlay
      with the batch tier's HDF5/XDMF path.
- [x] §15 Controls and time (DESIGN §14) — the scenario-editing vs time
      control split, read_controls and the substep-count map (the §1.2
      hooks), editing as a new scenario, the read-only replay scrubber,
      and the labeled scaling controls. **Completes PSEUDOCODE.md.**

**Resolved (DESIGN §6.1 signature).** DESIGN §6.1 now reads
`advance(state, time, dt, derivative_function)`, with a clause explaining
that a multi-stage method samples the derivative at `time + c*dt` and a
time-dependent torque (DESIGN §5.7) reads that absolute time. PSEUDOCODE
§1.1 and §7.1 now describe their signature as *matching* DESIGN rather
than refining it. The three levels agree; nothing further pending.

**`/refine` pass over the completed chain.** `src/` has no rigid-body code
yet (PSEUDOCODE → Code not yet applicable); VISION → ARCHITECTURE and
ARCHITECTURE → DESIGN consistent (the §6.1 edit touches nothing ARCHITECTURE
pins). Three PSEUDOCODE-internal gaps were fixed:

- [x] Finding 1 — §7.1 `select_integrator` now dispatches on the three
      integrator names of §12.3 (`rk4`, `implicit_midpoint`, `splitting`)
      rather than a binary SYMPLECTIC check.
- [x] Finding 3 — §12.3's presentation zone gained an `ellipsoid_scale`
      key and a `scale_factors` table that §14.5/§15.6 read; §14.5 now
      compares the stored string value.
- [x] Finding 2 — the scenario-editing restart is wired through a new
      `run_interactive_session` driver (§1.2) that the frame loop returns
      to, keeping the rebuild/setup out of the loop; §15.4 aligned.

---

## CODE

Implementation of `src/rigid_body/`, transcribed from PSEUDOCODE
bottom-up so each module tests against a real oracle as it lands. Run the
suite in the project's `rigid` venv: `pytest tests/ -v`.

- [x] `core/orientation.py` (PSEUDOCODE §2) — quaternion algebra, the
      sandwich rotation and body-to-space matrix, the kinematic q_dot, the
      Euler and axis-angle builders, and the Euler read-out. 17 unit tests.
      The read-out measures `sin(theta)` as `hypot(R[0][2], R[1][2])` to
      stay robust at the gimbal-lock poles (a `sqrt(1 - cos^2)` form
      misflagged `theta = pi`).
- [x] `body/shapes.py` + `body/analytic_inertia.py` (PSEUDOCODE §4.3,
      DESIGN §3.2) — the shape primitives and the closed-form inertia
      provider. 24 unit tests; the Platonic moments are checked against the
      exact tetrahedron decomposition in `dev/spikes/platonic_inertia.py`.
      That check confirmed the icosahedron is `(3 + sqrt5) / 20`, and a
      stale factor-of-two candidate in the spike's own display table was
      corrected to match.
- [x] `dynamics/state.py` (PSEUDOCODE §3) — the State record and the
      derived quantities (body/space angular momentum, kinetic energy).
      5 unit tests against exact identities.
- [x] `body/rigid_body_model.py` (PSEUDOCODE §4.1, §4.6) — the RigidBody
      record, the TopClass enum, and classify_top. 5 unit tests. The full
      build_body constructor (diagonalization, pivot, validation) is
      deferred to the scenario-assembly step.
- [x] `dynamics/torque_models.py` (PSEUDOCODE §6) — GravityTorque,
      ViscousDamping, and the fixed-order total. 5 unit tests, with
      tau = r x F checked by hand. 56 unit tests total.
- [x] `dynamics/equations_of_motion.py` (PSEUDOCODE §5) — the scalar Euler
      equations, the pure state_derivative, the instability growth rate,
      and energy_rate. 12 unit tests: exact match to the spike's
      euler_derivative, derivative-level conservation of energy and |L|,
      and the growth-rate formula. 68 unit tests total.
- [x] `dynamics/integrators.py` (PSEUDOCODE §7) — fixed-step RK4 with the
      field-wise state arithmetic (scale_state/add_states/advance_by), the
      post-step quaternion renormalization, and select_integrator. 9 unit
      tests, including the end-to-end oracle: RK4 integration of
      state_derivative reproduces the analytic elliptic solution of
      `free_top_elliptic.py` to < 1e-5, with fourth-order convergence and
      conserved energy and |L|^2. 77 unit tests total.
- [x] `dynamics/simulation_engine.py` (PSEUDOCODE §1.1, §1.3) +
      `sinks/sink_interface.py` — the atomic substep, the batch run loop,
      emit/finalize, and the abstract Sink. 6 integration tests: the
      determinism guarantee (bit-for-bit identical runs; read-only sinks
      and monitor), emit fan-out, and torque-free conservation through the
      engine. 83 tests total. The interactive loop (§1.2) waits on
      `render/` and `ui/`.
- [x] `body/rigid_body_model.py` build_body (PSEUDOCODE §4.2, §4.4, §4.5,
      §4.7) — validate_moments/validate_inertia_tensor, parallel_axis_shift,
      principal_frame_of (diagonal shortcut + eigh + handedness fix),
      build_body_from_shape (with the optional pivot) and
      build_body_from_moments. 11 unit tests. 94 total.
- [ ] Deferred: degenerate-subspace axis canonicalization to the geometry
      axis (DESIGN §3.4) in principal_frame_of — only bites for a
      re-diagonalized symmetric top (off-axis pivot); natural-orientation
      primitives take the diagonal shortcut.
- [x] `core/units.py` (PSEUDOCODE §12.4, ARCH §5.5) — the pint boundary:
      parse_quantity turns an authored string into a bare SI float with a
      dimension check. 8 unit tests. 102 total.
- [x] `scenario/` (PSEUDOCODE §12) — the two-zone schema (`scenario.py`,
      `fidelity.py`) holding both authored and resolved forms, and
      `serialization.py`: TOML load/save (tomllib/tomli read, tomli-w
      write), pint unit resolution, build_body_from_specification, the
      resolved-vs-recomputed consistency check, and
      run_batch_from_scenario. 6 integration tests. 108 tests total.
      **The batch tier is now fully functional from a scenario file.**
- [x] `analysis/analytic_solutions.py` (PSEUDOCODE §9) — steady principal
      rotation, the symmetric-top precession rates, the torque-free
      asymmetric top in Jacobi elliptic functions (both branches), and the
      heavy-top steady-precession quadratic. 11 unit tests: the asymmetric
      forms match the certified `free_top_elliptic.py` spike exactly, and
      steady rotation and the symmetric body rate are cross-checked by
      integration. 119 tests total. This surfaced a backwards precession-
      sign in the DESIGN §8.3 / PSEUDOCODE §9.3 prose (the formula was
      right); corrected at all three levels — prolate negative, oblate and
      the Earth positive, so the Chandler wobble is prograde.
- [ ] Next: the conservation monitor (§8), Poinsot and frames (§10, §11),
      then the render/ui interactive tier (`scene_description`, `palettes`,
      `vedo_renderer`, `controls`, `time_control`) and the rbsim/rbbatch
      entry scripts (XYZ idiom).

---

## ARCHIVE

<!-- Resolved items go here. Keep them for reference; use stable numbering
for cross-references. -->
