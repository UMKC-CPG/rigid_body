# Task List

> **Document hierarchy:** Tasks are organized by the level of the design
> chain they affect. Each item should cite the relevant document section.

---

## VISION

<!-- Tasks related to goals and principles. -->

---

## ARCHITECTURE

<!-- Tasks related to layout, modules, build. -->

- [ ] (§9.5) Route B has been run on Linux only. Have someone run the
      README's four lines on **macOS** and on **Windows** and send back
      what `rbsim --check` prints; record the result in §9.5 "What has
      been tried".
- [ ] (§9.5) Cut a release tag once Route B is confirmed, and point the
      README's `pip install` URL at `refs/tags/<tag>` so that a class
      installs a fixed version rather than `main`.

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
suite in the `physdemo` environment (`sdemo`): `pytest tests/ -v`.

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
- [x] `analysis/conservation_monitor.py` (PSEUDOCODE §8) — the
      ConservationMonitor watching kinetic_energy and angular_momentum_space
      for drift by BALANCE, not constancy: a trapezoid accumulator over the
      §5.5 rate laws, fixed-scale/per-unit-time reporting, the
      magnitude/direction split on L, an online least-squares TrendTracker
      for secular growth, and poinsot_identity_residual as a test oracle
      only. 10 unit tests: exact analytic motion reports machine-zero drift;
      under damping the balance residual is < 1e-3 x the naive constancy
      drift; under gravity the momentum balance holds while L swings; the
      monitor never mutates the state nor changes the trajectory; the
      identity reads zero even on a wrong trajectory; the trend slope is
      positive for a growing residual and zero for a bounded one. 129 total.
- [x] `geometry/poinsot.py` (PSEUDOCODE §10) — the Poinsot construction as
      pure geometry (no drawing): momental_ellipsoid (semi-axes 1/sqrt(I)),
      poinsot_contact_point (rho = omega/sqrt(2T)), invariable_plane (normal
      along L, distance sqrt(2T)/|L|), polhode by class (a point for the
      spherical top, an analytic circle for the symmetric top, the certified
      Jacobi-elliptic curve for the asymmetric top via an omega_2 = 0 anchor
      reconstructed from the state's own invariants), a numerical
      intersect_quadrics fallback (sweep the middle-axis amplitude, solve the
      2x2 for the outer squared components, tile four sign octants into the
      closed loop), and the herpolhode point and band radii. 16 unit tests:
      every polhode point sits on BOTH invariant quadrics to ~1e-15 in all
      three classes; the analytic and numerical routes trace the same curve;
      an RK4 torque-free trajectory keeps omega on the analytic polhode; the
      invariable plane holds still through a run; the herpolhode lies in it;
      and the band is a circle for a symmetric top, an annulus for an
      asymmetric one. 145 total.
- [x] `geometry/reference_frames.py` (PSEUDOCODE §11) — one motion in
      either frame, drawing nothing: express_in_space/express_in_body (the
      single body_to_space rotation and its inverse), the Frame enum and
      to_view (express a quantity in the held-still view), frame_invariants
      (the frame-free scalars: energies, |L|, |omega|, the omega-to-L angle,
      the moments), and angular_momentum_body_curve (L = I omega along the
      polhode, the §11.4 companion curve). 11 unit tests: the two mappings
      are exact inverses and agree with the state layer's
      angular_momentum_space; to_view passes a view-frame vector through and
      carries the other across; the omega-to-L angle is identical read in
      body or space; through a torque-free run the conserved invariants hold
      while |omega| and the angle demonstrably vary (frame-free is not
      time-constant); the body-frame L sweeps while the space-frame L holds
      still; and the companion curve rides the fixed-radius |L| sphere.
      **Completes the headless geometry layer.** 156 total.
- [x] `sinks/hdf5_sink.py` (PSEUDOCODE §13.6, ARCH §9.4) — the recording
      sink that spills the full state stream to disk with bounded memory:
      resizable, chunked HDF5 datasets (time, body_to_space_quaternion,
      angular_velocity_body) fed from a small in-memory batch flushed when
      full; self-describing root metadata (schema version, SI units,
      quaternion convention, sample count) with the run's scenario embedded
      as TOML provenance; and an XDMF companion (a temporal point collection
      hyper-slabbing each timestep out of the datasets) so ParaView can read
      the trajectory. 7 integration tests: disk matches the emitted stream
      byte-for-byte across several flushes plus a partial one; hand-fed
      states round-trip exactly; recording does not change the trajectory
      (read-only); provenance/metadata are present; an empty run and a
      double close are clean; and the XDMF is well-formed with one timestep
      per sample pointing back at the HDF5 datasets. 163 total.
      (Provenance wiring -- a scenario_to_toml helper feeding this sink --
      lands with the rbbatch entry script.)
- [x] `render/scene_description.py` (PSEUDOCODE §14.2, §14.5, §14.6) — the
      renderer-agnostic middle stage: a Drawable record (geometry, role,
      panel, coordinate_frame, label, scale_note), build_scene assembling
      the per-frame inventory from computed quantities, the ellipsoid-scale
      label (§14.5), and the telemetry overlay (§14.6, drift report + trend
      + time ratio + Euler angles with degeneracy). The Poinsot trio is
      torque-gated (present only when the monitor's torque list is empty); a
      moments-only body falls back to the ellipsoid proxy; the two shared
      arrows are stored in their native frames (omega body, L space) with
      coordinate_frame so the renderer can re-express them per panel; every
      drawable carries a non-empty label (Principle 5) and meaningful
      distinctions are redundant in role/label, not color alone (§14.3). 13
      unit tests. 176 total.
- [x] PSEUDOCODE §14.2 sync: the Drawable record now names the panel field
      `panel` and adds a `coordinate_frame` field (BODY | SPACE | NONE),
      with a prose note on the panel-vs-coordinate-frame distinction and
      the build_scene/§14.6 calls updated. This closes the code-level
      refinement the scene_description commit surfaced -- the renderer needs
      each shared vector's native frame to call to_view (§11.2). DESIGN
      §13.2's prose "Frame" column stays accurate (it names the panel) and
      needed no change.
- [x] `render/palettes.py` (PSEUDOCODE §14.3, §14.4) — the selectable
      role->encoding table: an Encoding record (color, line_style,
      line_weight, opacity, marker), a Palette record, three built-in
      schemes (light, dark, color_blind_safe using Okabe-Ito hues), and
      select_palette/resolve_encoding/frame_family_of. Redundancy is
      structural: only color changes between schemes while the shape
      channels (line style, marker, weight, opacity) are palette-independent
      per role, and each frame owns one consistent hue (§14.4). 19 unit
      tests: palettes cover exactly the roles build_scene emits; no two
      roles share an identical encoding; any two same-color roles differ in
      a non-color channel; the called-out pairs (polhode/herpolhode dash,
      omega/L arrowhead) differ by a shape channel in every scheme; each
      frame owns one hue; select/resolve reject unknowns loudly; and
      switching palettes preserves every shape channel. 195 total.
- [x] `dynamics/time_control.py` (PSEUDOCODE §15.3) — the pure pacing core
      of the time controls (note: ARCH §3.3 puts time_control in dynamics/,
      not ui/): the ControlMode/Pace enums, the Controls plain-data snapshot,
      SLOW_FACTOR/FAST_FACTOR, substeps_this_frame (the map onto a substep
      COUNT, never dt), pace_after_frame (single-step re-pauses), and
      default_controls. 10 unit tests: each pace maps to the right count;
      slow motion never stalls below one step; the pace order is
      monotonic; single-step re-pauses; the Controls record carries no
      dt/time_step, so pacing structurally cannot change the step size
      (determinism at the type level); default_controls starts LIVE at the
      scenario's nominal rate.
- [x] `ui/controls.py` (PSEUDOCODE §15.4, §15.6) — the headless
      scenario-editing surface: apply_scenario_edit (an edit is a NEW
      scenario via dataclasses.replace, never a mutation), replace_section
      (the nested-knob convenience), and scale_setting_labels (honest
      Principle-12 labels for every exaggeration factor != 1). 7 unit tests:
      an edit returns a fresh scenario and leaves the original intact; an
      empty edit is a no-op; a physics-field edit is a genuinely different
      run; a presentation-only edit leaves every physics field the same
      object (the §15.1 split); replace_section edits a nested knob without
      mutation and flows into the pacing; and the scaling labels name every
      exaggeration while skipping a factor of one. 212 total.
- [ ] Deferred with the vedo tier: `ui/controls.py`'s actual vedo widget
      bindings (read_controls reading a live window into a Controls
      snapshot), and time_control's replay_state_at (§15.5), which needs
      the still-unbuilt `dynamics/trajectory.py` (§13.4/§13.5 ring buffer +
      keyframes).
- [x] `dynamics/trajectory.py` (PSEUDOCODE §13.4, §13.5) — retention as a
      bounded, read-only cache over the recomputable Markovian motion.
      RetainedSample (state + time + optional monitor_accumulators, the one
      path-dependent exception); Trajectory, the ring buffer (append copies
      the state in and overwrites the oldest when full; read/newest/oldest
      by cursor; time_span/contains_time for the replay scrubber;
      from_retention); and KeyframeStore, sparse exact keyframes plus
      re-integration from the nearest earlier one via advance_one_substep.
      16 unit tests: appends read back in order and the full buffer
      overwrites the oldest; a stored state is isolated from the live state
      (mutating the source can't reach it); out-of-window reads raise;
      time_span/containment and the empty case; and the sharp one --
      keyframe re-integration reproduces the reference trajectory
      BIT-FOR-BIT for both torque-free and gravity runs (the Markov
      property), keeps only stride multiples, rounds read_at_time onto the
      substep grid, and does not disturb the stored keyframe. 228 total.
      This closes §13; the deferred time_control.replay_state_at (§15.5) can
      now compose Trajectory.read (in-window) with KeyframeStore.read_at_time
      (deep past).
- [x] `dynamics/time_control.replay_state_at` (PSEUDOCODE §15.5) — the
      replay scrubber lookup, composing the two §13 retention policies into
      one read: the recent past from the ring buffer, the deep past from the
      keyframe store. Added `Trajectory.read_at_time` (nearest in-window
      sample by binary search, ends clamped) as the by-time in-window read.
      A future target is clamped to the frontier (replay cannot outrun the
      engine); a deep-past target with no keyframe store raises LookupError.
      7 unit tests: in-window reads match the reference exactly; the deep
      past re-integrates bit-for-bit through keyframes; a future time clamps
      to the newest state; no-keyframe deep past raises; replay is read-only
      (repeat reads agree) across both paths; and read_at_time finds the
      nearest sample / clamps at the ends. This closes the last headless
      §15.5 gap. 235 total.
- [x] `scripts/rbbatch.py` + `scripts/rbbatchrc.py` (ARCH §3.9, §7) — the
      batch entry point in the XYZ/XYZrc idiom. ScriptSettings pulls
      machine-dependent defaults from rbbatchrc.py (output dir, XDMF on/off,
      flush size, monitor on/off -- never physics) and reconciles them with
      argparse (rc < scenario < CLI); the testable core run_batch_job loads
      a scenario, attaches an Hdf5Sink carrying the scenario as embedded
      provenance, optionally runs a ConservationMonitor over the run's own
      body/torques, advances the deterministic batch loop, and returns a
      BatchResult. Also added `serialization.scenario_to_toml` (the deferred
      provenance helper) and `serialization.build_run_components` (the
      resolved pieces, shared by run_batch_from_scenario and the monitor).
      7 integration tests: writes the expected 100-sample trajectory; the
      output embeds reloadable provenance; the torque-free drift report is
      ~machine zero; XDMF and the monitor each switch off; default_output_path
      uses the scenario stem; and two runs are byte-for-byte identical
      (determinism end to end). 242 total. **The batch tier is now runnable
      from the command line.**
- [x] `sinks/live_sink.py` (ARCH §3.7, §5.2) — the interactive tier's end
      of the sink boundary: a read-only Sink that transports the engine's
      state stream to the renderer layer. Captures the latest (state, time)
      for the frame loop to draw once per frame, and fires an optional
      per-state hook (on_state) and close hook (on_close). Decoupled from
      vedo (plain callables, so a fake renderer drives it in tests) and
      holds the latest state by reference, not copy, since the engine never
      mutates a produced state. 8 integration tests: it is a Sink and starts
      empty; receive captures the latest and counts; the on_state/on_close
      hooks fire the right number of times; hooks are optional; receive does
      not mutate the state; and through the engine it sees every substep,
      its latest equals the final state, and attaching it leaves the
      trajectory byte-for-byte unchanged. 250 total.
      NOTE (design chain): live_sink is specified only at the ARCHITECTURE
      level; §1.2 renders inline from the loop's own state. Built here as a
      transport adapter -- the shape the constraints leave to code -- with
      its exact frame-loop/vedo_renderer interplay settled when those land.
- [x] `render/vedo_renderer.py` (ARCH §5.3) — the renderer boundary: the
      only module that imports vedo/VTK. VedoRenderer turns a Scene + Palette
      into pixels, one panel per held-still frame (side_by_side for the
      Goal-5 two-frame view, or single). Each drawable is re-expressed into
      its panel's frame at draw time via its coordinate_frame (§11.2), and
      encoded by the palette (color/opacity/line weight, dashed/dash-dot
      stipple, single vs double arrowhead). Actor builders for all ten
      roles: body mesh (per shape primitive), momental ellipsoid (scaled
      wireframe), polhode (line, or point for a spherical top), invariable
      plane, herpolhode marker, the omega/L direction arrows (drawn at a
      display length tied to the ellipsoid, a labeled §14.5 convention),
      the two triads, and the Text2D telemetry overlay. Verified offscreen:
      a rich two-panel render with correct frame coding (body warm hue /
      space cool hue). 5 integration tests (framebuffer readback, skip
      without GL): the scene rasterizes to real pixels in both layouts; the
      color-blind palette renders; a damped cube (torque, no Poinsot,
      point polhode) renders; and successive frames render without error.
      255 total.
      DEFERRED refinements (all documented in code): the herpolhode swept
      trail + bounding band (needs renderer-held frame history), tighter
      per-panel camera framing, and Platonic body meshes (currently a sphere
      proxy -- the physics is spherical anyway).
- [x] `ui/interactive_session.py` (PSEUDOCODE §1.2) — the interactive
      driver: run_interactive runs one scenario's frame loop (read controls;
      return a pending edit, or replay stored history, or advance the live
      physics by substeps_this_frame, then build the scene and draw once per
      frame), and run_interactive_session applies each edit and runs the
      next scenario, finalizing sinks at session end. Renderer- and
      controls-agnostic: imports only the headless scene_description, takes
      the renderer and a ControlsSource as objects, so the whole loop is
      tested with fakes -- no vedo, no display. 9 integration tests: it
      advances and renders each frame; pause holds the motion but still
      redraws; single-step takes one substep; the same script gives the same
      rendered states (determinism); a pending edit stops the run and is
      returned; the session applies a physics edit into a genuinely
      different run; replay redraws a stored past state; a live sink taps
      every substep and its latest matches the drawn frame; and sinks are
      finalized at session end. Also added DESIGN §13.7 pinning the live
      render path (frame loop renders inline once per frame; live_sink is a
      per-substep tap; both read-only). 264 total.
- [x] BUG FIXED (poinsot/§10, was pre-existing): a state spinning (near)
      exactly about a principal axis made the asymmetric polhode's elliptic
      reconstruction take math.sqrt of a rounding-sized negative at the
      branch boundary -> ValueError("math domain error"). poinsot.polhode
      now detects a principal-axis spin (_is_principal_axis_spin: at rest,
      or only one appreciable omega component, rel. tol 1e-6) in the
      asymmetric branch and returns a single-point polhode (steady rotation,
      §9.2) -- covering all three axes, including the intermediate one whose
      separatrix would otherwise give an infinite period. 6 tests: a spin
      about each principal axis is a point on both quadrics; a near-axis
      spin is a point; an appreciably off-axis spin stays a proper elliptic
      loop (the guard swallows no real curves); and the predicate itself.
      Verified the original repro (build_scene with omega=[3,0,0]) no longer
      crashes. 270 total.
- [x] `ui/vedo_controls.py` + `scripts/rbsim.py` + `scripts/rbsimrc.py` —
      the interactive entry point. vedo_controls: KeyboardControlState (pure,
      graphics-free time-control state machine: space toggles pause, s
      single-steps then re-pauses via pace_after_frame, -/+ slow/fast, n
      normal, q quits), VedoControlsSource (binds it to a live plotter via a
      KeyPress callback + a non-blocking event pump; imports no vedo, only
      the plotter it is handed), and AutoControlsSource (input-free fixed
      frame count for offscreen/headless). rbsim: ScriptSettings in the XYZ
      idiom (rc window size + default layout/palette, precedence rc <
      scenario < CLI), and the testable run_interactive_job that loads a
      scenario, builds a VedoRenderer + a controls source (live, or Auto for
      --offscreen/--frames), and runs run_interactive_session; resolve_palette
      falls back to light for an unknown name. 14 tests: the full keyboard
      vocabulary; the auto source's frame bound; run_interactive_job renders
      the requested frames and advances, is deterministic, leaves an injected
      renderer open; palette fallback; and a guarded real-offscreen run.
      Verified end to end: `rbsim spin.toml --offscreen --frames N` renders
      the two-frame view (drift ~1e-14). 284 total. **The interactive tier
      is now runnable from the command line.**
- [x] Live-legibility pass (viewer reports from the first Open OnDemand
      run). PRESENTATION only, no physics: per-panel camera framing (each
      panel resets to its own contents on the first frame, so the body
      panel no longer starts inside the ellipsoid); on-screen labels
      realized from each drawable's label (omega/L at the tips, triad arms
      tagged 1/2/3 and X/Y/Z); panel titles ("Body frame"/"Space frame");
      palette-coherent background (the background now travels with the
      palette, so the light scheme's dark inks no longer draw on black);
      coarser ellipsoid wireframe (res 32 -> 12) at a touch more opacity.
      285 total.
- [x] Rolling ellipsoid in the space panel (§11.5). The body-anchored
      drawables (object, ellipsoid, polhode, body axes) were panel = BODY,
      where the body frame is held still -- so a viewer never saw them
      move, though §11.5 has always specified the ellipsoid rolling on the
      invariable plane in the space view. Made them panel = BOTH: still on
      the left, rolling on the right. Aligned the §14.2 build_scene spec
      (which contradicted §11.5) and the panel test. 285 total.
- [x] Display-layer toggles + on-screen key legend (new PSEUDOCODE §15.7).
      Four layers (body; ellipsoid + construction; vectors; triads) toggled
      by b/e/v/t; visible_layers on the read-only Controls record; build_scene
      filters its inventory (None = draw all, the batch tier); the telemetry
      overlay is always-on chrome in no layer. control_legend_lines() drawn
      in the window corner so the keys are discoverable. 15 new tests, 300
      total.
- [x] Per-layer display scale + ellipsoid ring cage (viewer reports).
      PRESENTATION only. The object and the momental ellipsoid live in
      different spaces (metres vs 1/sqrt(I)) with no common scale, so the
      object was a speck inside the ellipsoid; it is now drawn at a
      per-layer display scale (largest half-extent = 0.7x the reference
      size), uniform so its shape is exact, with an honest "not to scale"
      scale_note (§14.5). The reference size moved onto the Scene so it is
      stable when the ellipsoid layer is toggled off. The ellipsoid is now
      a latitude/longitude ring cage (parallels + meridians) rather than a
      triangulated wireframe, so the diagonals no longer clutter the shape
      and the object shows through. 5 new tests (ring points lie on the
      surface and split by kind; the body scales to the display fraction
      keeping its edge ratios; the reference scale survives hiding the
      ellipsoid; the object states it is not to scale). 305 total.
- [x] BUG FIXED (serialization/§11.3): the interactive tier drew NO object
      at all. build_run_components -> _rigid_body_from_resolved set
      geometry=None, so build_scene skipped the body_mesh for every loaded
      scenario (the "body" layer was empty; the object seen earlier was
      arrow bases). DESIGN §11.3 says the specification is "what the
      renderer needs in order to draw a shape" -- so the shape is now
      rebuilt from the specification and attached as geometry (drawing
      only; the dynamics still run on the recorded tensor, determinism
      untouched). 2 round-trip tests: a reloaded shape body carries its
      geometry while its resolved moments are unchanged; a moments-only
      body stays geometryless (ellipsoid proxy).
- [x] Scale notes surfaced on screen (§14.5, Principle 12). active_scale_notes
      gathers the scale_note of each drawable the scene shows (so a note
      appears/vanishes with its layer), and the renderer draws them as a
      bottom-right footnote: "object enlarged for visibility (not to
      scale)" and "ellipsoid scaled to inertia (unit form)". Making a
      quantity visible off its true scale is now never silent. 2 tests
      (the footnote lists both shown scaled quantities; hiding a layer
      drops its note). 309 total.
- [x] Keyboard commands chorded with Ctrl (§15.7). vedo's own viewer owns
      the bare keys (e closes the window, +/- cycle the axis style), so
      every live control is now Control-chorded: Ctrl+space pause, Ctrl+s
      single-step, Ctrl+-/+ slow/fast, Ctrl+n normal, Ctrl+q quit, and
      Ctrl+b/e/v/t for the layer toggles. A chorded key matches nothing in
      vedo's plain-key dispatch table, so the collisions are gone by
      construction rather than by luck; bare q/Escape stay honored too, so
      a window the backend closes on its own still trips the loop's closed
      flag. The corner legend and PSEUDOCODE §15.7 updated to match. The
      keyboard tests were rewritten onto the chords (net 309 total).
- [x] Ellipsoid mesh-density control (§15.7). The wireframe globe's ring
      count is now the viewer's to set: an abstract detail level (1..5,
      default 2) rides the read-only Controls record and passes through
      build_scene onto the Scene, and the renderer alone maps the level to
      a (parallels, meridians) count, so the level stays renderer-agnostic
      and the default reproduces the shipped 5x8 look. Ctrl+[ coarsens and
      Ctrl+] thickens the cage, clamped to its bounds; the legend documents
      both. 10 new tests (the control record, the scene passthrough, the
      monotone level->count map, and the keyboard stepping, clamping, and
      orthogonality to the pace and layers). 319 total.
- [x] Polhode drawn on the ellipsoid surface (§10.4). poinsot.polhode
      samples the path of omega at the angular-velocity scale, so the drawn
      curve floated a factor sqrt(2T) outside the momental ellipsoid a viewer
      sees. _polhode_geometry rescales the loop to the contact point rho =
      omega/sqrt(2T), where rho . I . rho = 1 places every point exactly on
      the surface; the herpolhode band still reads the omega-scale samples it
      rescales itself. 1 new test (every drawn point on the surface),
      confirmed visually on the dzhanibekov scenario. 320 total.
- [x] Herpolhode and polhode swept trails (§10.5, §14.2). One vector, omega,
      draws the polhode on the rolling ellipsoid and the herpolhode on the
      fixed plane at once (Goal 5); each drawable carries its current contact
      point and the renderer accumulates it across frames -- the one piece of
      frame history it holds, reset per run by the driver. The polhode shows
      its analytic loop faintly with a bright comet trail on top; the
      herpolhode, drawable from no single instant, is only the trace, filling
      the band whose two bounding circles are drawn. The window is bounded
      and fades tail-to-head (a moving comet, not a smeared band); paused and
      replayed frames dedup, so a still frame keeps its trail. 8 new tests
      (accumulate/dedup/bound, band basis and circle geometry, a GL
      accumulate-then-reset, and the driver reset). 328 total.
- [ ] Deferred interactive refinements: true Platonic body
      meshes; live scenario editing through the UI (currently pending_edit
      stays None -- time, layer, and mesh controls only); window-X close
      detection (v1 quits on Ctrl+q, with bare q/Escape as a safety net);
      and a live on/off marker in the key legend (currently a static
      reference; the toggle's effect is seen in the scene itself).

---

## ARCHIVE

<!-- Resolved items go here. Keep them for reference; use stable numbering
for cross-references. -->
