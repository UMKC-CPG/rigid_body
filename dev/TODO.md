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

**All DESIGN sections are written**, §1–§13 committed. §14 (interaction and
controls) and the refine edits are drafted and awaiting commit.

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

**Next milestone — close out DESIGN and open PSEUDOCODE:**

- [ ] Commit §13 (done: `ba2d9fd`) — then commit §14 and the refine edits.
      **This is the next step.**
- [ ] Tag `v0.3-design` (ARCHITECTURE §10) once the refine edits are
      committed and the chain is consistent. (Refine edits committed at
      `018bd0d`; tag command prepared, awaiting the programmer to run it.)
- [x] Begin `PSEUDOCODE.md` — scaffold (notation, Contents table) and §1
      (the simulation loop) drafted.

**Open items carried forward:**

- The §8.4 elliptic solution is numerically verified
  (`dev/spikes/free_top_elliptic.py`, passing). PSEUDOCODE and code for
  `analytic_solutions.py` must reproduce the forms that spike certifies.
- Two DESIGN forward-obligations for the code level: `numerical_inertia.py`
  must reproduce §3.2 to a stated tolerance (§3.7), and the TOML schema
  (§11.7) must be given its exact key layout in PSEUDOCODE.

---

## PSEUDOCODE

<!-- Tasks related to algorithm specifications. -->

`PSEUDOCODE.md` is being drafted section by section, mirroring the
algorithmic subset of DESIGN. Status is tracked in its own Contents
table; this list records only what remains.

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
- [ ] §12 Scenario load and save (DESIGN §11). **Next section** — this is
      the one to give the full, exact TOML key layout (§11.7); it was
      deferred earlier and is now due.
- [ ] §13–§15 follow the Contents table in `PSEUDOCODE.md`, in DESIGN
      order (trajectory retention, scene description, controls and time).

**Resolved (DESIGN §6.1 signature).** DESIGN §6.1 now reads
`advance(state, time, dt, derivative_function)`, with a clause explaining
that a multi-stage method samples the derivative at `time + c*dt` and a
time-dependent torque (DESIGN §5.7) reads that absolute time. PSEUDOCODE
§1.1 and §7.1 now describe their signature as *matching* DESIGN rather
than refining it. The three levels agree; nothing further pending.

---

## CODE

<!-- Tasks related to implementation. -->

---

## ARCHIVE

<!-- Resolved items go here. Keep them for reference; use stable numbering
for cross-references. -->
