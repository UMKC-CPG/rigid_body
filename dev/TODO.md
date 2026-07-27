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
- [ ] §4 Inertia and body construction (DESIGN §3). **Next section.**
- [ ] §5–§15 follow the Contents table in `PSEUDOCODE.md`, in DESIGN
      order, ending with §12 (the exact TOML key layout, per §11.7) and
      §9 (the analytic forms the `free_top_elliptic.py` spike certifies).

One refinement surfaced while drafting §1, worth a later `/refine`
decision: §1 threads `time` through `integrator.advance(...)`, whereas
DESIGN §6.1 wrote `advance(state, dt, derivative)` without it. Time is
needed for time-dependent torques (DESIGN §5.7). Either update DESIGN
§6.1's signature or note the refinement there.

---

## CODE

<!-- Tasks related to implementation. -->

---

## ARCHIVE

<!-- Resolved items go here. Keep them for reference; use stable numbering
for cross-references. -->
