# Architecture

> **Document hierarchy:** VISION → **ARCHITECTURE** → DESIGN → PSEUDOCODE
> → Code. For goals and principles, see `VISION.md`.

---

## 1. Repository Layout

```
rigid_body/
  dev/
    VISION.md         Goals and principles
    ARCHITECTURE.md   This document
    DESIGN.md         Algorithmic design
    PSEUDOCODE.md     Algorithm specifications
    TODO.md           Task list by level
    spikes/           Throwaway experiments whose results are cited
    notes/            Dated working notes (not binding)
  src/
    rigid_body/       The importable library (all physics and display)
      core/           Units and rotation mathematics
      body/           Shapes, density, inertia computation
      dynamics/       Equations of motion, torques, integrators
      analysis/       Conservation monitoring, analytic solutions
      geometry/       Derived display geometry (Poinsot, frames)
      scenario/       Scenario definition and serialization
      sinks/          Consumers of a computed trajectory
      render/         Scene description, palettes, vedo backend
      ui/             Interactive control panels
      cli/            Bodies of the commands (§3.9)
      defaults/       The shipped rc files (§7)
      examples/       Ready-to-run example scenarios (TOML)
    scripts/          Thin executable fronts for cli/ (§3.9)
  scenarios           Symbolic link to src/rigid_body/examples/
  tests/              Test suite (unit, integration, regression)
  pyproject.toml      Packaging (§9.5, Route B)
  CLAUDE.md           AI assistant guidance
```

The library lives under `src/rigid_body/` and holds everything
reusable. The commands follow the existing `XYZ.py` / `XYZrc.py`
convention — a settings class whose defaults come from a
resource-control file and are then reconciled with command-line
arguments — with one change forced by §9.5: their bodies live in the
library (`cli/`), and `src/scripts/` holds only the few lines that
start them.

**Everything the tool needs at run time lives under
`src/rigid_body/`,** because that directory is all that an installed
copy contains (§9.5): the code, the shipped rc defaults, and the example
scenarios. `scenarios` at the top level is kept as a symbolic link so
that `scenarios/dzhanibekov.toml` remains the short path it has always
been in a clone; it is a convenience of a checkout and nothing may
depend on it. (On a Windows checkout without symbolic-link support it
appears as a small text file. Windows users are served by the installed
route and `rbsim --examples`, not by a clone.)

---

## 2. Execution Tiers

The project runs the *same* physics in two modes that differ only in
fidelity and in what consumes the result. This is the central structural
idea of the architecture, and it is what VISION Principle 9 (physics
decoupled from presentation) exists to make possible.

```
  TIER 1: INTERACTIVE EXPLORATION        TIER 2: BATCH HIGH FIDELITY
  ------------------------------         ---------------------------
  Low fidelity, real time                High fidelity, no display
  Runs on a cluster desktop node         Runs as a scheduled HPC job
  Renders live through vedo              Writes HDF5 (+ XDMF)
  Purpose: find the interesting          Purpose: quantitative results,
  initial conditions and run times       long runs, expensive bodies

                \                        /
                 \                      /
                  v                    v
              +--------------------------+
              |   Shared physics core    |
              | body / dynamics/ analysis|
              +--------------------------+
```

**The scenario file is the bridge between the tiers.** A student explores
interactively until the motion looks worth studying, saves the complete
scenario (VISION Goal 11), and submits that same file to the batch tier.
Nothing is re-entered by hand, so the batch run is guaranteed to be the
run that was explored. Results return as HDF5 for post-processing or for
visualization in ParaView on the cluster's visualization capability.

**Fidelity is a set of scenario parameters, not a separate code path.**
The knobs are: the inertia computation method and its grid resolution,
the integrator type and time step, the trajectory sampling cadence, and
the retained trace length. Tier 1 and Tier 2 differ only in the values
of these knobs.

Per the programmer's direction, **Tier 2 is designed for now and built
later.** The abstractions below (especially the sink interface and the
scenario format) must be in place from the start so the batch tier drops
in without disturbing the interactive tier.

---

## 3. Module Map

### 3.1 `core/` — Foundations

| Module | Single responsibility |
| --- | --- |
| `units.py` | SI unit and dimension handling (Principle 11) |
| `orientation.py` | Quaternions, rotation matrices, Euler angles |

`orientation.py` owns the orientation representation so that no other
module has to know which one is used. This is what lets the choice be
made on numerical-artifact grounds (Principle 2) rather than convenience.

### 3.2 `body/` — What is rotating

| Module | Single responsibility |
| --- | --- |
| `shapes.py` | Shape primitives: sphere, ellipsoid, box, cylinder, cone |
| `density.py` | Density specification (uniform now, field later) |
| `inertia_provider.py` | The interface: shape + density -> inertia tensor |
| `analytic_inertia.py` | Closed-form tensors for uniform symmetric solids |
| `numerical_inertia.py` | Integration over a density field (future) |
| `rigid_body_model.py` | Assembled body: mass, center of mass, tensor, axes |

`inertia_provider.py` is the seam that VISION Principle 10 demands. It is
described in §5.1.

### 3.3 `dynamics/` — How it moves

| Module | Single responsibility |
| --- | --- |
| `torque_models.py` | External torque sources, composed as a list |
| `equations_of_motion.py` | Euler's equations: state -> state derivative |
| `integrators.py` | Time-stepping schemes, selectable per scenario |
| `simulation_engine.py` | The run loop; emits states to a sink |
| `trajectory.py` | State history buffer; supports replay (Goal 8) |
| `time_control.py` | Pause, step, slow motion, time scaling (Goal 8) |

`torque_models.py` treats gravity-through-a-pivot as one implementation
of a general interface, so that friction now, and electromagnetic torques
later (Future Direction 3), are additions rather than rewrites.

### 3.4 `analysis/` — Is it right?

| Module | Single responsibility |
| --- | --- |
| `conservation_monitor.py` | Track and report drift in E and L (Principle 2) |
| `analytic_solutions.py` | Closed-form motions for overlay and validation |

These two exist because VISION Principles 2 and 3 and Goal 9 all require
them. They are first-class runtime components, not test-only helpers: the
monitor's output is displayed live, and the analytic solution is drawn
alongside the numerical one.

### 3.5 `geometry/` — Derived display geometry

| Module | Single responsibility |
| --- | --- |
| `poinsot.py` | Momental ellipsoid, invariable plane, polhode, herpolhode |
| `reference_frames.py` | Express a quantity in the body or space frame |

This group computes *what* the geometric constructions are, in physical
coordinates. It performs no drawing. Keeping it out of `render/` means
the batch tier can write the polhode to HDF5 without a renderer present.

### 3.6 `scenario/` — The reproducible unit of work

| Module | Single responsibility |
| --- | --- |
| `scenario.py` | Complete description of a run, as data |
| `serialization.py` | Save and restore a scenario exactly (Goal 11) |
| `fidelity.py` | The fidelity knob set shared by both tiers |

A scenario holds the body, initial conditions, torque models, integrator
and fidelity settings, and viewpoint. It is plain data with no behavior,
which is what allows it to cross the tier boundary. See §7.

### 3.7 `sinks/` — What consumes a trajectory

| Module | Single responsibility |
| --- | --- |
| `sink_interface.py` | Abstract consumer of successive states |
| `live_sink.py` | Feeds the interactive renderer |
| `hdf5_sink.py` | Writes HDF5 plus an XDMF companion (future) |
| `video_sink.py` | Encodes a span of a run to video (Goal 11) |

Described in §5.2. This group is the tier boundary made concrete.

### 3.8 `render/` and `ui/` — Presentation

| Module | Single responsibility |
| --- | --- |
| `scene_description.py` | Renderer-agnostic list of things to draw |
| `palettes.py` | Light, dark, and color-blind-safe encodings (Prin. 6) |
| `vedo_renderer.py` | Realizes a scene description using vedo / VTK |
| `controls.py` | Interactive widgets and their bindings |

### 3.9 `cli/` and `scripts/` — Entry points

A command is reached in two ways (§9.5), and both must run the same
code. So the body of each command is a module in the library, and what
differs is only the few lines that start it.

| Module | Purpose |
| --- | --- |
| `cli/rbsim.py` | Body of the interactive simulation (Tier 1) |
| `cli/rbbatch.py` | Body of the batch job (Tier 2) |
| `cli/support.py` | What both share: finding the rc file and the |
| | packaged examples, copying them out, the self-check. INHERITED |
| | from the physdemo skeleton; holds no command name, the command |
| | modules pass theirs in (PSEUDOCODE §16.2) |
| `defaults/rbsimrc.py`, `defaults/rbbatchrc.py` | The shipped |
| | resource-control defaults (§7) |

| Front | How it is reached |
| --- | --- |
| `scripts/rbsim.py`, `scripts/rbbatch.py` | Executable scripts; the |
| | `physdemo` suite links them, and a clone runs them directly. |
| | Each puts `src/` on the path from its resolved location, then |
| | calls its `cli` module. |
| console scripts `rbsim`, `rbbatch` | Declared in `pyproject.toml`; |
| | created by `pip install`. Each calls `console_main`. |

Both fronts log the invocation to `command` and then call `main()`;
`main(argv)` itself never logs, so the test suite can call it freely.
Neither the fronts nor `cli/` hold any physics. `cli/` sits at the top
of the dependency graph, where `scripts/` was, and nothing imports it.

**Inherited files.** The fronts, `cli/support.py`,
`render/offscreen.py`, the `defaults/` and `examples/` package
docstrings, `tests/conftest.py`, the installed-copy and offscreen
tests, and the slash commands are the physdemo suite's skeleton files
with this tool's name substituted (PSEUDOCODE §16). They are changed
in the suite first and refreshed here, never edited here.

---

## 4. Dependency Graph

Dependencies point downward only. No module may import from a group
listed above it.

```
(scripts/rbsim.py and scripts/rbbatch.py are fronts: they import their
 cli/ module and nothing else)

cli/rbsim.py                        cli/rbbatch.py
  |                                   |
  +-- ui/ ------------+               |
  |                   |               |
  +-- render/         |               +-- sinks/hdf5_sink
  |     scene_description            |     video_sink
  |     palettes                     |
  |     vedo_renderer                |
  |                                  |
  +-- sinks/live_sink ---------------+
        |
        +-- sinks/sink_interface
              |
              +-- dynamics/simulation_engine
              |     +-- dynamics/integrators
              |     +-- dynamics/equations_of_motion
              |     +-- dynamics/torque_models
              |     +-- dynamics/trajectory, time_control
              |
              +-- analysis/conservation_monitor
              |   analysis/analytic_solutions
              |
              +-- geometry/poinsot, reference_frames
              |
              +-- scenario/scenario, serialization, fidelity
                    |
                    +-- body/rigid_body_model
                    |     +-- body/inertia_provider  <-- interface
                    |           +-- body/analytic_inertia
                    |           +-- body/numerical_inertia (future)
                    |     +-- body/shapes, density
                    |
                    +-- core/units, orientation
```

The single most important property of this graph: **nothing under
`dynamics/`, `body/`, `analysis/`, or `geometry/` may import from
`render/`, `ui/`, or `sinks/`.** That is the rule that keeps the batch
tier possible and satisfies Principle 9. It is enforced by a test (§8.6).

---

## 5. Key Boundaries

Five seams in this architecture exist specifically to protect a VISION
principle. Each is an interface that must remain stable.

### 5.1 The inertia-provider boundary (Principle 10)

`inertia_provider.py` defines one operation: given a shape and a density
specification, return the mass, the center of mass, and the inertia
tensor. `analytic_inertia.py` implements it with closed-form expressions;
`numerical_inertia.py` will later implement it by integrating over a
density field.

Everything downstream — `rigid_body_model.py`, all of `dynamics/`, all of
`geometry/` — sees only the returned quantities. No consumer may ask
which provider produced them or branch on the answer. This is the seam
that lets Future Direction 1 (arbitrary geometry, non-uniform density)
arrive without touching the dynamics.

### 5.2 The sink boundary (execution tiers)

`simulation_engine.py` does not know what happens to the states it
produces. It emits them to an object satisfying `sink_interface.py`.
Attaching `live_sink` yields the interactive tier; attaching `hdf5_sink`
yields the batch tier; attaching several at once records a live session.

### 5.3 The renderer boundary (Principle 9)

`geometry/` and `analysis/` produce physical quantities. Only
`scene_description.py` turns them into drawable primitives, and only
`vedo_renderer.py` knows about vedo or VTK. Should a browser-delivered
backend become preferable, it is a new module in `render/` and nothing
else changes. This matters on a cluster: see §9.3.

### 5.4 The kernel boundary (Python now, compiled later)

Two places will dominate cost once Future Direction 1 is enabled: the
integrator inner loop in `integrators.py`, and the density-field
integration in `numerical_inertia.py`. Both are therefore isolated
behind narrow interfaces with array-shaped inputs and outputs, so a
compiled implementation (Fortran, C++, or a just-in-time compiler) can
replace either one without a caller changing. No other module should
grow a numerically hot inner loop; if one does, that is a signal to
move the work into these two.

### 5.5 The units boundary (Principle 11)

Physical quantities carry explicit units at the edges of the system and
are bare SI floats everywhere inside it. The conversion happens exactly
once, when a scenario is loaded, and once more in reverse when a value
is displayed.

`core/units.py` is the **only** module permitted to import the units
library. It exposes two operations: parse a quantity (a string such as
`"10 kg"` or `"0.5 kg*m^2"`, or a number plus a unit) into an SI float,
and format an SI float back into a human-readable quantity for display.

The library is **pint**. It was chosen over `astropy.units` (which would
drag in an entire astronomy stack), `unyt` (technically fine but far
less widely known), and `numericalunits` (no real quantity objects and
cryptic failures) for three reasons specific to this project:

1. Its weakness is per-operation overhead, and by construction that
   overhead occurs only at the boundary, never in the integrator.
2. It produces the clearest error messages of any option. A student who
   writes `kg/m^2` where `kg*m^2` was meant is told what is wrong, which
   is worth more here than in most software.
3. It parses unit strings directly, so scenario files can carry
   human-readable quantities rather than naked numbers whose units live
   only in a comment.

Below `scenario/`, no module imports pint, accepts a pint object, or
returns one. Everything in `dynamics/`, `body/`, `analysis/`, and
`geometry/` speaks bare SI floats and documents the unit in the name or
docstring. This is enforced by a test (§8.6).

---

## 6. The Simulation Loop

### 6.1 Ownership: the simulation drives, single-threaded

The interactive tier runs its own loop, which steps the physics, updates
the scene, asks the renderer to draw, and then pumps the windowing
system's event queue. It does **not** hand control to VTK's interactor
and hang physics off a timer callback.

The reason is uniformity with Tier 2. The batch tier's loop is the same
loop with a different sink and no event pumping, so one mental model
covers both, and `simulation_engine.py` is shared rather than
reimplemented per tier.

The loop is **single-threaded**. Running physics on a worker thread was
considered and rejected for now: VTK is not thread-safe, so rendering
would be pinned to the main thread regardless; the locking would work
against Principle 7 (student-readable source); and nondeterministic
interleaving is in direct tension with the exact reproducibility that
Goal 11 promises. Should the future heavy-body case (Future Direction 1)
make a responsive UI impossible single-threaded, threading can be added
behind the sink boundary (§5.2) without disturbing the physics.

### 6.2 Pacing: fixed step, fixed substeps, no wall clock

The physics advances by a **fixed time step** taken from the scenario's
fidelity settings, and a **fixed number of substeps is taken per
rendered frame**. Elapsed wall-clock time is never consulted.

This is a deliberate departure from the accumulator pattern used in
interactive graphics generally, where elapsed real time is measured and
consumed in fixed chunks so that motion proceeds at a true real-time
rate regardless of machine speed. That trade is correct for a game and
wrong here, for two reasons:

1. **Reproducibility.** If the number of steps taken depends on how
   busy the machine was, two runs of one scenario diverge. That breaks
   Goal 11 and falsifies the §2 claim that a batch run is the run that
   was explored interactively.
2. **Honest drift.** Principle 2 puts the departure of the conserved
   quantities on screen. If the step count varied with frame rate, the
   displayed drift would vary with machine load — an artifact of the
   renderer masquerading as a property of the integrator, which is
   precisely the confusion Principle 2 exists to prevent.

The cost of this choice is that the animation does not run at a
guaranteed real-time rate: on a loaded machine it simply proceeds more
slowly in wall time. The physics is untouched. The ratio of simulated
time to elapsed time is displayed rather than silently corrected, so a
viewer always knows whether they are watching real time.

### 6.3 Time controls

The controls demanded by Goal 8 are expressed entirely as the number of
substeps taken per rendered frame:

| Control | Implementation |
| --- | --- |
| Pause | Zero substeps per frame; rendering continues |
| Single step | One substep, then return to zero |
| Slow motion | Fewer substeps per frame |
| Fast forward | More substeps per frame |
| Replay | Re-read stored states instead of stepping |

Note that none of these alters the time step itself. Changing `dt` would
change the trajectory, so a student slowing the motion down to watch a
Dzhanibekov flip sees exactly the same flip, not a differently
integrated one.

### 6.4 The determinism guarantee

Together, §6.1 and §6.2 yield a property worth stating plainly and
testing (§8.6):

> Given the same scenario, the computed trajectory is identical —
> bit for bit — regardless of tier, machine load, frame rate, or which
> time controls were exercised during the run.

Time controls change what is *shown* and when, never what is *computed*.

### 6.5 Trajectory retention

Replay requires history, and history costs memory. The retention policy
is a fidelity parameter (§2), not a hard-coded constant. The design
question of what to retain and what to do on overflow — ring buffer,
downsampling, or spilling to the sink — belongs to DESIGN. The
architectural constraint is only this: retention is bounded and
configured, never unbounded growth, and the retention limit is recorded
in the scenario so that a replay is reproducible.

---

## 7. Configuration

Two configuration mechanisms exist and they hold different kinds of
thing. Confusing them would undermine reproducibility, so the division
is stated here as a rule.

**The rc file** (`rbsimrc.py`, following the `XYZrc.py` idiom) holds what
is *machine-dependent and rarely changed*: filesystem paths, output
directories, default window size, preferred palette, cluster and queue
settings, and the default values of anything below. It is looked for in
the working directory, then in `$RIGID_BODY_RC`, and last in the package
itself (`rigid_body/defaults/`), which is the documented set of defaults
and is always present, in a clone and in an installed copy alike.
`rbsim --write-rc` and `rbbatch --write-rc` copy the shipped file into
the working directory for a user who wants to change it; nobody should
need to know where the package is installed.

**The scenario file** holds the *physics*: the body and its density, the
initial conditions, the torque models in force, the integrator and
fidelity settings actually used, and the viewpoint.

Precedence runs in one direction:

```
rc file defaults  <  scenario file  <  command-line arguments
```

From this follows the rule that matters most:

> **Any value that can affect the computed trajectory must live in the
> scenario, never only in the rc file.** The rc file may supply its
> default, but the scenario records the resolved value that was used.

Otherwise a scenario would produce different physics on different
machines, which would defeat both Goal 11 and the tier bridge of §2. A
scenario must be self-contained: handing the file to another user on
another machine must reproduce the same trajectory. The rc file governs
convenience and environment; it never governs physics.

---

## 8. Testing Strategy

### 8.1 Layers

| Directory | Scope |
| --- | --- |
| `tests/unit/` | Pure functions: inertia tensors, rotation |
| | conversions, unit round-trips |
| `tests/integration/` | Subsystems together: the engine with |
| | torques and an integrator over a few steps |
| `tests/regression/` | Whole scenarios against stored reference |
| | output |

### 8.2 Oracles

VISION Principle 3 makes analytically solvable cases the standard of
correctness rather than mere examples. The oracles are:

- The closed-form inertia tensor of each primitive solid in §3.2.
- Torque-free motion of a symmetric top, whose angular-velocity
  precession rate about the symmetry axis is known exactly.
- Rotation initialized exactly about a principal axis, which must
  remain about that axis.
- The steady-precession rate of the heavy symmetric top.
- The Poinsot invariants: `2T = omega . L`, and `|L|` constant under
  torque-free motion.

When `numerical_inertia.py` arrives, its first duty is to reproduce
`analytic_inertia.py` on the symmetric solids to within a stated
tolerance. That is the test which certifies the §5.1 boundary.

### 8.3 Invariants

Properties that must hold everywhere, asserted wherever the quantity is
produced:

- The inertia tensor is symmetric and positive-definite.
- The principal moments satisfy the triangle inequalities, `I1 + I2 >=
  I3` and permutations. A body violating these is not physical.
- Rotation matrices are orthogonal with determinant `+1`.
- Quaternions remain normalized.

### 8.4 Tolerance policy

Every numerical tolerance must be **derived and justified**, not tuned.
A tolerance is chosen from the integrator's order and step size, or from
the floating-point precision of the operation, and the reasoning is
recorded in a comment beside it.

The failure mode this exists to prevent is loosening a tolerance until a
failing test passes. That converts the suite from a check into a rubber
stamp, and it is especially dangerous here because Principle 2 permits
drift: a test that tolerates arbitrary drift is testing nothing. Where a
test bounds drift, the bound is stated as a rate (per unit simulated
time) so that it remains meaningful as run lengths change.

Drift is held to two different standards, according to regime. The
second is not a stricter version of the first but a different kind of
requirement, and it is important not to confuse them:

| Regime | Standard |
| --- | --- |
| Classroom demonstration | Relative energy drift below about |
| | 1e-6 over the run, expressed as a rate |
| | per unit simulated time |
| Long integration | Energy error *bounded* rather than |
| (VISION Future Direction 4) | secular: a structure-preserving |
| | integrator, not a tighter tolerance |

The distinction matters because the failure it guards against is
invisible to ordinary testing. A conventional integrator run over a
demonstration-length interval looks excellent and will pass any
tolerance test written for the first regime. The same integrator run
over the ten million rotations of a planetary precession accumulates a
steadily growing error that no reduction in step size removes, because
the error is secular rather than random. Tests for the second regime
must therefore check the *growth* of the error over a long run, not its
magnitude over a short one.

### 8.5 Reference-output governance

A file in `tests/regression/reference_outputs/` is a claim about correct
behavior, so it may only be created from a case that is either checked
against an oracle (§8.2) or verified by hand. Regenerating a reference
requires the commit message to say what changed and why the new values
are more correct than the old ones.

Reference outputs are stored in a **compact, diff-reviewable text
format**, not HDF5. Two reasons: a reviewer can see in a pull request
exactly which numbers moved, and the repository's `.gitignore` excludes
HDF5 as bulky derived output (§9.4). Regression references are small by
construction; anything large enough to need HDF5 is a batch result, not
a test fixture.

### 8.6 Architectural tests

Four structural properties are mechanically checkable and are therefore
tested rather than left to discipline:

1. **The import rule of §4.** Nothing under `dynamics/`, `body/`,
   `analysis/`, or `geometry/` imports from `render/`, `ui/`, or
   `sinks/`.
2. **The units boundary of §5.5.** No module outside `core/units.py`
   imports pint.
3. **The determinism guarantee of §6.4.** The same scenario run twice,
   and run under differing time-control sequences, yields identical
   trajectories.
4. **The installed-copy guarantee of §9.5.** Everything a run needs is
   inside the package: each rc file loads with the search restricted to
   the package, every example scenario is found through the package and
   loads, the console scripts named in `pyproject.toml` import, every
   third-party module the package imports is a declared dependency, and
   each script in `src/scripts/` is only a front. `scenarios/` and the
   packaged examples are the same files.

---

## 9. Build System

### 9.1 Language and dependencies

Python 3.10 or newer, with a NumPy-based numerical core.

| Dependency | Tested with | Purpose |
| --- | --- | --- |
| `numpy` | 2.2.6 | Arrays, linear algebra |
| `scipy` | 1.15.3 | Integrators, eigenvalue solvers |
| `vedo` | 2026.6.1 | Interactive 3D rendering |
| `vtk` | 9.6.2 | Rendering engine beneath vedo |
| `h5py` | 3.16.0 | HDF5 output for the batch tier |
| `matplotlib` | 3.10.9 | Auxiliary plots |
| `pint` | 0.24.4 | Units at the boundary (§5.5) |
| `tomli-w` | 1.2.0 | Writing TOML scenario files (DESIGN §11) |
| `pytest` | 9.1.1 | Test suite (the `test` extra) |

These are the versions pinned by the `physdemo` suite (§9.5), which is
the single statement of what the course tools need; `pyproject.toml`
repeats the subset this tool imports, with lower bounds no tighter than
the suite's. Useful beside the tool but not dependencies of it:
`ffmpeg` (video from saved frames, Goal 11) and ParaView (post-hoc
visualization of the HDF5/XDMF output).

Deliberately *not* dependencies yet: `numba` and `mpi4py` (both belong
to the deferred compiled-kernel work of §5.4), and any GUI toolkit
beyond what vedo provides.

*Reading* TOML scenarios needs no new dependency on Python 3.11+, where
`tomllib` is in the standard library; on the 3.10 floor the `tomli`
backport supplies it. Only *writing* needs `tomli-w`, which is why the
table lists the writer alone.

**No GPU is required.** Measurements in §9.3 established that software
rendering is sufficient, so neither a GPU allocation nor VirtualGL is a
prerequisite for the interactive tier.

### 9.2 Running

```bash
# Route A (§9.5): the physdemo suite.
sdemo          # alias for:  source <suite prefix>/activate.sh
               # (or, where Lmod is used:  module load cpg_physdemo)
# Route B (§9.5): a pip-installed copy.
source physdemo/bin/activate

# Then, identically on both routes:
rbsim --check                  # can this computer run and draw it?
rbsim dzhanibekov              # a packaged example, by bare name
rbsim --examples               # copy the example scenarios here
rbsim dzhanibekov.toml         # Tier 1: your own (edited) copy
rbsim --write-rc               # copy the rc defaults here, to edit
rbbatch dzhanibekov.toml -o dzhanibekov.h5     # Tier 2: batch run

# In a clone, from the repository root:
rbsim scenarios/dzhanibekov.toml
pytest tests/ -v
```

`rbsim --check` is the one-line answer to "will it work here": it
reports the versions in use, runs a short simulation, draws it
offscreen, and verifies that the picture is not blank.

**Offscreen drawing** (`--offscreen`, `--screenshot`, `--save-frames`,
`--check`, the tests) chooses VTK's window class by one rule, shared
with the other course tools and kept in `render/offscreen.py`: on
Linux, a request to draw offscreen selects VTK's EGL window class
before VTK is imported, whatever `DISPLAY` says, because VTK's X window
class hangs on a `DISPLAY` that is set but dead; a request for a window
leaves VTK alone; macOS and Windows need nothing and are left alone.

### 9.3 Cluster deployment and the rendering budget

*(Revised 2026-09-17. This section first said that both tiers run on
the teaching cluster rather than on student laptops. The class will not
use the cluster these measurements were made on: students will run on
their own laptops if that can be made near effortless, and otherwise on
a different teaching cluster with a read-only shared installation and
small home quotas; §9.5 serves both. The measurements below stand as
what they are — software rendering on one research cluster's nodes — and
remain the evidence that no GPU is needed.)*

**Rendering risk: retired by measurement.** A benchmark scene containing
everything the real tool draws — body, wireframe momental ellipsoid,
invariable plane, polhode and herpolhode traces, both coordinate frames,
vectors, and the telemetry overlay — was rendered offscreen and timed.
Every measurement below was checked by reading the framebuffer back and
confirming the scene actually appeared in it, because a render window
without a valid context accepts draw calls silently and reports a
gratifying but fictitious frame rate. The benchmark is kept at
`dev/spikes/vedo_fps_spike.py` so these numbers can be re-checked when
the hardware, the libraries, or the scene complexity change.

Software rendering, 8-core node in the `interactive` partition:

| Window | Ellipsoid res. | Trace points | Mean FPS |
| --- | --- | --- | --- |
| 1280 x 960 | 32 | 1 500 | **34.2** |
| 1280 x 960 | 32 | 8 000 | 32.2 |
| 1280 x 960 | 48 | 4 000 | 28.7 |
| 1600 x 1200 | 24 | 1 500 | 25.1 |
| 1600 x 1200 | 32 | 1 500 | 24.0 |
| 1920 x 1440 | 32 | 1 500 | 18.2 |
| 1920 x 1440 | 96 | 20 000 | 11.3 |

**The renderer is fill-rate bound, not geometry bound.** Raising the
trace from 1 500 to 8 000 points cost only 6% of the frame rate, and
coarsening the ellipsoid from 32 to 24 divisions bought only 4%. Window
area, by contrast, dominates everything: going from 1280 x 960 to
1920 x 1440 is 2.25x the pixels and costs nearly half the frame rate.

This yields a clear design budget. **Geometric detail is cheap and may
be spent freely** — long traces and a finely divided ellipsoid are
affordable. **Window area is the scarce resource** and is the first knob
to turn if the interactive tier feels sluggish. A default window near
1280 x 960 sustains roughly 34 fps, comfortably interactive.

**GPU rendering is available but not required.** For reference, the same
scene through headless EGL on an H100 node reached 188.7 fps at
1280 x 960 and 84.7 fps at 1920 x 1440 with 20 000 trace points — six to
seven times software. The project deliberately does *not* depend on it:
requiring a GPU allocation would put students in contention for scarce
nodes, and software rendering already meets Principle 4. Should a
demonstration ever need a very large display or a much heavier scene,
EGL is a measured and available escape hatch requiring no code change.

**Frame delivery: measured.** The numbers above measure render
*throughput* on the compute node, offscreen. How finished frames reach
the viewer was measured on 2026-09-17 with the suite's
`physdemo-check --onscreen` (a 960 x 720 window, a small test scene, so
the figures compare *paths* and are not comparable with the table
above). All three paths drew correctly, every one in software
(`llvmpipe`):

| How the display is reached | Frame rate |
| --- | --- |
| Open OnDemand desktop | 9.7 fps |
| `ssh -X` to a login node | 6.2 fps |
| Interactive job with `--x11` | 4.4 fps |

**The Open OnDemand desktop is the recommended path for students**: it
is the fastest, and it does not depend on the student's own X server.
X11 forwarding works from a client whose X server supports GLX, at a
lower frame rate, because each finished frame crosses the network
uncompressed. It has been seen to crash from some Windows clients
(MoTTY / PuTTY: `bad X server connection`), and those students should
use OnDemand. Delivery, not rendering, is now the slowest stage, which
strengthens the advice above: a smaller window is the first knob.
These are observations of one site; the suite keeps them in
`physdemo/site/hellbender/notes.md`, where another site's would go
beside them.

### 9.4 Data interchange

The batch tier writes HDF5 for numerical results, accompanied by an XDMF
descriptor so that ParaView can read the time series directly. HDF5 is
chosen because it is self-describing, handles large time series well, and
is already available on the cluster. The full scenario is embedded in the
output file's metadata so that any result can be traced back to the run
that produced it.

HDF5 output is excluded from version control as bulky derived data: the
scenario that generated it is tracked instead, which is smaller and more
useful (§8.5).

### 9.5 The two ways in

Students must not have to build a numerical environment by hand, and
nobody should have to recall a path to run a tool. The tool reaches a
user in two ways. They exist because the two intended places want
opposite things, and they run the same code (§3.9).

**Route A: the `physdemo` suite, for a shared computer.** The tool is
one member of the suite (`github.com/UMKC-CPG/physdemo`): a set of
course demonstration tools that share one Python environment and one
`bin/` directory of commands. One person installs the suite; everyone
else only sources its `activate.sh` (or loads its optional Lmod
module). Nothing is installed per user, which is what a teaching
cluster with small home quotas and a read-only shared directory
requires. Because the environment is shared, everyone runs identical
library versions. The tool is *linked*, never copied and never
pip-installed into the suite, so a clone stays live.

The rules this tool obeys so that the suite can link it:

1. Every entry point under `src/scripts/` begins with
   `#!/usr/bin/env python3` and is executable.
2. An entry point finds the package from its own **resolved** location
   (`Path(__file__).resolve()`), never from the working directory and
   never from the unresolved path, which would name the suite's link to
   the script rather than the file.
3. The shipped defaults are inside the package (§7), so a linked command
   needs no configuration step.

**Route B: `pip install`, for a personal computer.** A laptop has one
user, no shared directory, and quite possibly no `bash` (Windows), so
the suite's shell scripts are the wrong tool. There the tool installs
like any Python package, with its dependencies, into an environment the
user makes:

```
python -m venv physdemo
source physdemo/bin/activate        (Windows: physdemo\Scripts\activate)
pip install https://github.com/UMKC-CPG/rigid_body/archive/refs/heads/main.zip
rbsim --check
rbsim dzhanibekov
```

The archive URL needs no `git` on the laptop; a release is the same URL
with `refs/tags/<tag>`. `pip` creates `rbsim` and `rbbatch` from the
console scripts declared in `pyproject.toml`, on every operating system.
The same environment holds the other course tools: each is one more
`pip install` line.

This is why everything the tool needs at run time is inside the package
(§1), and why `pyproject.toml` **declares the dependencies**: on this
route nobody else will supply them. Route B installs the newest versions
that satisfy the bounds, so a breaking release of a dependency shows up
on a laptop first; the suite's pinned `requirements.txt` is the
known-good fallback (`pip install -r` it, then this tool with
`--no-deps`).

**Portability.** Nothing in this repository names a path on one
computer. What is specific to one site lives in the suite's `site/`
directory.

**What has been tried.** Route A on Linux with Python 3.10, including
from a read-only installation with an empty home directory. Route B on
Linux. Neither route has yet been run on macOS or Windows; the first
person to do so should run `rbsim --check` and report what it prints.

**History.** An earlier draft of this section described a per-project
mamba environment and a `cpg_rigidbody` modulefile. Neither was ever
created: through `v0.3` the tool ran from a virtual environment named
`rigid`, activated by hand. The suite replaced that, and Route B was
added when laptops became an intended place to run.

---

## 10. Development Checkpoints

The repository is under version control at
`git@github.com:UMKC-CPG/rigid_body.git`, with work on `main`.

Each level of the document chain gets a tagged baseline when it is first
considered complete, so that later drift can be measured against a fixed
point:

```
v0.1-vision          VISION.md complete
v0.2-architecture    ARCHITECTURE.md complete
v0.3-design          DESIGN.md complete
v0.4-pseudocode      PSEUDOCODE.md complete
v0.5-torque-free     First running interactive torque-free simulation
v0.6-poinsot         Poinsot construction rendered (Goal 1)
v0.7-dzhanibekov     Intermediate-axis instability demonstrated (Goal 2)
v0.8-heavy-top       Applied torques and precession (Goals 3, 10)
v1.0-classroom       Usable in a graduate mechanics course
```

Work proceeds on short-lived topic branches merged into the main line.
Tier 2 (batch and HDF5) is scheduled after `v1.0-classroom`, since its
abstractions — not its implementation — are what the earlier milestones
depend on.
