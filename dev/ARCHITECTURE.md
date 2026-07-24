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
    scripts/          Command-line entry points (XYZ.py idiom)
  tests/              Test suite (unit, integration, regression)
  CLAUDE.md           AI assistant guidance
```

The library lives under `src/rigid_body/` and holds everything
reusable. The user-facing entry points live in `src/scripts/`, following
the existing `XYZ.py` / `XYZrc.py` convention: a settings class whose
defaults come from a resource-control file and are then reconciled with
command-line arguments.

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
which is what allows it to cross the tier boundary.

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

### 3.9 `scripts/` — Entry points

| Script | Purpose |
| --- | --- |
| `rbsim.py` | Launch the interactive simulation (Tier 1) |
| `rbsimrc.py` | Resource-control defaults for `rbsim.py` |
| `rbbatch.py` | Run a saved scenario as a batch job (Tier 2, future) |
| `rbbatchrc.py` | Resource-control defaults for `rbbatch.py` |

---

## 4. Dependency Graph

Dependencies point downward only. No module may import from a group
listed above it.

```
scripts/rbsim.py                    scripts/rbbatch.py
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
tier possible and satisfies Principle 9. It is worth enforcing with a
test.

---

## 5. Key Boundaries

Four seams in this architecture exist specifically to protect a VISION
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
else changes. This matters on a cluster: see §6.3.

### 5.4 The kernel boundary (Python now, compiled later)

Two places will dominate cost once Future Direction 1 is enabled: the
integrator inner loop in `integrators.py`, and the density-field
integration in `numerical_inertia.py`. Both are therefore isolated
behind narrow interfaces with array-shaped inputs and outputs, so a
compiled implementation (Fortran, C++, or a just-in-time compiler) can
replace either one without a caller changing. No other module should
grow a numerically hot inner loop; if one does, that is a signal to
move the work into these two.

---

## 6. Build System

### 6.1 Language and dependencies

Python 3.10 or newer, with a NumPy-based numerical core.

| Dependency | Status on cluster | Purpose |
| --- | --- | --- |
| `numpy` 2.2.6 | present | Arrays, linear algebra |
| `scipy` 1.15.3 | present | Integrators, eigenvalue solvers |
| `vedo` 2025.5.4 | present | Interactive 3D rendering |
| `vtk` 9.5.2 | present | Rendering engine beneath vedo |
| `h5py` 3.15.1 | present | HDF5 output for the batch tier |
| `matplotlib` 3.10.3 | present | Auxiliary plots |
| `pytest` | required | Test suite |
| `ffmpeg` 6.0 | module | Video export (Goal 11) |
| `paraview` 6.1.1 | module | Post-hoc visualization of HDF5/XDMF |
| `virtualgl` 3.1.4 | module | Optional GPU acceleration only (§6.3) |

Deliberately *not* dependencies yet: `numba` and `mpi4py` (absent on the
cluster; both belong to the deferred compiled-kernel work of §5.4), and
any GUI toolkit beyond what vedo provides.

**No GPU is required.** Measurements in §6.3 established that software
rendering is sufficient, so neither a GPU allocation nor VirtualGL is a
prerequisite for the interactive tier.

### 6.2 Running

```bash
# Tier 1: interactive exploration (see §6.3 for the cluster case).
python3 src/scripts/rbsim.py

# Tier 2: batch high-fidelity run from a saved scenario (future).
python3 src/scripts/rbbatch.py my_scenario.json

# Tests.
pytest tests/ -v
```

### 6.3 Cluster deployment and the rendering budget

Both the interactive and the batch tiers run on the teaching cluster
rather than on student laptops, which makes the environment controllable
and uniform.

**Rendering risk: retired by measurement.** A benchmark scene containing
everything the real tool draws — body, wireframe momental ellipsoid,
invariable plane, polhode and herpolhode traces, both coordinate frames,
vectors, and the telemetry overlay — was rendered offscreen and timed.
Every measurement below was checked by reading the framebuffer back and
confirming the scene actually appeared in it, because a render window
without a valid context accepts draw calls silently and reports a
gratifying but fictitious frame rate.

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

**Remaining open item: frame delivery.** The numbers above measure
render *throughput* on the compute node. How finished frames reach the
viewer — a VNC or OnDemand desktop session is the expected path — is a
separate and smaller question that has not yet been measured. Note that
plain SSH X11 forwarding is not the answer: it ships GL commands rather
than pixels and will disappoint regardless of how fast the node renders.

### 6.4 Data interchange

The batch tier writes HDF5 for numerical results, accompanied by an XDMF
descriptor so that ParaView can read the time series directly. HDF5 is
chosen because it is self-describing, handles large time series well, and
is already available on the cluster. The full scenario is embedded in the
output file's metadata so that any result can be traced back to the run
that produced it.

---

## 7. Development Checkpoints

The repository is **not yet under version control**; `git init` remains
to be done before the `/commit` and `/bigcommit` workflows can be used.

Once initialized, each level of the document chain gets a tagged baseline
when it is first considered complete, so that later drift can be measured
against a fixed point:

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
