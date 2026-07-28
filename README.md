# Rigid Body

An interactive, real-time teaching tool for **rigid-body rotation**,
built for a graduate theoretical-mechanics course. A student sets up a
body and its initial conditions, then watches the motion evolve live
alongside the conserved quantities that govern it.

The aim is to make the content of Euler's equations visible: the
torque-free tumbling of an asymmetric top, the intermediate-axis
(tennis-racket / Dzhanibekov) instability, and gyroscopic precession
and nutation — each connected back to the mathematics rather than
merely animated.

## Status

**Runnable.** The full five-level document chain is written and the
implementation is complete: both the interactive tier (a live vedo
window) and the batch tier (HDF5 output for ParaView) run from a saved
scenario, and the test suite passes. See **Running** below.

A few interactive-display refinements remain (per-panel camera framing
as the body tumbles, the herpolhode's swept trail, live scenario editing
through the window); these are tracked in `dev/TODO.md`. The physics,
the Poinsot geometry, the conservation monitor, and both entry points
are done and covered by tests.

## What it will do

- Render the full **Poinsot construction**: the momental ellipsoid
  rolling without slipping on the invariable plane, tracing the
  polhode on the ellipsoid and the herpolhode on the plane.
- Show the same motion in **both the body frame and the space frame**,
  since conflating the two is the most common conceptual error in the
  subject.
- Reproduce the **Dzhanibekov flip** on demand, with time controls
  (pause, step, slow motion, replay) slow enough to see how it happens.
- Report **conserved quantities live**, including the magnitude of
  numerical drift, so that a numerical artifact is never mistaken for
  a physical effect.
- Save a complete **scenario** — body, initial conditions, torques,
  and viewpoint — so a demonstration can be reproduced exactly.

## Documents

Development follows a five-level chain, each level citing the one
above it. All design documents live in `dev/`:

| Document | Question it answers |
| --- | --- |
| `dev/VISION.md` | Why does this project exist? |
| `dev/ARCHITECTURE.md` | How is it organized? |
| `dev/DESIGN.md` | How do the algorithms work? |
| `dev/PSEUDOCODE.md` | What are the steps, precisely? |
| `src/` | The implementation. |

`dev/TODO.md` tracks tasks by level. When a change is made at any
level, check upward (does this invalidate a parent claim?) and
downward (does this require child updates?) before committing.

## Layout

```
dev/          Design document chain
scenarios/    Ready-to-run example scenarios (TOML)
src/
  rigid_body/   The importable library (physics and display)
  scripts/      Command-line entry points (rbsim, rbbatch)
tests/        Test suite (pytest)
```

## Running

The project targets Python 3.10+ with NumPy, SciPy, vedo, VTK, and
h5py, and runs inside a dedicated virtual environment (`rigid`) on the
teaching cluster. Activate it first:

```bash
source /cluster/VAST/rulisp-lab/cpg/virtual_envs/rigid/bin/activate
cd /cluster/pixstor/home/rulisp/CPG/cpg-repo/rigid_body
```

### Interactive tier — a live window (`rbsim`)

Open a scenario in a live window and watch it tumble. This needs a
display (an X, VNC, or OnDemand desktop session — plain SSH X11
forwarding is not recommended, see `dev/ARCHITECTURE.md` §9.3):

```bash
python src/scripts/rbsim.py scenarios/dzhanibekov.toml
```

Controls: **space** pauses and resumes, **s** single-steps, **-** slows,
**+** speeds up, **n** returns to normal speed, **q** quits. Rendering is
in **software**; no GPU is required.

With no display, render a fixed number of frames offscreen (useful for a
preview or a headless node):

```bash
python src/scripts/rbsim.py scenarios/dzhanibekov.toml --offscreen --frames 60
```

### Batch tier — high-fidelity output for ParaView (`rbbatch`)

Run a scenario deterministically and write the full trajectory to HDF5
with an XDMF companion (and a live conservation-drift report):

```bash
python src/scripts/rbbatch.py scenarios/dzhanibekov.toml -o dzhanibekov.h5
```

The scenario is embedded in the output as provenance, so any result
traces back to the exact run that produced it, and the same scenario
reproduces the same trajectory bit-for-bit on any machine.

### Example scenarios

| File | What it shows |
| --- | --- |
| `scenarios/dzhanibekov.toml` | The intermediate-axis (tennis-racket) flip: an asymmetric box spun about its middle axis periodically flipping. |
| `scenarios/free_tumble.toml` | A generic torque-free tumble of the same asymmetric box, with a rich polhode and herpolhode. |
| `scenarios/symmetric_precession.toml` | A symmetric top (a cylinder) in steady precession: the polhode is a circle. |

A scenario is a plain TOML file describing the body, initial conditions,
torques, fidelity, and viewpoint; copy one and edit it to make your own.

### Tests

```bash
pytest tests/ -v
```

## License

See the repository settings; not yet specified here.
