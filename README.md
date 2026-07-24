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

**Early design.** The VISION and ARCHITECTURE documents are complete;
DESIGN, PSEUDOCODE, and the implementation are not yet written. There
is no runnable program in this repository yet.

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
dev/        Design document chain
src/
  rigid_body/   The importable library (physics and display)
  scripts/      Command-line entry points
tests/      Test suite (pytest)
```

## Running

The project targets Python 3.10+ with NumPy, SciPy, vedo, VTK, and
h5py. It is developed and run on a teaching cluster rather than on
student laptops, which keeps the environment uniform.

Rendering is done in **software**; no GPU is required. Measured frame
rates and the resulting design budget are recorded in
`dev/ARCHITECTURE.md`, section 9.3. The short version: geometric
detail is cheap, window area is the scarce resource.

```bash
pytest tests/ -v          # Test suite
```

## License

See the repository settings; not yet specified here.
