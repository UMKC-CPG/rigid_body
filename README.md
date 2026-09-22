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
dev/            Design document chain
scenarios       Symbolic link to src/rigid_body/examples/
src/
  rigid_body/   The importable library (physics and display), plus
    cli/        the bodies of the commands (rbsim, rbbatch),
    defaults/   the shipped rc files, and
    examples/   ready-to-run example scenarios (TOML)
  scripts/      Thin executable fronts for cli/
tests/          Test suite (pytest)
pyproject.toml  Packaging, for `pip install`
```

## Installing

There are two ways, for two situations; they run the same code.

**On your own computer (Windows, macOS, or Linux).** You need Python
3.10 or later. Make an environment, install the tool into it, and
check that the computer can draw:

```bash
python -m venv physdemo
source physdemo/bin/activate        # Windows: physdemo\Scripts\activate
pip install https://github.com/UMKC-CPG/rigid_body/archive/refs/heads/main.zip
rbsim --check
```

`pip` fetches the numerical and graphics libraries (about 1 GB) and
creates the `rbsim` and `rbbatch` commands. No `git`, compiler, or GPU
is needed. In later sessions only the `activate` line is repeated. The
companion `scattering` tool installs into the same environment with one
more `pip install` line.

**On a shared computer (a teaching cluster).** The tool is one member
of the [`physdemo`](https://github.com/UMKC-CPG/physdemo) suite, which
one person installs for everybody: a single Python environment and a
directory of commands. Nothing is installed per user, which suits small
home directories and a read-only shared area. The instructor follows
the suite's README; a student only turns it on:

```bash
source /path/to/the/shared/physdemo/activate.sh
rbsim --check
```

## Running

```bash
rbsim dzhanibekov          # run a packaged example by name
rbsim --examples           # copy the example scenarios here, to edit
rbsim dzhanibekov.toml     # run your edited copy
rbsim --write-rc           # copy the window/palette defaults here
rbsim --help
```

Run from a directory you can write in: screenshots, HDF5 output, and
the `command` log go to the working directory. In a directory you
cannot write, the tool still runs and says what it could not save.

### Interactive tier — a live window (`rbsim`)

Open a scenario in a live window and watch it tumble:

```bash
rbsim dzhanibekov.toml
```

Controls: **space** pauses and resumes, **s** single-steps, **-** slows,
**+** speeds up, **n** returns to normal speed, **q** quits. Toggle what is
drawn to cut the clutter: **b** the body object, **e** the momental
ellipsoid and its construction, **v** the ω/L vectors, **t** the axis
triads. The full key reference is drawn in the window's bottom-left corner,
so nothing needs memorizing. Rendering is in **software**; no GPU is
required.

> **On the cluster, use an Open OnDemand (or VNC) desktop for a live
> window.** It is the fastest path and does not depend on your own X
> server. SSH X11 forwarding (`ssh -X`) also works from a client whose
> X server supports GLX, at a lower frame rate; from some Windows
> clients (MoTTY / PuTTY) it has crashed with `bad X server
> connection`. See `dev/ARCHITECTURE.md` §9.3 for the measurements. If
> you only have an SSH terminal, use the headless capture below.

### No display? Capture frames headlessly

With no desktop session, render offscreen and save images you can view
(or turn into a video). Save the final frame:

```bash
rbsim dzhanibekov.toml \
    --screenshot dzhanibekov.png --frames 60
```

or save every frame as an image sequence and assemble a video with
ffmpeg:

```bash
rbsim dzhanibekov.toml \
    --save-frames dz_frames/ --frames 300
ffmpeg -framerate 30 -i dz_frames/frame_%05d.png dzhanibekov.mp4
```

### Batch tier — high-fidelity output for ParaView (`rbbatch`)

Run a scenario deterministically and write the full trajectory to HDF5
with an XDMF companion (and a live conservation-drift report):

```bash
rbbatch dzhanibekov.toml -o dzhanibekov.h5
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
torques, fidelity, and viewpoint. `rbsim --examples` copies these three
into the working directory; edit one to make your own. Each can also be
run unedited by its bare name (`rbsim free_tumble`).

### Tests

```bash
pytest tests/ -v
```

## License

GPL-3.0-or-later; see `LICENSE`. If you build on this tool — by hand
or with an AI assistant — carry the attribution and the citations in
the design chain (`dev/`) forward.
