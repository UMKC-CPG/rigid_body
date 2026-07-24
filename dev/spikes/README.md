# Spikes

Throwaway experiments kept because their *results* are cited elsewhere
in the document chain. A spike is not production code and is not held
to the architecture: it exists to answer one question, and it stays in
the repository only so that the answer can be re-checked when the
hardware, the libraries, or the scene complexity change.

---

## `vedo_fps_spike.py`

**Question it answered:** can software rendering sustain an interactive
frame rate for the scene this project actually draws, or is a GPU
required?

**Answer:** software suffices, and the renderer is fill-rate bound
rather than geometry bound. Results and the resulting design budget are
recorded in `ARCHITECTURE.md`, section 9.3.

It builds a scene deliberately representative of the real tool — rigid
body, wireframe momental ellipsoid, invariable plane, polhode and
herpolhode traces, both coordinate frames, vectors, and the telemetry
overlay — then animates and times it offscreen.

### The trap it guards against

An early run of this benchmark reported 37.5 fps while drawing
*nothing at all*. A VTK render window without a valid OpenGL context
accepts `Render()` calls and returns immediately, producing a
gratifying and completely fictitious frame rate.

Every measurement therefore reads the framebuffer back and verifies
that the scene is present, reporting `pixels_verified` alongside the
frame rate. **A result with `pixels_verified: false` means nothing.**
Use `--save-image` to check by eye as well.

### Re-running it

Rendering is offscreen, so no display is needed; `DISPLAY` must be
unset or the run will try to reach an X server and fail.

Software rendering on an interactive node, the supported configuration:

```bash
srun --partition=interactive --cpus-per-task=8 --mem=16G \
     --time=00:20:00 bash -c \
     'unset DISPLAY; python3 dev/spikes/vedo_fps_spike.py \
      --frames 100 --window-width 1280 --window-height 960 \
      --ellipsoid-resolution 32 --trace-points 1500 \
      --label software-modest'
```

GPU rendering through headless EGL, kept as a documented escape hatch:

```bash
srun --partition=gpu --gres=gpu:H100:1 --cpus-per-task=8 --mem=16G \
     --time=00:20:00 bash -c \
     'unset DISPLAY; VTK_DEFAULT_OPENGL_WINDOW=vtkEGLRenderWindow \
      python3 dev/spikes/vedo_fps_spike.py --frames 200 \
      --label gpu-egl'
```

Sweep `--window-width` / `--window-height` first when investigating
performance; window area dominates everything else.

---

## `platonic_inertia.py`

**Question it answered:** what are the inertia tensors of the five
Platonic solids, and are they really all isotropic?

**Answer:** all five are exactly isotropic — spherical tops — with the
constants now recorded in `DESIGN.md` §3.2.

It computes each tensor two independent ways: exactly, by decomposing
the polyhedron into tetrahedra and integrating the second moment over
each, and approximately, by Monte Carlo sampling of the interior. Two
methods are used deliberately, because an exact formula applied wrongly
is still wrong, and the two approaches fail in unrelated ways.

The check paid for itself immediately: a remembered value for the
icosahedron was wrong by exactly a factor of two and would otherwise
have gone into `DESIGN.md`, and from there into the code and its tests.

```bash
python3 dev/spikes/platonic_inertia.py
```

This script is also a prototype of the numerical inertia provider that
`DESIGN.md` §3.7 requires, whose first duty is to reproduce the closed
forms of §3.2. The tetrahedral-decomposition routine here is the method
that provider will use for mesh-defined bodies.

---

## `free_top_elliptic.py`

**Question it answered:** are the Jacobi-elliptic formulas for the
torque-free asymmetric top in `DESIGN.md` §8.4 correct as written, or was
a coefficient, the modulus, or the time scale garbled in transcription?

**Answer:** correct, on both branches. The analytic angular velocity
agrees with a high-accuracy integration of Euler's equations to about
`1e-13` — the integrator's own noise floor — and the analytic solution
conserves `2T` and `|L|^2` to `1e-14` on its own. The section's claim
that the §4.4 growth rate `sigma` is the separatrix's departure exponent
was confirmed to the same standard: a rate fitted from a near-intermediate-axis
run matches the closed form to `1e-4`.

§8.4 wrote its formulas down from memory and explicitly demanded this
check before the code trusts them. That demand is not ceremony: §3.2
records a remembered inertia constant that was wrong by exactly a factor
of two, and an elliptic solution offers far more places for such an error
to hide. Both branches are exercised — spin nearest the largest-moment
axis, and spin nearest the smallest, which §8.4 obtains by exchanging
axes 1 and 3.

```bash
python3 dev/spikes/free_top_elliptic.py
```

When `analytic_solutions.py` is written, this spike is the prototype of
its free-top oracle, and the verified formulas here are the ones that
module must reproduce.
