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
