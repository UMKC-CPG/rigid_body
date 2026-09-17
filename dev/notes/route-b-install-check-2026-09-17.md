# Route B (pip install) end-to-end check

**Date:** 2026-09-17. **Specified by:** PSEUDOCODE Section 16.5,
"Verification", last item; ARCHITECTURE 8.6(4), 9.5.

The structural tests (`tests/unit/test_installed_copy.py`) cannot
prove that an installed copy works; only installing one can. This is
the record of doing so, and the recipe for doing it again.

## Recipe

In an empty directory OUTSIDE the repository, with an empty `HOME` and
a bare `PATH` so that nothing of the developer's environment leaks in:

```bash
python -m venv laptop && source laptop/bin/activate
pip install --no-cache-dir /path/to/rigid_body    # or the archive URL
mkdir work && cd work
rbsim --check
rbsim --examples
rbsim free_tumble.toml --offscreen --frames 2 --screenshot f.png
rbbatch symmetric_precession --no-xdmf
rbbatch --write-rc
```

## Result (Linux x86_64, Python 3.10.19)

- `pip` resolved and installed every dependency by itself: numpy
  2.2.6, scipy 1.15.3, matplotlib 3.10.9, vedo 2026.6.1, **vtk 9.7.0**,
  h5py 3.16.0, pint 0.24.4, tomli_w 1.2.0.
- vtk 9.7.0 is NEWER than the physdemo suite's pin (9.6.2). The tool
  works with it (ARCHITECTURE 9.5: Route B meets new releases first).
- `rbsim` and `rbbatch` were the venv's console scripts, and the
  package was imported from `site-packages`, not from the repository.
- `rbsim --check`: PASS (3 frames offscreen in 0.7 s).
- `--examples` wrote the three scenarios; the run from the copied file
  wrote `f.png`; the batch run by bare name wrote
  `symmetric_precession.h5`; `--write-rc` wrote `rbbatchrc.py`;
  `command` recorded the runs and none of the utility invocations.
- The same day, Route A was checked by name through the dev suite from
  an empty directory with a stale `DISPLAY` set: `--check` PASS, an
  offscreen run, and a batch run.

## Not yet done

macOS and Windows (`dev/TODO.md`, ARCHITECTURE 9.5). Nothing above
depends on a shell, but nobody has watched it run there.
