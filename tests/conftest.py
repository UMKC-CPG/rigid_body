"""Shared pytest fixtures and import-path setup (inherited from the
physdemo skeleton).

Putting `src/` on sys.path here means the test suite runs against the
working tree with no install step, which is what you want while
developing. An installed copy is still tested by running pytest from
outside the repository.
"""

import os
import sys

import pytest

# Resolve <repo>/src relative to this file so the suite works no matter
# which directory pytest was invoked from.
_source_root = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src'))

if _source_root not in sys.path:
    sys.path.insert(0, _source_root)

# The suite always draws offscreen. Choose VTK's window class now,
# before any test imports the renderer, so that a DISPLAY that is set
# but dead cannot hang a render test (physdemo contract C17).
from rigid_body.render.offscreen import prepare_offscreen  # noqa: E402

prepare_offscreen()


@pytest.fixture
def source_root():
    """Absolute path to the `src/` directory, for tests that need to
    locate a script or a data file rather than import a module."""
    return _source_root


@pytest.fixture
def run_directory(tmp_path, monkeypatch):
    """Run a test inside a fresh temporary working directory.

    Entry points write into the current directory (the `command` log,
    screenshots), so a test that exercises one must not run in the
    repository root. Depend on this fixture and the directory is
    created, switched to, and cleaned up automatically.
    """
    monkeypatch.chdir(tmp_path)
    return tmp_path


@pytest.fixture(scope='session')
def offscreen_context():
    """Skip the test unless this computer can draw offscreen. A blank
    framebuffer means a window that accepts calls and draws nothing,
    which is a property of the machine and not a failure of the
    tool (physdemo VISION P4)."""
    vedo = pytest.importorskip('vedo')
    try:
        plotter = vedo.Plotter(offscreen=True, size=(64, 48))
        plotter.add(vedo.Sphere(r=1.0).c('tomato'))
        plotter.show(interactive=False)
        image = plotter.screenshot(asarray=True)
        plotter.close()
    except Exception as problem:                 # noqa: BLE001
        pytest.skip(f'no offscreen render context: {problem}')
    if image.min() == image.max():
        pytest.skip('offscreen framebuffer was empty (no GL context)')
    return True
