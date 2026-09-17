"""Choosing VTK's window class for offscreen drawing (PSEUDOCODE
Section 16.4; DESIGN Section 15.4). This module imports neither vtk nor vedo, because its one
function must run BEFORE either is imported: VTK fixes its window
class at import time from the environment variable
VTK_DEFAULT_OPENGL_WINDOW.

The rule is the physdemo suite's, and it is portable by construction:

- Offscreen requested, on Linux: ask for VTK's EGL window class,
  whatever DISPLAY says. A DISPLAY that is set but dead (a stale SSH
  forwarding) is common on clusters, and VTK's X window class hangs
  on it; so the decision keys on what was asked for, not on DISPLAY.
- On screen: leave VTK alone; its default window class is right.
- macOS and Windows: do nothing. Their default window classes draw
  offscreen unaided, and EGL does not exist there.

An explicit VTK_DEFAULT_OPENGL_WINDOW in the environment always wins.

Attribution: this module is part of the rigid_body teaching tool. It
is the same rule, and the same code, as `render/offscreen.py` in the
companion scattering tool, where it was first needed and verified.
"""

import os
import sys
import warnings

EGL_WINDOW_CLASS = 'vtkEGLRenderWindow'
_VARIABLE = 'VTK_DEFAULT_OPENGL_WINDOW'


def prepare_offscreen():
    """Arrange for VTK to draw offscreen. Call before anything imports
    vtk or vedo. Returns True if the EGL window class is now in
    effect because of this call or an identical earlier setting."""
    if not sys.platform.startswith('linux'):
        return False
    if 'vtkmodules' in sys.modules or 'vtk' in sys.modules:
        if os.environ.get(_VARIABLE) != EGL_WINDOW_CLASS:
            warnings.warn('VTK is already imported; its window class is '
                          'fixed and offscreen drawing may need a display')
        return os.environ.get(_VARIABLE) == EGL_WINDOW_CLASS
    os.environ.setdefault(_VARIABLE, EGL_WINDOW_CLASS)
    return os.environ[_VARIABLE] == EGL_WINDOW_CLASS


def no_display_available():
    """True on Linux when neither an X nor a Wayland display is named,
    so that no window could open at all."""
    return (sys.platform.startswith('linux')
            and not os.environ.get('DISPLAY')
            and not os.environ.get('WAYLAND_DISPLAY'))
