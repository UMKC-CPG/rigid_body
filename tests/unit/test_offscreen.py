"""Verifies the window-class rule of PSEUDOCODE Section 16.4, without VTK."""

import os
import sys

from rigid_body.render import offscreen


def _clean(monkeypatch, platform, **environment):
    monkeypatch.setattr(sys, "platform", platform)
    for name in ("VTK_DEFAULT_OPENGL_WINDOW", "DISPLAY", "WAYLAND_DISPLAY"):
        monkeypatch.delenv(name, raising=False)
    for name, value in environment.items():
        monkeypatch.setenv(name, value)
    for module in ("vtk", "vtkmodules"):
        monkeypatch.delitem(sys.modules, module, raising=False)


def test_linux_offscreen_ignores_a_stale_display(monkeypatch):
    _clean(monkeypatch, "linux", DISPLAY="localhost:10.0")
    assert offscreen.prepare_offscreen() is True
    assert os.environ["VTK_DEFAULT_OPENGL_WINDOW"] == "vtkEGLRenderWindow"


def test_macos_and_windows_are_left_alone(monkeypatch):
    for platform in ("darwin", "win32"):
        _clean(monkeypatch, platform)
        assert offscreen.prepare_offscreen() is False
        assert "VTK_DEFAULT_OPENGL_WINDOW" not in os.environ


def test_explicit_setting_wins(monkeypatch):
    _clean(monkeypatch, "linux",
           VTK_DEFAULT_OPENGL_WINDOW="vtkXOpenGLRenderWindow")
    assert offscreen.prepare_offscreen() is False
    assert os.environ["VTK_DEFAULT_OPENGL_WINDOW"] == \
        "vtkXOpenGLRenderWindow"
