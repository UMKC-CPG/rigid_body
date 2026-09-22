"""Verifies the window-class rule of physdemo PSEUDOCODE 4 (an
inherited module), without VTK."""

import sys

from rigid_body.render import offscreen


def _clean(monkeypatch, platform, **environment):
    monkeypatch.setattr(sys, 'platform', platform)
    for name in ('VTK_DEFAULT_OPENGL_WINDOW', 'DISPLAY',
                 'WAYLAND_DISPLAY'):
        monkeypatch.delenv(name, raising=False)
    for name, value in environment.items():
        monkeypatch.setenv(name, value)
    for module in ('vtk', 'vtkmodules'):
        monkeypatch.delitem(sys.modules, module, raising=False)


def test_linux_offscreen_ignores_a_stale_display(monkeypatch):
    _clean(monkeypatch, 'linux', DISPLAY='localhost:10.0')
    assert offscreen.prepare_offscreen() is True
    import os
    assert os.environ['VTK_DEFAULT_OPENGL_WINDOW'] == 'vtkEGLRenderWindow'
    assert offscreen.no_display_available() is False


def test_macos_and_windows_are_left_alone(monkeypatch):
    for platform in ('darwin', 'win32'):
        _clean(monkeypatch, platform)
        assert offscreen.prepare_offscreen() is False
        import os
        assert 'VTK_DEFAULT_OPENGL_WINDOW' not in os.environ
        assert offscreen.no_display_available() is False


def test_explicit_setting_wins(monkeypatch):
    _clean(monkeypatch, 'linux',
           VTK_DEFAULT_OPENGL_WINDOW='vtkXOpenGLRenderWindow')
    assert offscreen.prepare_offscreen() is False
    import os
    assert os.environ['VTK_DEFAULT_OPENGL_WINDOW'] == \
        'vtkXOpenGLRenderWindow'


def test_no_display_on_linux(monkeypatch):
    _clean(monkeypatch, 'linux')
    assert offscreen.no_display_available() is True
    _clean(monkeypatch, 'linux', WAYLAND_DISPLAY='wayland-0')
    assert offscreen.no_display_available() is False
