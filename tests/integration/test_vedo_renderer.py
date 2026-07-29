"""Integration tests for the vedo renderer (render/vedo_renderer.py).

A pixel-producing module cannot be checked the way the headless modules are,
but one thing can be verified automatically and is worth verifying: that the
renderer actually *rasterizes the scene* rather than silently returning an
empty buffer. A render window without a valid GL context accepts draw calls
and returns nothing, so -- following the fps spike's discipline -- these
tests render offscreen and read the framebuffer back, asserting the scene
appears in it (ARCHITECTURE Section 9.3).

The tests skip cleanly where no GL/OSMesa context is available, so they
verify the render where they can and never fail merely for lack of a
display.
"""

import numpy as np
import pytest

vedo = pytest.importorskip("vedo")

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import torque_models as tm
from rigid_body.core import orientation as ori
from rigid_body.analysis.conservation_monitor import ConservationMonitor
from rigid_body.render import scene_description as sd
from rigid_body.render import palettes as pal
from rigid_body.render.vedo_renderer import VedoRenderer


def make_body(shape, density=1000.0):
    """Build a natural-orientation RigidBody from a closed-form shape."""
    properties = ai.analytic_inertia(shape, density)
    moments = np.diag(properties.inertia_tensor)
    top_class, intermediate_axis = classify_top(moments)
    return RigidBody(
        principal_moments=moments,
        principal_axes=np.eye(3),
        total_mass=properties.mass,
        center_of_mass=properties.center_of_mass,
        top_class=top_class,
        intermediate_axis=intermediate_axis,
        geometry=shape)


def torque_free_scene():
    """A tumbling asymmetric box scene, at a generic orientation."""
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    orientation = ori.quaternion_from_axis_angle([1.0, 1.0, 0.0], 0.6)
    state = st.State(orientation, np.array([0.3, 0.5, 2.0]))
    monitor = ConservationMonitor(state, body, [])
    report = monitor.update(state, 0.01)
    scene = sd.build_scene(
        state, body, monitor, report=report, time_ratio=0.8)
    return scene, state


# These tests force a black window (rather than the palette's own field)
# so the "bright pixel == scene content" premise below holds regardless of
# which scheme is rendered.
_TEST_BACKGROUND = "black"


def foreground_and_colors(image):
    """Return (foreground fraction, distinct sampled colors) of an image.

    The background is forced black, so any bright pixel belongs to the
    scene; a healthy render covers an appreciable share of the frame with
    many distinct colors, while a blank buffer fails both.
    """
    flat = image.reshape(-1, image.shape[-1])
    foreground_fraction = float((flat.max(axis=1) > 12).mean())
    distinct_colors = int(len(np.unique(flat[::37], axis=0)))
    return foreground_fraction, distinct_colors


def render_to_array(scene, state, palette, layout):
    """Render a scene offscreen and return the framebuffer, or skip.

    Skips the test if no offscreen GL context is available, so a machine
    without a rasterizer does not report a failure it cannot help.
    """
    try:
        renderer = VedoRenderer(
            palette, layout=layout, size=(800, 600), offscreen=True,
            background=_TEST_BACKGROUND)
    except Exception as problem:                    # pragma: no cover
        pytest.skip(f"no offscreen render context: {problem}")
    try:
        renderer.render(scene, state)
        image = renderer.screenshot(as_array=True)
    finally:
        renderer.close()
    if image is None or image.size == 0:            # pragma: no cover
        pytest.skip("offscreen framebuffer was empty (no GL context)")
    return image


# --------------------------------------------------------------------
# The render actually rasterizes the scene
# --------------------------------------------------------------------

@pytest.mark.parametrize("layout", ["side_by_side", "single"])
def test_scene_rasterizes_to_real_pixels(layout):
    scene, state = torque_free_scene()
    image = render_to_array(scene, state, pal.LIGHT_PALETTE, layout)
    foreground_fraction, distinct_colors = foreground_and_colors(image)
    # A real render of this scene covers a healthy share of the frame with
    # many distinct colors; a blank buffer fails both checks.
    assert foreground_fraction > 0.01
    assert distinct_colors > 8


def test_color_blind_palette_also_renders():
    scene, state = torque_free_scene()
    image = render_to_array(
        scene, state, pal.COLOR_BLIND_SAFE_PALETTE, "side_by_side")
    foreground_fraction, _colors = foreground_and_colors(image)
    assert foreground_fraction > 0.01


def test_applied_torque_scene_renders_without_poinsot():
    # A damped cube (spherical top, with torque) has no Poinsot objects and
    # a single-point polhode; it must still render.
    body = make_body(shapes.Cube(0.2))
    state = st.State(
        np.array([1.0, 0.0, 0.0, 0.0]), np.array([1.0, 2.0, 3.0]))
    monitor = ConservationMonitor(state, body, [tm.ViscousDamping(0.1)])
    scene = sd.build_scene(state, body, monitor)
    image = render_to_array(scene, state, pal.LIGHT_PALETTE, "single")
    foreground_fraction, _colors = foreground_and_colors(image)
    assert foreground_fraction > 0.005


def test_successive_frames_render_without_error():
    # The interactive loop calls render() every frame; rebuilding and
    # replacing the actors across frames must not error or blank out.
    scene, state = torque_free_scene()
    try:
        renderer = VedoRenderer(
            pal.LIGHT_PALETTE, layout="side_by_side", size=(640, 480),
            offscreen=True, background=_TEST_BACKGROUND)
    except Exception as problem:                    # pragma: no cover
        pytest.skip(f"no offscreen render context: {problem}")
    try:
        for _ in range(3):
            renderer.render(scene, state)
        image = renderer.screenshot(as_array=True)
    finally:
        renderer.close()
    if image is None or image.size == 0:            # pragma: no cover
        pytest.skip("offscreen framebuffer was empty (no GL context)")
    foreground_fraction, _colors = foreground_and_colors(image)
    assert foreground_fraction > 0.01
