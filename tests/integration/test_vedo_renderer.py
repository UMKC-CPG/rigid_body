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

from collections import deque

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
from rigid_body.render.vedo_renderer import (
    VedoRenderer, _ellipsoid_ring_points, _ellipsoid_ring_counts,
    _scale_to_display_size, _ELLIPSOID_DEFAULT_DETAIL,
    _BODY_DISPLAY_FRACTION, _updated_trail, _rotated_trail,
    _orthonormal_basis, _circle_points, _TRAIL_WINDOW)
from rigid_body.dynamics.time_control import (
    ELLIPSOID_DETAIL_MIN, ELLIPSOID_DETAIL_MAX)


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
# The ellipsoid ring cage and the object's display scale (pure geometry,
# no GL context needed)
# --------------------------------------------------------------------

def test_ellipsoid_rings_lie_on_the_surface_and_split_by_kind():
    semi = np.array([2.0, 3.0, 1.5])
    parallels, meridians = _ellipsoid_ring_counts(None)  # default look
    rings = list(_ellipsoid_ring_points(semi, parallels, meridians))
    closed_flags = [closed for _points, closed in rings]
    # Parallels are closed circles; meridians are open pole-to-pole arcs.
    assert closed_flags.count(True) == parallels
    assert closed_flags.count(False) == meridians
    # Every ring point lies on the ellipsoid: (x/a)^2 + (y/b)^2 + (z/c)^2 = 1.
    for points, _closed in rings:
        residual = ((points / semi) ** 2).sum(axis=1)
        np.testing.assert_allclose(residual, 1.0, atol=1e-9)


def test_mesh_detail_level_maps_to_a_monotone_ring_count():
    # The default level (or None) reproduces the look the tool shipped with:
    # five parallels and eight meridians.
    assert _ellipsoid_ring_counts(None) == _ellipsoid_ring_counts(
        _ELLIPSOID_DEFAULT_DETAIL)
    assert _ellipsoid_ring_counts(None) == (5, 8)
    # A finer level is a strictly denser cage than a coarser one, in both
    # ring families -- so the control visibly thins or thickens the mesh.
    coarse_parallels, coarse_meridians = _ellipsoid_ring_counts(
        ELLIPSOID_DETAIL_MIN)
    fine_parallels, fine_meridians = _ellipsoid_ring_counts(
        ELLIPSOID_DETAIL_MAX)
    assert fine_parallels > coarse_parallels
    assert fine_meridians > coarse_meridians
    # Even the sparsest cage keeps at least a couple of rings each way, so it
    # never collapses to an unreadable shape.
    assert coarse_parallels >= 2
    assert coarse_meridians >= 3


def test_the_body_mesh_scales_to_the_display_fraction():
    # A tiny box is enlarged so its largest half-extent is the display
    # fraction of the reference size, and its shape (edge ratios) is kept.
    box = vedo.Box(length=0.1, width=0.15, height=0.3)
    _scale_to_display_size(box, reference_scale=4.0)
    bounds = box.bounds()
    extents = (bounds[1] - bounds[0], bounds[3] - bounds[2],
               bounds[5] - bounds[4])
    half_extent = 0.5 * max(extents)
    assert half_extent == pytest.approx(_BODY_DISPLAY_FRACTION * 4.0)
    # Edge ratios are preserved (uniform scale): 0.1 : 0.15 : 0.3.
    np.testing.assert_allclose(
        np.array(extents) / max(extents), [1 / 3, 1 / 2, 1.0], atol=1e-6)


# --------------------------------------------------------------------
# The swept trails: accumulation, and the herpolhode band geometry
# (Section 10.5, pure -- no GL context needed)
# --------------------------------------------------------------------

def test_a_trail_accumulates_dedups_repeats_and_stays_bounded():
    trail = deque(maxlen=3)
    _updated_trail(trail, np.array([0.0, 0.0, 0.0]))
    # A repeated point (a paused or replayed frame) does not grow the trail.
    _updated_trail(trail, np.array([0.0, 0.0, 0.0]))
    assert len(trail) == 1
    # A genuinely new point extends it...
    _updated_trail(trail, np.array([1.0, 0.0, 0.0]))
    assert len(trail) == 2
    # ...and the window is bounded: old points age off the tail.
    for step in range(5):
        _updated_trail(trail, np.array([float(step), 1.0, 0.0]))
    assert len(trail) == 3


def test_the_trail_window_is_a_positive_bound():
    # The renderer keeps a bounded, most-recent stretch of the trail so the
    # herpolhode reads as a moving comet rather than a solid band.
    assert _TRAIL_WINDOW > 0


def test_a_rotated_trail_is_empty_for_no_history_and_rotates_otherwise():
    assert _rotated_trail(None, np.eye(3)).shape == (0, 3)
    assert _rotated_trail(deque(), np.eye(3)).shape == (0, 3)
    trail = deque([np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])])
    # A quarter turn about z sends x->y and y->-x; the whole trail rotates.
    quarter_turn = np.array([[0.0, -1.0, 0.0],
                             [1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0]])
    rotated = _rotated_trail(trail, quarter_turn)
    np.testing.assert_allclose(
        rotated, [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]], atol=1e-12)


def test_the_band_basis_is_orthonormal_and_in_the_plane():
    for normal in (np.array([0.0, 0.0, 1.0]),
                   np.array([1.0, 2.0, -2.0]),
                   np.array([0.9, 0.0, 0.1])):
        first_axis, second_axis = _orthonormal_basis(normal)
        unit_normal = normal / np.linalg.norm(normal)
        # Unit length, mutually perpendicular, and both spanning the plane.
        assert np.linalg.norm(first_axis) == pytest.approx(1.0)
        assert np.linalg.norm(second_axis) == pytest.approx(1.0)
        assert np.dot(first_axis, second_axis) == pytest.approx(0.0, abs=1e-12)
        assert np.dot(first_axis, unit_normal) == pytest.approx(0.0, abs=1e-12)
        assert np.dot(second_axis, unit_normal) == pytest.approx(
            0.0, abs=1e-12)


def test_a_band_circle_lies_in_the_plane_at_the_given_radius():
    normal = np.array([0.0, 0.0, 1.0])
    center = 2.0 * normal                      # height 2 above the origin
    first_axis, second_axis = _orthonormal_basis(normal)
    radius = 3.0
    circle = _circle_points(center, first_axis, second_axis, radius)
    # Every sample sits at the band radius from the centre...
    distances = np.linalg.norm(circle - center, axis=1)
    np.testing.assert_allclose(distances, radius, atol=1e-12)
    # ...and in the plane (constant height along the normal).
    np.testing.assert_allclose(circle @ normal, 2.0, atol=1e-12)


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


def test_swept_trails_accumulate_across_frames_then_reset_clears_them():
    # Rendering a torque-free scene populates the polhode and herpolhode
    # swept trails (Section 10.5); reset_trails -- which the driver calls at
    # each run start -- clears them, so a new run does not begin smeared with
    # the previous one's trace.
    scene, state = torque_free_scene()
    try:
        renderer = VedoRenderer(
            pal.LIGHT_PALETTE, layout="side_by_side", size=(320, 240),
            offscreen=True, background=_TEST_BACKGROUND)
    except Exception as problem:                    # pragma: no cover
        pytest.skip(f"no offscreen render context: {problem}")
    try:
        renderer.render(scene, state)
        assert len(renderer._trails.get("polhode", [])) >= 1
        assert len(renderer._trails.get("herpolhode", [])) >= 1
        renderer.reset_trails()
        assert renderer._trails == {}
    finally:
        renderer.close()


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
