"""Unit tests for the scene description (render/scene_description.py).

The scene description is the renderer-agnostic middle stage: plain data,
no pixels. The tests pin the promises that make it trustworthy:

* Every drawable stands for a named quantity and carries a non-empty label
  (VISION Principle 5) -- nothing decorative is added.
* The Poinsot construction appears exactly when the motion is torque-free
  and is dropped the moment a torque is switched on (Section 10).
* Each drawable is anchored to the right panel and tagged with the frame
  its geometry lives in, so the renderer can re-express a shared vector.
* A body given by moments alone falls back to the ellipsoid proxy.
* The ellipsoid-scale note is stated and switchable (Section 14.5), and
  meaningful distinctions are carried redundantly by role and label, not
  color alone (Section 14.3).
* The telemetry overlay is frame-free and carries the drift report, the
  trend, the time ratio, and the Euler angles with degeneracy flagged.
* Building a scene never mutates the state it reads.
"""

import numpy as np
import pytest

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import (
    RigidBody, classify_top, build_body_from_moments)
from rigid_body.dynamics import state as st
from rigid_body.dynamics import torque_models as tm
from rigid_body.geometry.reference_frames import Frame
from rigid_body.analysis.conservation_monitor import (
    ConservationMonitor, TrendSummary)
from rigid_body.core.orientation import EulerAngles
from rigid_body.render import scene_description as sd


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])


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


ASYMMETRIC_BOX = shapes.Parallelepiped(0.2, 0.3, 0.5)
TUMBLING_OMEGA = np.array([0.3, 0.5, 2.0])


def torque_free_scene():
    """A scene for a freely tumbling asymmetric box, with a fresh monitor."""
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(state, body, [])
    report = monitor.update(state, 0.01)
    scene = sd.build_scene(
        state, body, monitor, report=report, time_ratio=0.8)
    return body, state, scene


def roles_of(scene):
    """Return the set of roles present in a scene."""
    return {drawable.role for drawable in scene.drawables}


def drawable_named(scene, role):
    """Return the single drawable with the given role."""
    matches = [d for d in scene.drawables if d.role == role]
    assert len(matches) == 1, f"expected one {role}, got {len(matches)}"
    return matches[0]


# --------------------------------------------------------------------
# Principle 5: everything is a named quantity with a label
# --------------------------------------------------------------------

def test_every_drawable_carries_a_nonempty_label():
    _body, _state, scene = torque_free_scene()
    assert scene.drawables  # the scene is not empty
    for drawable in scene.drawables:
        assert isinstance(drawable.label, str)
        assert drawable.label.strip() != ""
        assert drawable.role.strip() != ""


# --------------------------------------------------------------------
# The Poinsot construction is torque-gated (Section 10)
# --------------------------------------------------------------------

def test_torque_free_scene_includes_the_poinsot_construction():
    _body, _state, scene = torque_free_scene()
    roles = roles_of(scene)
    assert {"polhode", "invariable_plane", "herpolhode"} <= roles


def test_applied_torque_scene_drops_the_poinsot_construction():
    # The instant a torque is present, the Poinsot objects vanish -- they
    # exist only for torque-free motion.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(
        state, body, [tm.ViscousDamping(0.1)])
    scene = sd.build_scene(state, body, monitor)
    roles = roles_of(scene)
    assert not ({"polhode", "invariable_plane", "herpolhode"} & roles)
    # But the body, ellipsoid, arrows, triads, and overlay remain.
    assert {"body_mesh", "momental_ellipsoid", "angular_velocity",
            "angular_momentum", "telemetry"} <= roles


# --------------------------------------------------------------------
# Panels and coordinate frames (Section 11, 14.2)
# --------------------------------------------------------------------

def test_drawables_are_anchored_to_the_right_panel_and_frame():
    # Body-anchored drawables appear in BOTH panels (Section 11.4): still in
    # the body view, rolling in the space view. Space-anchored references
    # stay in the space panel; the overlay belongs to neither.
    _body, _state, scene = torque_free_scene()
    expectations = {
        "body_mesh": (sd.DrawablePanel.BOTH, Frame.BODY),
        "momental_ellipsoid": (sd.DrawablePanel.BOTH, Frame.BODY),
        "polhode": (sd.DrawablePanel.BOTH, Frame.BODY),
        "invariable_plane": (sd.DrawablePanel.SPACE, Frame.SPACE),
        "herpolhode": (sd.DrawablePanel.SPACE, Frame.SPACE),
        "angular_velocity": (sd.DrawablePanel.BOTH, Frame.BODY),
        "angular_momentum": (sd.DrawablePanel.BOTH, Frame.SPACE),
        "body_triad": (sd.DrawablePanel.BOTH, Frame.BODY),
        "lab_triad": (sd.DrawablePanel.SPACE, Frame.SPACE),
        "telemetry": (sd.DrawablePanel.NEITHER, None)}
    for role, (panel, frame) in expectations.items():
        drawable = drawable_named(scene, role)
        assert drawable.panel is panel
        assert drawable.coordinate_frame is frame


def test_shared_arrows_carry_their_native_geometry():
    # omega is stored in body components, L in space components, so the
    # renderer can re-express each into a panel with to_view.
    body, state, scene = torque_free_scene()
    omega = drawable_named(scene, "angular_velocity")
    np.testing.assert_array_equal(
        omega.geometry, state.angular_velocity_body)
    momentum = drawable_named(scene, "angular_momentum")
    np.testing.assert_array_equal(
        momentum.geometry, st.angular_momentum_space(state, body))


# --------------------------------------------------------------------
# Display layers filter what is drawn (Section 15.7)
# --------------------------------------------------------------------

def test_no_visible_layers_argument_shows_everything():
    # The default (and the batch tier) pass nothing and draw the full scene.
    _body, _state, scene = torque_free_scene()
    full = roles_of(scene)
    assert {"body_mesh", "momental_ellipsoid", "polhode",
            "angular_velocity", "body_triad", "telemetry"} <= full


def test_hiding_the_ellipsoid_layer_drops_its_construction():
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(state, body, [])
    scene = sd.build_scene(
        state, body, monitor,
        visible_layers=frozenset({"body", "vectors", "triads"}))
    roles = roles_of(scene)
    # The whole ellipsoid layer is gone...
    assert not ({"momental_ellipsoid", "polhode", "invariable_plane",
                 "herpolhode"} & roles)
    # ...while the other layers and the always-on overlay remain.
    assert {"body_mesh", "angular_velocity", "angular_momentum",
            "body_triad", "lab_triad", "telemetry"} <= roles


def test_hiding_the_body_layer_keeps_only_its_own_role_out():
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(state, body, [])
    scene = sd.build_scene(
        state, body, monitor,
        visible_layers=frozenset({"ellipsoid", "vectors", "triads"}))
    roles = roles_of(scene)
    assert "body_mesh" not in roles
    assert "momental_ellipsoid" in roles       # a different layer, still on


def test_the_telemetry_overlay_survives_every_layer_being_off():
    # The overlay is always-on chrome, not part of any toggleable layer.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(state, body, [])
    scene = sd.build_scene(
        state, body, monitor, visible_layers=frozenset())
    assert roles_of(scene) == {"telemetry"}


def test_the_role_layer_map_covers_every_drawable_but_the_overlay():
    # Every role a scene emits, except the always-on telemetry, maps to a
    # layer -- so nothing is left unreachable by a toggle.
    _body, _state, scene = torque_free_scene()
    for drawable in scene.drawables:
        if drawable.role == "telemetry":
            assert drawable.role not in sd.LAYER_OF_ROLE
        else:
            assert drawable.role in sd.LAYER_OF_ROLE


# --------------------------------------------------------------------
# The reference scale and the object's per-layer display scale (14.5)
# --------------------------------------------------------------------

def test_scene_reference_scale_is_the_ellipsoid_max_semi_axis():
    _body, _state, scene = torque_free_scene()
    ellipsoid = drawable_named(scene, "momental_ellipsoid")
    expected = float(np.max(ellipsoid.geometry.semi_axes))
    assert scene.reference_scale == pytest.approx(expected)
    assert scene.reference_scale > 0.0


def test_the_polhode_is_drawn_on_the_ellipsoid_surface():
    # Poinsot's point is that the contact point rho = omega/sqrt(2T) touches
    # the momental ellipsoid, so the drawn polhode should lie ON the surface
    # a viewer sees, not float outside it at the raw omega radius. Every
    # drawn point must satisfy the ellipsoid equation sum((x/semi)^2) = 1
    # (Sections 10.2, 10.4).
    _body, _state, scene = torque_free_scene()
    semi_axes = drawable_named(
        scene, "momental_ellipsoid").geometry.semi_axes
    polhode = drawable_named(scene, "polhode").geometry
    # The whole drawn loop lies on the surface...
    loop_residual = ((polhode.loop_points / semi_axes) ** 2).sum(axis=1)
    np.testing.assert_allclose(loop_residual, 1.0, atol=1e-9)
    # ...and so does the current contact point the trail head rides.
    head_residual = ((polhode.current_point / semi_axes) ** 2).sum()
    assert head_residual == pytest.approx(1.0, abs=1e-9)


def test_ellipsoid_detail_defaults_to_none_and_is_carried_through():
    # The detail level is draw-only presentation state: build_scene passes
    # whatever it is handed straight onto the scene for the renderer, and a
    # caller that does not choose one (the batch tier) leaves it None.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(state, body, [])
    default_scene = sd.build_scene(state, body, monitor)
    assert default_scene.ellipsoid_detail is None
    chosen_scene = sd.build_scene(
        state, body, monitor, ellipsoid_detail=4)
    assert chosen_scene.ellipsoid_detail == 4


def test_reference_scale_survives_hiding_the_ellipsoid_layer():
    # The scene carries the reference size so it stays fixed even when the
    # ellipsoid drawable is filtered out -- otherwise the object and arrows
    # would jump in size the moment the ellipsoid is toggled off.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(state, body, [])
    full = sd.build_scene(state, body, monitor)
    hidden = sd.build_scene(
        state, body, monitor,
        visible_layers=frozenset({"body", "vectors", "triads"}))
    assert "momental_ellipsoid" not in roles_of(hidden)
    assert hidden.reference_scale == pytest.approx(full.reference_scale)


def test_the_object_states_it_is_not_to_scale():
    # The object and the ellipsoid share no physical scale, so the object's
    # display size is a labeled choice (Principle 12): it says so.
    _body, _state, scene = torque_free_scene()
    body_mesh = drawable_named(scene, "body_mesh")
    assert body_mesh.scale_note is not None
    assert "scale" in body_mesh.scale_note.lower()


def test_active_scale_notes_lists_the_shown_scaled_quantities():
    # Both the object and the ellipsoid are drawn scaled, so the footnote
    # the renderer shows names both (Principle 12, surfaced on screen).
    _body, _state, scene = torque_free_scene()
    notes = sd.active_scale_notes(scene)
    assert any("not to scale" in note for note in notes)
    assert any("ellipsoid scaled" in note for note in notes)


def test_hiding_a_layer_drops_its_scale_note_from_the_footnote():
    # Toggling the object off must take its "not to scale" note with it, so
    # the footnote never claims something is scaled that is not on screen.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(state, body, [])
    scene = sd.build_scene(
        state, body, monitor,
        visible_layers=frozenset({"ellipsoid", "vectors", "triads"}))
    notes = sd.active_scale_notes(scene)
    assert not any("not to scale" in note for note in notes)
    assert any("ellipsoid scaled" in note for note in notes)


# --------------------------------------------------------------------
# A moments-only body falls back to the ellipsoid proxy (Section 4)
# --------------------------------------------------------------------

def test_moments_only_body_has_no_mesh_but_keeps_the_ellipsoid():
    # A body specified by moments alone has no geometry to draw, so the
    # momental ellipsoid is its visual proxy (the Earth of Goal 12).
    body = build_body_from_moments(
        principal_moments=np.array([2.0, 3.0, 4.0]),
        total_mass=5.0, center_of_mass=np.zeros(3))
    assert body.geometry is None
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(state, body, [])
    scene = sd.build_scene(state, body, monitor)
    roles = roles_of(scene)
    assert "body_mesh" not in roles
    assert "momental_ellipsoid" in roles


# --------------------------------------------------------------------
# Labeled scale choices and redundant encoding (Section 14.3, 14.5)
# --------------------------------------------------------------------

def test_ellipsoid_scale_label_defaults_to_inertia_and_switches():
    assert "inertia" in sd.ellipsoid_scale_label()

    class _Presentation:
        ellipsoid_scale = "energy"

    assert "energy" in sd.ellipsoid_scale_label(_Presentation())


def test_ellipsoid_drawable_states_its_scale():
    _body, _state, scene = torque_free_scene()
    ellipsoid = drawable_named(scene, "momental_ellipsoid")
    assert ellipsoid.scale_note is not None
    assert "inertia" in ellipsoid.scale_note


def test_meaningful_distinctions_are_redundant_not_color_only():
    # The polhode and herpolhode differ by role and label; the two triads
    # differ by label (1, 2, 3 vs X, Y, Z). Nothing rests on color alone.
    _body, _state, scene = torque_free_scene()
    polhode = drawable_named(scene, "polhode")
    herpolhode = drawable_named(scene, "herpolhode")
    assert polhode.role != herpolhode.role
    assert polhode.label != herpolhode.label

    body_triad = drawable_named(scene, "body_triad")
    lab_triad = drawable_named(scene, "lab_triad")
    assert body_triad.label != lab_triad.label
    assert "1, 2, 3" in body_triad.label
    assert "X, Y, Z" in lab_triad.label


# --------------------------------------------------------------------
# The telemetry overlay (Section 14.6)
# --------------------------------------------------------------------

def test_telemetry_overlay_is_frame_free_and_complete():
    _body, _state, scene = torque_free_scene()
    overlay = drawable_named(scene, "telemetry")
    assert overlay.panel is sd.DrawablePanel.NEITHER
    assert overlay.coordinate_frame is None

    readout = overlay.geometry
    assert isinstance(readout, sd.TelemetryReadout)
    # The instantaneous drift report is passed through.
    assert readout.report is not None
    assert hasattr(readout.report, "energy_drift_rate")
    # The secular trend and the display-only time ratio.
    assert isinstance(readout.trend, TrendSummary)
    assert readout.time_ratio == 0.8
    # The Euler angles with the degeneracy flag (identity is at a pole).
    assert isinstance(readout.euler_angles, EulerAngles)
    assert readout.euler_angles.is_degenerate is True


def test_overlay_report_is_none_before_the_first_step():
    # With no report supplied, the overlay still builds; the instantaneous
    # drift is simply absent until the first monitor update.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, TUMBLING_OMEGA)
    monitor = ConservationMonitor(state, body, [])
    scene = sd.build_scene(state, body, monitor)
    overlay = drawable_named(scene, "telemetry")
    assert overlay.geometry.report is None


# --------------------------------------------------------------------
# The herpolhode geometry, and purity
# --------------------------------------------------------------------

def test_herpolhode_geometry_point_lies_in_the_invariable_plane():
    body, state, scene = torque_free_scene()
    herpolhode = drawable_named(scene, "herpolhode").geometry
    assert isinstance(herpolhode, sd.HerpolhodeGeometry)
    assert herpolhode.inner_radius <= herpolhode.outer_radius
    height = float(np.dot(
        herpolhode.current_point, herpolhode.plane_normal))
    assert height == pytest.approx(herpolhode.plane_distance, abs=1e-9)


def test_build_scene_does_not_mutate_the_state():
    body = make_body(ASYMMETRIC_BOX)
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    omega = TUMBLING_OMEGA.copy()
    state = st.State(quaternion.copy(), omega.copy())
    monitor = ConservationMonitor(state, body, [])
    sd.build_scene(state, body, monitor)
    np.testing.assert_array_equal(
        state.body_to_space_quaternion, quaternion)
    np.testing.assert_array_equal(state.angular_velocity_body, omega)
