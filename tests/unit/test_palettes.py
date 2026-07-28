"""Unit tests for the palettes (render/palettes.py).

The palette is where the color-blind-safety promise (VISION Principle 6) is
kept, so the tests check the redundancy rule as an invariant rather than by
inspecting particular colors:

* Every palette encodes exactly the roles the scene description can emit --
  no drawable is left un-encodable.
* No meaningful distinction rests on color alone: any two roles sharing a
  color differ in a non-color channel, and no two roles share an identical
  encoding. The two traces and the two arrows differ by a *shape* channel
  in every scheme, and the two frames each own one consistent hue.
* select_palette resolves the built-in schemes and rejects an unknown name
  loudly; resolve_encoding rejects an unknown role.
"""

from itertools import combinations

import numpy as np
import pytest

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.analysis.conservation_monitor import ConservationMonitor
from rigid_body.render import scene_description as sd
from rigid_body.render import palettes as pal


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


ALL_PALETTES = list(pal.PALETTES.values())


# --------------------------------------------------------------------
# The palette covers exactly what the scene can emit
# --------------------------------------------------------------------

def test_palettes_cover_every_role_the_scene_emits():
    # Build a torque-free scene (which includes the Poinsot roles) and
    # confirm every role in it resolves in every palette -- nothing a scene
    # can draw is left without an encoding.
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    state = st.State(
        np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.3, 0.5, 2.0]))
    monitor = ConservationMonitor(state, body, [])
    scene = sd.build_scene(state, body, monitor)

    scene_roles = {drawable.role for drawable in scene.drawables}
    for palette in ALL_PALETTES:
        for role in scene_roles:
            # Must not raise.
            pal.resolve_encoding(palette, role)


def test_role_table_matches_the_scene_roles_exactly():
    # The palette's declared role set should be neither short of nor beyond
    # what the scene emits, so a new drawable cannot be forgotten here.
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    state = st.State(
        np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.3, 0.5, 2.0]))
    monitor = ConservationMonitor(state, body, [])
    scene = sd.build_scene(state, body, monitor)
    scene_roles = {drawable.role for drawable in scene.drawables}
    assert set(pal.ALL_ROLES) == scene_roles


# --------------------------------------------------------------------
# The redundancy rule (Section 14.3)
# --------------------------------------------------------------------

@pytest.mark.parametrize("palette", ALL_PALETTES, ids=lambda p: p.name)
def test_no_two_roles_share_an_identical_encoding(palette):
    for first_role, second_role in combinations(pal.ALL_ROLES, 2):
        assert (palette.encodings[first_role]
                != palette.encodings[second_role])


@pytest.mark.parametrize("palette", ALL_PALETTES, ids=lambda p: p.name)
def test_same_color_roles_differ_in_a_non_color_channel(palette):
    # The heart of Principle 6: if two roles share a hue, they must differ
    # in a channel that survives the loss of color.
    for first_role, second_role in combinations(pal.ALL_ROLES, 2):
        first = palette.encodings[first_role]
        second = palette.encodings[second_role]
        if first.color != second.color:
            continue
        differs_in_shape = (
            first.line_style != second.line_style
            or first.marker != second.marker
            or first.line_weight != second.line_weight
            or first.opacity != second.opacity)
        assert differs_in_shape, (
            f"{first_role} and {second_role} rest on color alone")


@pytest.mark.parametrize("palette", ALL_PALETTES, ids=lambda p: p.name)
def test_the_called_out_pairs_differ_by_a_shape_channel(palette):
    # Section 14.3 names these explicitly: the two traces differ by dash
    # pattern, and the two arrows by arrowhead -- in every scheme, so the
    # redundancy is never color's alone to carry.
    polhode = palette.encodings["polhode"]
    herpolhode = palette.encodings["herpolhode"]
    assert polhode.line_style != herpolhode.line_style

    omega = palette.encodings["angular_velocity"]
    momentum = palette.encodings["angular_momentum"]
    assert omega.marker != momentum.marker


# --------------------------------------------------------------------
# Frame coding: one consistent hue per frame (Section 14.4)
# --------------------------------------------------------------------

@pytest.mark.parametrize("palette", ALL_PALETTES, ids=lambda p: p.name)
def test_each_frame_owns_one_consistent_hue(palette):
    body_colors = {
        palette.encodings[role].color for role in pal.ALL_ROLES
        if pal.frame_family_of(role) == "body"}
    space_colors = {
        palette.encodings[role].color for role in pal.ALL_ROLES
        if pal.frame_family_of(role) == "space"}
    assert len(body_colors) == 1
    assert len(space_colors) == 1
    # And the two frames are told apart by hue.
    assert body_colors != space_colors


def test_frame_family_assignment_is_complete_and_expected():
    assert pal.frame_family_of("polhode") == "body"
    assert pal.frame_family_of("herpolhode") == "space"
    assert pal.frame_family_of("angular_velocity") == "body"
    assert pal.frame_family_of("angular_momentum") == "space"
    assert pal.frame_family_of("telemetry") == "neutral"


# --------------------------------------------------------------------
# Selection and lookup
# --------------------------------------------------------------------

def test_select_palette_resolves_the_builtins():
    for name in ("light", "dark", "color_blind_safe"):
        assert pal.select_palette(name).name == name


def test_select_palette_rejects_an_unknown_name():
    with pytest.raises(ValueError) as error:
        pal.select_palette("neon")
    # The error names the available schemes, not just the failure.
    assert "color_blind_safe" in str(error.value)


def test_resolve_encoding_rejects_an_unknown_role():
    with pytest.raises(ValueError):
        pal.resolve_encoding(pal.LIGHT_PALETTE, "not_a_role")


def test_switching_palette_preserves_the_shape_channels():
    # Principle 6's structural claim: only color changes between schemes;
    # the line style, weight, opacity, and marker of a role are invariant.
    for role in pal.ALL_ROLES:
        light = pal.resolve_encoding(pal.LIGHT_PALETTE, role)
        dark = pal.resolve_encoding(pal.DARK_PALETTE, role)
        safe = pal.resolve_encoding(
            pal.COLOR_BLIND_SAFE_PALETTE, role)
        for other in (dark, safe):
            assert other.line_style == light.line_style
            assert other.line_weight == light.line_weight
            assert other.opacity == light.opacity
            assert other.marker == light.marker
