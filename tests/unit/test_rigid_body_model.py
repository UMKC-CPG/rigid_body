"""Unit tests for body classification (body/rigid_body_model.py).

The classification is checked against solids whose type is known by
construction: every Platonic solid and the sphere are spherical tops, a
cylinder is a symmetric top, and a box with distinct edges is asymmetric
with a definite intermediate axis (DESIGN Sections 3.3 and 1.4).
"""

import numpy as np
import pytest

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import (
    classify_top, TopClass, validate_moments, validate_inertia_tensor,
    parallel_axis_shift, principal_frame_of, build_body_from_shape,
    build_body_from_moments)


def moments_of(shape, density=1000.0):
    return np.diag(ai.analytic_inertia(shape, density).inertia_tensor)


def test_cube_is_a_spherical_top():
    top_class, intermediate_axis = classify_top(moments_of(shapes.Cube(0.2)))
    assert top_class is TopClass.SPHERICAL
    assert intermediate_axis is None


def test_icosahedron_is_a_spherical_top():
    top_class, intermediate_axis = classify_top(
        moments_of(shapes.Icosahedron(0.2)))
    assert top_class is TopClass.SPHERICAL
    assert intermediate_axis is None


def test_cylinder_is_a_symmetric_top():
    top_class, intermediate_axis = classify_top(
        moments_of(shapes.Cylinder(0.4, 1.0)))
    assert top_class is TopClass.SYMMETRIC
    assert intermediate_axis is None


def test_box_with_distinct_edges_is_asymmetric():
    # Edges 0.2, 0.3, 0.5 give moments ordered I_1 > I_2 > I_3, so the
    # median moment sits on axis 2 -- the intermediate axis.
    top_class, intermediate_axis = classify_top(
        moments_of(shapes.Parallelepiped(0.2, 0.3, 0.5)))
    assert top_class is TopClass.ASYMMETRIC
    assert intermediate_axis == 2


def test_intermediate_axis_is_the_median_moment():
    # Moments (2, 4, 3): the median 3 sits on axis 3.
    top_class, intermediate_axis = classify_top(np.array([2.0, 4.0, 3.0]))
    assert top_class is TopClass.ASYMMETRIC
    assert intermediate_axis == 3


# --------------------------------------------------------------------
# Physical validity
# --------------------------------------------------------------------

def test_validate_moments_rejects_a_nonpositive_moment():
    with pytest.raises(ValueError):
        validate_moments(np.array([1.0, -0.5, 1.0]))


def test_validate_moments_rejects_a_triangle_violation():
    # 1 + 1 < 3 fails the triangle inequality.
    with pytest.raises(ValueError):
        validate_moments(np.array([1.0, 1.0, 3.0]))


def test_validate_moments_accepts_a_real_body():
    validate_moments(moments_of(shapes.Parallelepiped(0.2, 0.3, 0.5)))


def test_validate_inertia_tensor_rejects_an_asymmetric_matrix():
    asymmetric = np.array([[1.0, 0.2, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
    with pytest.raises(ValueError):
        validate_inertia_tensor(asymmetric)


# --------------------------------------------------------------------
# The parallel-axis shift and diagonalization
# --------------------------------------------------------------------

def test_parallel_axis_shift_along_an_axis():
    # Shifting by d along z raises the two transverse moments by M d^2 and
    # leaves the axial moment unchanged.
    inertia = np.diag([2.0, 3.0, 4.0])
    mass, distance = 5.0, 0.5
    shifted = parallel_axis_shift(inertia, mass, [0.0, 0.0, distance])
    expected = np.diag([
        2.0 + mass * distance**2,
        3.0 + mass * distance**2,
        4.0])
    np.testing.assert_allclose(shifted, expected, atol=1e-12)


def test_principal_frame_of_a_diagonal_tensor_takes_the_shortcut():
    frame = principal_frame_of(np.diag([2.0, 3.0, 4.0]))
    np.testing.assert_allclose(frame.moments, [2.0, 3.0, 4.0])
    np.testing.assert_allclose(frame.axes, np.eye(3))


def test_principal_frame_of_a_rotated_tensor_recovers_the_moments():
    # Rotate a diagonal tensor, then recover its eigenvalues and a proper
    # (right-handed) axis set.
    diagonal = np.diag([2.0, 3.0, 4.0])
    angle = 0.7
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    about_z = np.array([[cos_a, -sin_a, 0.0],
                        [sin_a, cos_a, 0.0],
                        [0.0, 0.0, 1.0]])
    rotated = about_z @ diagonal @ about_z.T

    frame = principal_frame_of(rotated)
    np.testing.assert_allclose(
        np.sort(frame.moments), [2.0, 3.0, 4.0], atol=1e-12)
    assert np.linalg.det(frame.axes) == pytest.approx(1.0)
    # The axes diagonalize the tensor back to the moments.
    reconstructed = frame.axes.T @ rotated @ frame.axes
    np.testing.assert_allclose(
        np.diag(reconstructed), frame.moments, atol=1e-12)


# --------------------------------------------------------------------
# The constructors
# --------------------------------------------------------------------

def test_build_from_shape_gives_a_classified_body():
    body = build_body_from_shape(shapes.Cube(0.2), 2700.0)
    assert body.top_class is TopClass.SPHERICAL
    assert body.total_mass == pytest.approx(2700.0 * 0.2**3)
    np.testing.assert_allclose(body.principal_axes, np.eye(3))
    assert body.geometry == shapes.Cube(0.2)


def test_build_from_shape_moments_match_the_provider():
    body = build_body_from_shape(shapes.Parallelepiped(0.2, 0.3, 0.5),
                                 1000.0)
    np.testing.assert_allclose(
        body.principal_moments,
        moments_of(shapes.Parallelepiped(0.2, 0.3, 0.5)))
    assert body.top_class is TopClass.ASYMMETRIC


def test_axial_pivot_preserves_diagonality_and_sets_the_lever_arm():
    # A cone pivoted along its symmetry axis stays a symmetric top, and the
    # stored center of mass is the pivot-to-center-of-mass lever arm.
    lever = np.array([0.0, 0.0, 0.25])
    body = build_body_from_shape(
        shapes.Cone(0.3, 1.0), 1000.0,
        pivot_from_center_of_mass=lever)
    assert body.top_class is TopClass.SYMMETRIC
    np.testing.assert_allclose(body.principal_axes, np.eye(3))
    np.testing.assert_allclose(body.center_of_mass, -lever)


def test_build_from_moments_has_no_geometry():
    body = build_body_from_moments([2.0, 3.0, 4.0], 1.5)
    np.testing.assert_allclose(body.principal_moments, [2.0, 3.0, 4.0])
    assert body.geometry is None
    assert body.top_class is TopClass.ASYMMETRIC
    assert body.total_mass == pytest.approx(1.5)
