"""Unit tests for the orientation mathematics (core/orientation.py).

These exercise the quaternion algebra, the sandwich rotation, the
orientation matrix, the kinematic derivative, the Euler and axis-angle
builders, and the Euler read-out with its gimbal-lock degeneracy. The
oracles are exact identities of rigid-body rotation, so any deviation is
a real error rather than an approximation (ARCHITECTURE Section 8.2).
"""

import math

import numpy as np
import pytest

from rigid_body.core import orientation as ori


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])


# --------------------------------------------------------------------
# Quaternion algebra
# --------------------------------------------------------------------

def test_multiplying_by_identity_returns_the_original():
    quaternion = ori.normalize_quaternion(np.array([0.3, -0.5, 0.2, 0.8]))
    left_product = ori.quaternion_multiply(IDENTITY_QUATERNION, quaternion)
    right_product = ori.quaternion_multiply(quaternion, IDENTITY_QUATERNION)
    np.testing.assert_allclose(left_product, quaternion, atol=1e-12)
    np.testing.assert_allclose(right_product, quaternion, atol=1e-12)


def test_multiplication_is_not_commutative():
    # Two rotations about different axes do not commute; the product
    # depends on the order (DESIGN Section 2.2).
    first = ori.rotation_quaternion_about_z(0.7)
    second = ori.rotation_quaternion_about_x(0.5)
    forward = ori.quaternion_multiply(first, second)
    backward = ori.quaternion_multiply(second, first)
    assert not np.allclose(forward, backward)


def test_conjugate_undoes_a_rotation():
    quaternion = ori.rotation_quaternion_about_z(1.1)
    product = ori.quaternion_multiply(
        quaternion, ori.quaternion_conjugate(quaternion))
    np.testing.assert_allclose(product, IDENTITY_QUATERNION, atol=1e-12)


def test_normalize_gives_unit_norm_and_keeps_the_sign():
    raw = np.array([-2.0, 0.0, 0.0, 0.0])
    unit = ori.normalize_quaternion(raw)
    assert np.linalg.norm(unit) == pytest.approx(1.0)
    # The sign must be left alone: -2 -> -1, never flipped to +1, so the
    # double cover (DESIGN Section 2.2) is undisturbed.
    assert unit[0] < 0.0


# --------------------------------------------------------------------
# Rotating a vector, and the orientation matrix
# --------------------------------------------------------------------

def test_identity_rotation_leaves_a_vector_unchanged():
    vector = np.array([1.0, 2.0, 3.0])
    rotated = ori.rotate_body_to_space(IDENTITY_QUATERNION, vector)
    np.testing.assert_allclose(rotated, vector, atol=1e-12)


def test_quarter_turn_about_z_sends_x_to_y():
    # An active +90 degree rotation about z carries the x axis onto the
    # y axis.
    quaternion = ori.rotation_quaternion_about_z(math.pi / 2.0)
    rotated = ori.rotate_body_to_space(quaternion, np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(rotated, [0.0, 1.0, 0.0], atol=1e-12)


def test_space_to_body_inverts_body_to_space():
    quaternion = ori.normalize_quaternion(np.array([0.5, 0.5, -0.3, 0.7]))
    vector = np.array([0.2, -1.4, 0.9])
    there = ori.rotate_body_to_space(quaternion, vector)
    back = ori.rotate_space_to_body(quaternion, there)
    np.testing.assert_allclose(back, vector, atol=1e-12)


def test_matrix_agrees_with_the_sandwich_product():
    quaternion = ori.normalize_quaternion(np.array([0.6, -0.2, 0.5, 0.4]))
    matrix = ori.quaternion_to_matrix(quaternion)
    for vector in np.eye(3):
        by_matrix = matrix @ vector
        by_sandwich = ori.rotate_body_to_space(quaternion, vector)
        np.testing.assert_allclose(by_matrix, by_sandwich, atol=1e-12)


def test_orientation_matrix_is_a_proper_rotation():
    quaternion = ori.normalize_quaternion(np.array([0.1, 0.7, -0.4, 0.5]))
    matrix = ori.quaternion_to_matrix(quaternion)
    # Orthogonal with determinant +1: a rotation, not a reflection.
    np.testing.assert_allclose(
        matrix @ matrix.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(matrix) == pytest.approx(1.0)


# --------------------------------------------------------------------
# The orientation derivative
# --------------------------------------------------------------------

def test_orientation_derivative_of_spin_about_z():
    # With the identity orientation and a pure spin about z, only the
    # z component of q_dot is nonzero, at half the spin rate (DESIGN 2.4).
    spin_rate = 4.0
    q_dot = ori.orientation_derivative(
        IDENTITY_QUATERNION, np.array([0.0, 0.0, spin_rate]))
    np.testing.assert_allclose(
        q_dot, [0.0, 0.0, 0.0, 0.5 * spin_rate], atol=1e-12)


# --------------------------------------------------------------------
# Building quaternions from authoring input
# --------------------------------------------------------------------

def test_axis_angle_about_z_matches_the_z_builder():
    angle = 0.9
    from_axis = ori.quaternion_from_axis_angle([0.0, 0.0, 1.0], angle)
    from_builder = ori.rotation_quaternion_about_z(angle)
    np.testing.assert_allclose(from_axis, from_builder, atol=1e-12)


def test_axis_angle_ignores_the_axis_length():
    angle = 0.6
    unit = ori.quaternion_from_axis_angle([0.0, 1.0, 0.0], angle)
    scaled = ori.quaternion_from_axis_angle([0.0, 5.0, 0.0], angle)
    np.testing.assert_allclose(unit, scaled, atol=1e-12)


# --------------------------------------------------------------------
# Euler angles as an output
# --------------------------------------------------------------------

@pytest.mark.parametrize("precession, nutation, spin", [
    (0.3, 0.7, 1.1),
    (-0.4, 1.2, 0.8),
    (1.5, 0.9, -1.0),
])
def test_euler_round_trip_in_the_generic_case(precession, nutation, spin):
    quaternion = ori.quaternion_from_euler_zxz(precession, nutation, spin)
    recovered = ori.euler_angles_from_quaternion(quaternion)
    assert not recovered.is_degenerate
    assert recovered.precession_angle == pytest.approx(precession, abs=1e-9)
    assert recovered.nutation_angle == pytest.approx(nutation, abs=1e-9)
    assert recovered.spin_angle == pytest.approx(spin, abs=1e-9)


def test_gimbal_lock_at_zero_nutation_reports_the_sum():
    # theta = 0: only phi + psi is meaningful (DESIGN Sections 1.3, 2.6).
    precession, spin = 0.3, 1.1
    quaternion = ori.quaternion_from_euler_zxz(precession, 0.0, spin)
    recovered = ori.euler_angles_from_quaternion(quaternion)
    assert recovered.is_degenerate
    assert recovered.spin_angle is None
    assert recovered.nutation_angle == pytest.approx(0.0, abs=1e-7)
    assert recovered.precession_angle == pytest.approx(
        precession + spin, abs=1e-7)


def test_gimbal_lock_at_pi_nutation_reports_the_difference():
    # theta = pi: only phi - psi is meaningful (the sign of cos(theta)).
    precession, spin = 0.3, 1.1
    quaternion = ori.quaternion_from_euler_zxz(precession, math.pi, spin)
    recovered = ori.euler_angles_from_quaternion(quaternion)
    assert recovered.is_degenerate
    assert recovered.spin_angle is None
    assert recovered.nutation_angle == pytest.approx(math.pi, abs=1e-7)
    assert recovered.precession_angle == pytest.approx(
        precession - spin, abs=1e-7)
