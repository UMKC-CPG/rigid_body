"""Orientation mathematics: the layer every algorithm stands on.

This module is the code form of PSEUDOCODE Section 2, itself the
transcription of DESIGN Sections 1.3, 2.2, 2.4, and 2.6. The simulation
loop, the equations of motion, the conservation monitor, and the
renderer all rotate vectors or read out angles through the routines
here, so a convention fixed once is honored everywhere above.

Conventions, fixed here with care because they are the classic source of
silent error (DESIGN Section 2.2):

* **Scalar first.** A quaternion is the four numbers ``(w, x, y, z)``
  with the real part ``w`` first, stored as a length-4 NumPy array.
* **Body to space.** A unit quaternion ``q`` represents the rotation
  that carries a vector from body components to space components, by the
  sandwich product ``v_space = q * (0, v_body) * conjugate(q)``.
* **Unit norm.** Only unit quaternions represent rotations. This module
  never silently changes a quaternion's sign, because ``q`` and ``-q``
  are the same rotation (the double cover) and the determinism test of
  ARCHITECTURE Section 8.6 compares trajectories.

Every value here is a bare SI float in a NumPy array; nothing carries a
unit object (ARCHITECTURE Section 5.5).
"""

import math
from typing import NamedTuple, Optional

import numpy as np


# When the nutation angle drives ``sin(theta)`` below this threshold, the
# precession and spin angles are individually undetermined and only their
# combination is meaningful (DESIGN Sections 1.3 and 2.6). The value is a
# small dimensionless number: well below any physically meaningful tilt,
# yet safely above floating-point noise in the rotation matrix.
DEGENERACY_TOLERANCE = 1.0e-8


class EulerAngles(NamedTuple):
    """The three z-x-z Euler angles recovered from an orientation.

    The angles are an *output only* form (DESIGN Section 2.6): they are
    computed from the quaternion for display and never fed back into the
    state. When ``is_degenerate`` is true the orientation is at or near
    the gimbal-lock configuration ``sin(theta) = 0``; there the
    precession and spin are individually undetermined, ``spin_angle`` is
    ``None``, and ``precession_angle`` instead holds the one meaningful
    combination (``phi + psi`` near ``theta = 0``, ``phi - psi`` near
    ``theta = pi``).
    """

    precession_angle: float
    nutation_angle: float
    spin_angle: Optional[float]
    is_degenerate: bool


# --------------------------------------------------------------------
# Quaternion algebra (PSEUDOCODE Section 2.1)
# --------------------------------------------------------------------

def quaternion_multiply(left, right):
    """Return the Hamilton product ``left * right``, scalar first.

    The product is not commutative: the order encodes the order in which
    the two rotations compose (DESIGN Sections 2.2 and 2.4). Both inputs
    and the result are scalar-first quaternions ``(w, x, y, z)``.
    """
    left_w, left_x, left_y, left_z = left
    right_w, right_x, right_y, right_z = right
    return np.array([
        left_w*right_w - left_x*right_x - left_y*right_y - left_z*right_z,
        left_w*right_x + left_x*right_w + left_y*right_z - left_z*right_y,
        left_w*right_y - left_x*right_z + left_y*right_w + left_z*right_x,
        left_w*right_z + left_x*right_y - left_y*right_x + left_z*right_w,
    ])


def quaternion_conjugate(quaternion):
    """Return the conjugate: the vector part negated, scalar kept.

    For a unit quaternion the conjugate is the inverse rotation, which is
    what carries a vector from space components back to body components.
    """
    real_part, vector_x, vector_y, vector_z = quaternion
    return np.array([real_part, -vector_x, -vector_y, -vector_z])


def pure_quaternion(vector):
    """Build the pure quaternion ``(0, v_x, v_y, v_z)`` from a 3-vector.

    A pure quaternion (zero real part) is how a plain vector enters the
    sandwich product and the kinematic law below.
    """
    vector = np.asarray(vector, dtype=float)
    return np.array([0.0, vector[0], vector[1], vector[2]])


def scale_quaternion(factor, quaternion):
    """Return the quaternion with every component multiplied by ``factor``."""
    return factor * np.asarray(quaternion, dtype=float)


def normalize_quaternion(quaternion):
    """Return the quaternion rescaled to unit norm, sign unchanged.

    Only unit quaternions represent rotations (DESIGN Section 2.2). The
    sign is left strictly alone: dividing by a positive magnitude cannot
    flip ``q`` to ``-q``, so the double-cover care the rest of the
    project takes is never undone here.
    """
    magnitude = float(np.linalg.norm(quaternion))
    return np.asarray(quaternion, dtype=float) / magnitude


# --------------------------------------------------------------------
# Rotating a vector, and the orientation matrix (PSEUDOCODE Section 2.2)
# --------------------------------------------------------------------

def rotate_body_to_space(quaternion, vector_body):
    """Carry a vector from body components to space components.

    Applies the sandwich product ``q * (0, v_body) * conjugate(q)`` of
    DESIGN Section 2.2 and returns the vector part of the result; the
    real part is zero to round-off.
    """
    sandwich = quaternion_multiply(
        quaternion_multiply(quaternion, pure_quaternion(vector_body)),
        quaternion_conjugate(quaternion))
    return sandwich[1:]


def rotate_space_to_body(quaternion, vector_space):
    """Carry a vector from space components to body components.

    The inverse of :func:`rotate_body_to_space`: because ``q`` maps body
    to space, its conjugate maps space to body (DESIGN Section 1.2). Used
    by a space-frame torque such as gravity (PSEUDOCODE Section 6.3).
    """
    return rotate_body_to_space(
        quaternion_conjugate(quaternion), vector_space)


def quaternion_to_matrix(quaternion):
    """Return the 3x3 body-to-space rotation matrix of a unit quaternion.

    The matrix is equivalent to the sandwich product above and is what
    the renderer and the derived-quantity code use when a whole rotation
    matrix is wanted at once (DESIGN Section 1.4). The rows and columns
    are numbered from zero. Assumes a unit quaternion; normalize first if
    that is in doubt.
    """
    real, x_part, y_part, z_part = quaternion
    return np.array([
        [1 - 2*(y_part*y_part + z_part*z_part),
         2*(x_part*y_part - real*z_part),
         2*(x_part*z_part + real*y_part)],
        [2*(x_part*y_part + real*z_part),
         1 - 2*(x_part*x_part + z_part*z_part),
         2*(y_part*z_part - real*x_part)],
        [2*(x_part*z_part - real*y_part),
         2*(y_part*z_part + real*x_part),
         1 - 2*(x_part*x_part + y_part*y_part)],
    ])


# --------------------------------------------------------------------
# The orientation derivative (PSEUDOCODE Section 2.3)
# --------------------------------------------------------------------

def orientation_derivative(quaternion, angular_velocity_body):
    """Return ``q_dot``, the kinematic rate of the orientation.

    Implements ``q_dot = 0.5 * q * (0, omega_body)`` (DESIGN Section
    2.4). The angular velocity is in *body* components, so its pure
    quaternion multiplies ``q`` from the right; pairing the space-frame
    form with body-frame ``omega`` would be a silent-inverse error. This
    is the orientation half of the state derivative; the angular-velocity
    half comes from Euler's equations. It does *not* renormalize -- the
    integrator restores the unit norm once per step (DESIGN Section 6.3).
    """
    angular_velocity_quaternion = pure_quaternion(angular_velocity_body)
    product = quaternion_multiply(
        quaternion, angular_velocity_quaternion)
    return 0.5 * product


# --------------------------------------------------------------------
# Building a quaternion from authoring input (PSEUDOCODE Section 2.4)
# --------------------------------------------------------------------

def rotation_quaternion_about_z(angle):
    """Return the unit quaternion for a rotation ``angle`` about z."""
    half_angle = angle / 2.0
    return np.array([math.cos(half_angle), 0.0, 0.0,
                     math.sin(half_angle)])


def rotation_quaternion_about_x(angle):
    """Return the unit quaternion for a rotation ``angle`` about x."""
    half_angle = angle / 2.0
    return np.array([math.cos(half_angle), math.sin(half_angle),
                     0.0, 0.0])


def quaternion_from_euler_zxz(precession_angle, nutation_angle,
                              spin_angle):
    """Build a quaternion from z-x-z Euler angles (DESIGN Section 1.3).

    Composes ``R_z(phi) R_x(theta) R_z(psi)`` as a quaternion product in
    the same order, since quaternion multiplication mirrors the matrix
    product for these body-to-space rotations.
    """
    precession = rotation_quaternion_about_z(precession_angle)
    nutation = rotation_quaternion_about_x(nutation_angle)
    spin = rotation_quaternion_about_z(spin_angle)
    return quaternion_multiply(
        precession, quaternion_multiply(nutation, spin))


def quaternion_from_axis_angle(axis, angle):
    """Build a quaternion for a rotation ``angle`` about ``axis``.

    The axis need not be normalized; the ``1 / length`` factor is folded
    into the vector-part scale so the result is a unit quaternion.
    """
    axis = np.asarray(axis, dtype=float)
    length = float(np.linalg.norm(axis))
    half_angle = angle / 2.0
    vector_scale = math.sin(half_angle) / length
    return np.array([
        math.cos(half_angle),
        axis[0]*vector_scale,
        axis[1]*vector_scale,
        axis[2]*vector_scale,
    ])


# --------------------------------------------------------------------
# Euler angles as an output (PSEUDOCODE Section 2.5)
# --------------------------------------------------------------------

def euler_angles_from_quaternion(quaternion):
    """Recover the z-x-z Euler angles from an orientation, for display.

    Returns an :class:`EulerAngles`. This is an output-only computation
    (DESIGN Section 2.6): never fed back into the state, so the angles
    cannot drift out of agreement with the orientation they describe.

    Near ``sin(theta) = 0`` the precession and spin are individually
    undetermined (gimbal lock, DESIGN Section 1.3). There the routine
    reports the one well-defined combination and flags the result
    degenerate, rather than returning two rapidly changing numbers as
    though they were measurements.

    Every matrix index below follows from writing out
    ``R = R_z(phi) R_x(theta) R_z(psi)`` by hand (rows and columns
    numbered from zero):

        R[2][2] = cos(theta)
        R[0][2] =  sin(phi) sin(theta)   R[1][2] = -cos(phi) sin(theta)
        R[2][0] =  sin(theta) sin(psi)   R[2][1] =  sin(theta) cos(psi)
        R[0][0], R[1][0] -> cos, sin of (phi + psi) at theta = 0
                            cos, sin of (phi - psi) at theta = pi
    """
    matrix = quaternion_to_matrix(quaternion)

    # Measure sin(theta) DIRECTLY from the off-axis entries rather than
    # as sqrt(1 - cos^2), which loses precision to cancellation near a
    # pole. Since R[0][2] = sin(phi) sin(theta) and
    # R[1][2] = -cos(phi) sin(theta), their hypotenuse is |sin(theta)|
    # for any phi, and at a pole it is round-off small (~1e-16) rather
    # than ~1e-8. With cos(theta) from the (2, 2) entry, atan2 then gives
    # a well-conditioned nutation in [0, pi] -- unlike arccos, which is
    # ill-conditioned exactly at the poles.
    cos_nutation = float(matrix[2, 2])
    sin_nutation = math.hypot(matrix[0, 2], matrix[1, 2])
    nutation_angle = math.atan2(sin_nutation, cos_nutation)

    if sin_nutation > DEGENERACY_TOLERANCE:
        # Generic case: all three angles are separately determined.
        precession_angle = math.atan2(matrix[0, 2], -matrix[1, 2])
        spin_angle = math.atan2(matrix[2, 0], matrix[2, 1])
        return EulerAngles(
            precession_angle, nutation_angle, spin_angle, False)

    # Gimbal lock: only the combination R[1][0], R[0][0] carries
    # information. It equals phi + psi when theta ~ 0 and phi - psi when
    # theta ~ pi (the sign of cos(theta)).
    combination = math.atan2(matrix[1, 0], matrix[0, 0])
    return EulerAngles(combination, nutation_angle, None, True)
