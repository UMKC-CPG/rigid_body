"""Closed-form rigid-body motions, for overlay and validation.

This is the code form of PSEUDOCODE Section 9 (DESIGN Section 8). A handful
of motions have exact solutions, and VISION Principle 3 makes them the
yardstick the integrator is judged against while VISION Goal 9 draws them
on screen beside the numerical motion. Each solution is produced in the
same form the engine emits -- a state, or the body-frame angular velocity,
as a function of time -- so the renderer and the test harness treat
analytic and numerical motion identically.

The asymmetric-top solution is the certified one: the forms here match the
exact elliptic solution verified by ``dev/spikes/free_top_elliptic.py``
against a high-accuracy integration of the same initial condition (DESIGN
Section 8.4).
"""

import math
from typing import NamedTuple

import numpy as np
from scipy.special import ellipj, ellipk

from rigid_body.core.orientation import (
    rotate_body_to_space, quaternion_from_axis_angle, quaternion_multiply)
from rigid_body.dynamics.state import State


class FreeTopParameters(NamedTuple):
    """Branch parameters for the torque-free asymmetric-top solution.

    ``coefficients`` are the three amplitudes on axes 1, 2, 3;
    ``axis_functions`` names the Jacobi function riding each axis;
    ``rate`` is the time scale; ``modulus_squared`` is the squared elliptic
    modulus; and ``period`` is the body-frame period.
    """

    coefficients: np.ndarray
    axis_functions: tuple
    rate: float
    modulus_squared: float
    period: float


# --------------------------------------------------------------------
# Steady rotation about a principal axis (DESIGN Section 8.2)
# --------------------------------------------------------------------

def steady_principal_rotation(initial_state, time):
    """Return the state of a body spinning steadily about a principal axis.

    A body set spinning about one principal axis stays there, with constant
    angular velocity, turning uniformly about the space-fixed axis along the
    angular momentum. Any leak of ``omega`` onto the other two axes is pure
    numerical error, so this is the first and sharpest trivial oracle.
    """
    angular_velocity = initial_state.angular_velocity_body
    axis_space = rotate_body_to_space(
        initial_state.body_to_space_quaternion, angular_velocity)
    rotation = quaternion_from_axis_angle(
        axis_space, float(np.linalg.norm(angular_velocity)) * time)
    # A space-frame rotation composes on the left of the orientation.
    orientation = quaternion_multiply(
        rotation, initial_state.body_to_space_quaternion)
    return State(orientation, angular_velocity)


# --------------------------------------------------------------------
# The torque-free symmetric top (DESIGN Section 8.3)
# --------------------------------------------------------------------

def symmetric_top_precession_rates(state, body):
    """Return the body-frame and space-frame precession rates.

    For a symmetric top (``I_1 = I_2 != I_3``) under no torque the motion is
    a steady precession with two distinct rates: the pair
    ``(omega_1, omega_2)`` circles body axis 3 at the body rate, and the
    figure axis precesses about the fixed angular momentum at the space
    rate. Returns ``(body_rate, space_rate)``.

    The sign of the body rate is physics: negative for a prolate body
    (``I_3 < I_1``, rod-like) and positive for an oblate one
    (``I_3 > I_1``, disk-like, such as the Earth, whose free precession is
    therefore prograde -- the Chandler wobble).
    """
    moment_transverse = float(body.principal_moments[0])
    moment_axial = float(body.principal_moments[2])
    spin_axial = float(state.angular_velocity_body[2])

    body_rate = spin_axial * (moment_axial - moment_transverse) / (
        moment_transverse)
    momentum_magnitude = float(np.linalg.norm(
        body.principal_moments * state.angular_velocity_body))
    space_rate = momentum_magnitude / moment_transverse
    return body_rate, space_rate


# --------------------------------------------------------------------
# The torque-free asymmetric top (DESIGN Section 8.4)
# --------------------------------------------------------------------

def free_asymmetric_top_parameters(principal_moments, initial_spin):
    """Return the branch parameters for the free asymmetric-top solution.

    The moments must be strictly ordered ``I_1 < I_2 < I_3`` and the initial
    spin anchored at ``omega_2 = 0`` -- the phase where the elliptic
    functions start from ``(sn, cn, dn) = (0, 1, 1)``. The two branches
    correspond to spin nearest the largest-moment axis or the smallest.
    """
    moment_1, moment_2, moment_3 = (float(m) for m in principal_moments)
    if not (moment_1 < moment_2 < moment_3):
        raise ValueError(
            "free asymmetric top requires I_1 < I_2 < I_3")
    spin = np.asarray(initial_spin, dtype=float)
    if abs(spin[1]) > 1.0e-12:
        raise ValueError(
            "initial spin must be anchored at omega_2 = 0")

    twice_energy = float(
        moment_1 * spin[0]**2 + moment_2 * spin[1]**2
        + moment_3 * spin[2]**2)
    momentum_squared = float(
        (moment_1 * spin[0])**2 + (moment_2 * spin[1])**2
        + (moment_3 * spin[2])**2)

    if momentum_squared - twice_energy * moment_2 >= 0.0:
        # Main branch: spin nearest axis 3 (the largest moment).
        coefficients = np.array([
            math.sqrt((twice_energy * moment_3 - momentum_squared)
                      / (moment_1 * (moment_3 - moment_1))),
            math.sqrt((twice_energy * moment_3 - momentum_squared)
                      / (moment_2 * (moment_3 - moment_2))),
            math.sqrt((momentum_squared - twice_energy * moment_1)
                      / (moment_3 * (moment_3 - moment_1)))])
        axis_functions = ("cn", "sn", "dn")
        rate = math.sqrt(
            (moment_3 - moment_2)
            * (momentum_squared - twice_energy * moment_1)
            / (moment_1 * moment_2 * moment_3))
        modulus_squared = (
            (moment_2 - moment_1)
            * (twice_energy * moment_3 - momentum_squared)
            / ((moment_3 - moment_2)
               * (momentum_squared - twice_energy * moment_1)))
    else:
        # Complementary branch: spin nearest axis 1 (the smallest moment),
        # DESIGN Section 8.4 with axes 1 and 3 exchanged.
        coefficients = np.array([
            math.sqrt((momentum_squared - twice_energy * moment_3)
                      / (moment_1 * (moment_1 - moment_3))),
            math.sqrt((twice_energy * moment_1 - momentum_squared)
                      / (moment_2 * (moment_1 - moment_2))),
            math.sqrt((twice_energy * moment_1 - momentum_squared)
                      / (moment_3 * (moment_1 - moment_3)))])
        axis_functions = ("dn", "sn", "cn")
        rate = math.sqrt(
            (moment_1 - moment_2)
            * (momentum_squared - twice_energy * moment_3)
            / (moment_1 * moment_2 * moment_3))
        modulus_squared = (
            (moment_2 - moment_3)
            * (twice_energy * moment_1 - momentum_squared)
            / ((moment_1 - moment_2)
               * (momentum_squared - twice_energy * moment_3)))

    period = 4.0 * float(ellipk(modulus_squared)) / rate
    return FreeTopParameters(
        coefficients, axis_functions, rate, modulus_squared, period)


def evaluate_free_asymmetric_top(parameters, time):
    """Evaluate the body-frame angular velocity of the free top at ``time``.

    Accepts a scalar or an array of times; returns a 3-vector or an array of
    shape ``(len(time), 3)``. The three Jacobi functions are computed once
    at the scaled time and each axis picks the one it rides.
    """
    scaled_time = parameters.rate * np.asarray(time, dtype=float)
    sn, cn, dn, _phase = ellipj(scaled_time, parameters.modulus_squared)
    function_table = {"sn": sn, "cn": cn, "dn": dn}
    columns = [parameters.coefficients[axis]
               * function_table[parameters.axis_functions[axis]]
               for axis in range(3)]
    return np.stack(columns, axis=-1)


# --------------------------------------------------------------------
# Steady precession of the heavy symmetric top (DESIGN Section 8.5)
# --------------------------------------------------------------------

def heavy_top_steady_precession(body, spin_rate_axial, nutation_angle,
                                gravity_magnitude, pivot_distance):
    """Return the steady-precession rates of a heavy symmetric top.

    At a fixed nutation angle a steady precession rate must satisfy
    ``I_1 cos(theta) phi^2 - I_3 omega_3 phi + M g l = 0``. This quadratic
    has two roots -- the top can precess slow or fast at the same tilt --
    and real roots exist only when the top spins fast enough. Returns a dict
    with ``exists`` and, when true, ``slow_root`` and ``fast_root``.
    """
    moment_transverse = float(body.principal_moments[0])
    moment_axial = float(body.principal_moments[2])

    quadratic_a = moment_transverse * math.cos(nutation_angle)
    quadratic_b = -(moment_axial * spin_rate_axial)
    quadratic_c = body.total_mass * gravity_magnitude * pivot_distance

    discriminant = quadratic_b**2 - 4.0 * quadratic_a * quadratic_c
    if discriminant < 0.0:
        return {"exists": False}

    root = math.sqrt(discriminant)
    return {
        "exists": True,
        # The slow root is the familiar M g l / (I_3 omega_3) of a fast top.
        "slow_root": (-quadratic_b - root) / (2.0 * quadratic_a),
        "fast_root": (-quadratic_b + root) / (2.0 * quadratic_a)}
