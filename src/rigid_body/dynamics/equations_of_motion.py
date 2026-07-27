"""Euler's equations: the derivative the integrator steps.

This is the code form of PSEUDOCODE Section 5 (DESIGN Section 4). The
``omega_dot`` half is where the physics lives -- Euler's equations -- and
the ``q_dot`` half is the pure kinematic bookkeeping of the orientation
module. A torque enters only the first half; it never appears in
``q_dot`` (DESIGN Section 4.2).

The derivative function is **pure**: it reads the state and returns a
derivative, mutating nothing, so a multi-stage integrator can evaluate it
at trial states off the trajectory without the evaluations interfering.
The returned derivative carries the same two fields as the state -- a
quaternion rate and an angular-velocity rate -- so the integrator advances
each by the matching rate.
"""

import math

import numpy as np

from rigid_body.core.orientation import orientation_derivative
from rigid_body.dynamics.state import State
from rigid_body.dynamics.torque_models import total_torque_body
from rigid_body.body.rigid_body_model import TopClass


def angular_acceleration_body(angular_velocity_body, body, torque_body):
    """Return ``omega_dot`` from Euler's equations (DESIGN Section 4.1).

    In the body principal-axis frame the inertia tensor is diagonal, so
    the vector equation separates into three scalar ones and inverting the
    tensor is a per-component divide, never a matrix solve. The
    ``(I_j - I_k)`` products are the gyroscopic term ``omega x (I omega)``
    written out per axis.
    """
    omega_1, omega_2, omega_3 = angular_velocity_body
    moment_1, moment_2, moment_3 = body.principal_moments
    torque_1, torque_2, torque_3 = torque_body
    return np.array([
        ((moment_2 - moment_3) * omega_2 * omega_3 + torque_1) / moment_1,
        ((moment_3 - moment_1) * omega_3 * omega_1 + torque_2) / moment_2,
        ((moment_1 - moment_2) * omega_1 * omega_2 + torque_3) / moment_3,
    ])


def state_derivative(time, state, body, torque_models):
    """Return the full state derivative ``(q_dot, omega_dot)``.

    Stitches the physics half (Euler's equations, above) with the
    bookkeeping half (the kinematic law of the orientation module),
    summing the active torques first (DESIGN Section 4.2). Pure: nothing
    here or in the torque models mutates the state.

    The result is a ``State`` whose two fields hold the *rates* of the
    corresponding state fields, not a physical state -- the quaternion
    rate is not a unit quaternion. This shared two-field shape is what the
    integrator's arithmetic relies on (PSEUDOCODE Section 7.1).
    """
    quaternion = state.body_to_space_quaternion
    angular_velocity = state.angular_velocity_body

    # Torque first, summed in body components (DESIGN Section 4.5).
    torque_body = total_torque_body(time, state, body, torque_models)

    angular_velocity_rate = angular_acceleration_body(
        angular_velocity, body, torque_body)
    quaternion_rate = orientation_derivative(quaternion, angular_velocity)

    return State(body_to_space_quaternion=quaternion_rate,
                 angular_velocity_body=angular_velocity_rate)


def instability_growth_rate(body, spin_rate):
    """Return the intermediate-axis growth rate ``sigma`` (DESIGN 4.4).

    A small tilt off the intermediate axis grows as ``exp(sigma t)``.
    Written in the sorted moments so it does not depend on which axis is
    labeled intermediate. The rate is zero unless the three moments are
    strictly distinct -- the interface guard that forbids offering the
    Dzhanibekov demonstration for a body that provably cannot show it.
    """
    if body.top_class is not TopClass.ASYMMETRIC:
        return 0.0
    moment_min, moment_mid, moment_max = np.sort(body.principal_moments)
    return abs(spin_rate) * math.sqrt(
        (moment_max - moment_mid) * (moment_mid - moment_min)
        / (moment_min * moment_max))


def estimate_time_to_flip(growth_rate, initial_tilt):
    """Estimate when a growing tilt reaches order one (DESIGN Section 4.4).

    Roughly ``(1 / sigma) * ln(1 / eps_0)`` for an initial tilt
    ``initial_tilt``, from ``eps(t) = initial_tilt * exp(sigma t)``. With
    no instability the flip never comes, so the time is infinite.
    """
    if growth_rate == 0.0:
        return math.inf
    return (1.0 / growth_rate) * math.log(1.0 / initial_tilt)


def energy_rate(state, torque_body):
    """Return ``d(kinetic_energy)/dt = Gamma . omega`` (DESIGN Section 4.6).

    The exact rate law, true whether or not a torque acts, which is what
    keeps the conservation monitor meaningful once a torque is switched
    on. The companion law is ``d(L_space)/dt = Gamma_space``; both rates
    vanish for torque-free motion.
    """
    return float(np.dot(torque_body, state.angular_velocity_body))
