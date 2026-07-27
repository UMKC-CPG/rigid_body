"""Torque models: the external torques the equations of motion take.

This is the code form of PSEUDOCODE Section 6 (DESIGN Section 5). Each
model answers one question through a single operation,

    torque_body(time, state, body) -> a 3-vector in body components,

and obeys the two constraints the equations of motion impose (DESIGN
Section 4.5): the torque arrives in *body* components, and torques are
*additive*, so several acting at once sum to a single vector. A model is
also **pure** -- it reads the state and returns a torque, accumulating
nothing -- because a multi-stage integrator evaluates the derivative at
trial states off the trajectory, and a model that remembered anything
across those evaluations would corrupt itself.

Torque-free motion is not a special case here: it is an empty list of
models, whose sum ``total_torque_body`` returns as the zero vector
(DESIGN Section 5.2). Internal dissipation is deliberately absent: it is
not a torque at all but a statement that the body is not rigid, so it does
not implement this interface (DESIGN Section 5.5).
"""

import numpy as np

from rigid_body.core.orientation import rotate_space_to_body


class GravityTorque:
    """Uniform gravity acting through a fixed pivot (the heavy top).

    Gravity is naturally a space-frame vector -- it points down whatever
    the body does -- so it is rotated into the body frame and crossed with
    the constant body-frame lever arm from the pivot to the center of mass
    (DESIGN Section 5.3). That ordering does one rotation per step and
    keeps the constant lever arm visibly constant.
    """

    def __init__(self, gravity_space, pivot_to_center_of_mass_body):
        # ``gravity_space`` is the gravitational acceleration vector
        # (points down); the lever arm is fixed in the body frame.
        self.gravity_space = np.asarray(gravity_space, dtype=float)
        self.pivot_to_center_of_mass_body = np.asarray(
            pivot_to_center_of_mass_body, dtype=float)

    def torque_body(self, time, state, body):
        gravity_body = rotate_space_to_body(
            state.body_to_space_quaternion, self.gravity_space)
        # Force is mass times gravitational acceleration; torque is the
        # lever arm crossed with that force (tau = r x F).
        weight_body = body.total_mass * gravity_body
        return np.cross(self.pivot_to_center_of_mass_body, weight_body)


class ViscousDamping:
    """A torque opposing the angular velocity (external dissipation).

    The right model for a body immersed in a fluid or dragging on its
    mount (DESIGN Section 5.4). Its power ``Gamma . omega`` is
    ``-c |omega|^2 <= 0``, so energy falls monotonically and the body
    spins down; angular momentum falls too, because this is a genuine
    external torque. It is *not* a model of internal friction.
    """

    def __init__(self, damping_coefficient):
        self.damping_coefficient = float(damping_coefficient)

    def torque_body(self, time, state, body):
        return -self.damping_coefficient * state.angular_velocity_body


def total_torque_body(time, state, body, torque_models):
    """Sum the active torques, in body components and in fixed order.

    Torques are additive (DESIGN Section 4.5). The list is iterated in the
    order given, because floating-point addition is not associative and a
    different order could give a bitwise-different total, breaking the
    determinism guarantee (DESIGN Section 5.6). An empty list yields the
    zero vector -- torque-free motion (DESIGN Section 5.2).
    """
    total = np.zeros(3)
    for model in torque_models:
        total = total + model.torque_body(time, state, body)
    return total
