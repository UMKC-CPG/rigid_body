"""The integrated state and the quantities derived from it.

This is the code form of PSEUDOCODE Section 3 (DESIGN Sections 2.1 and
2.5). The state advanced by the integrator is seven bare SI numbers -- a
scalar-first unit quaternion and the body-frame angular velocity -- and
everything else the program shows is *recomputed* from those seven
numbers on demand, never stored alongside them, so a derived value can
never fall out of agreement with the state it summarizes.

Under torque-free motion the space-frame angular momentum and the kinetic
energy are the two conserved quantities the monitor watches (Section 8)
and the Poinsot construction is built from (Section 10). Note it is the
*space-frame* momentum that is conserved; the body-frame components
change continuously as the body tumbles beneath the fixed vector.
"""

from dataclasses import dataclass

import numpy as np

from rigid_body.core.orientation import rotate_body_to_space


@dataclass(eq=False)
class State:
    """The seven-number state: orientation and angular velocity.

    ``body_to_space_quaternion`` is four numbers, scalar first and unit
    norm (Section 2.2); ``angular_velocity_body`` is three numbers in the
    body principal-axis frame. Both are bare SI floats -- units live only
    at the scenario boundary (ARCHITECTURE Section 5.5). Translation is
    not part of the state: for free tumbling the center of mass decouples,
    and for the heavy top the pivot is fixed (DESIGN Section 2.1).

    A step produces a *new* ``State`` rather than mutating one in place,
    matching the read-only discipline the loop depends on.
    """

    body_to_space_quaternion: np.ndarray
    angular_velocity_body: np.ndarray


def angular_momentum_body(state, body):
    """Return the angular momentum in body components.

    Componentwise ``I_k * omega_k`` because the body frame is the
    principal-axis frame, where the inertia tensor is diagonal (DESIGN
    Sections 1.1 and 2.5).
    """
    return body.principal_moments * state.angular_velocity_body


def angular_momentum_space(state, body):
    """Return the same angular-momentum vector in space components.

    It is this space-frame form that is conserved under torque-free
    motion (DESIGN Section 2.5). Equivalent to
    ``body_to_space_matrix @ angular_momentum_body`` but obtained by
    rotating the single vector through the sandwich product (Section 2.2).
    """
    return rotate_body_to_space(
        state.body_to_space_quaternion,
        angular_momentum_body(state, body))


def kinetic_energy(state, body):
    """Return the rotational kinetic energy of the state.

    ``0.5 * (I_1 omega_1^2 + I_2 omega_2^2 + I_3 omega_3^2)`` (DESIGN
    Section 2.5), a frame-independent scalar.
    """
    angular_velocity = state.angular_velocity_body
    return 0.5 * float(np.sum(
        body.principal_moments * angular_velocity**2))
