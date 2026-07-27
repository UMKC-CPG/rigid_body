"""Integrators: the selectable schemes that step the state.

This is the code form of PSEUDOCODE Section 7 (DESIGN Section 6). The
integrator is the one component whose *errors* are meant to be seen: a
nonzero error is expected here, and the goal is to measure and disclose
it, not to hide it. Which integrator runs is a scenario setting, so every
integrator presents the same shape,

    advance(state, time, dt, derivative_function) -> new_state,

and knows nothing of bodies, torques, or rendering -- all of that is
sealed inside ``derivative_function`` by the time the integrator sees it.

The step is fixed, never adaptive (DESIGN Section 6.1): one call, one
``dt``, one step, so the sequence of states depends on the scenario alone
and not on how the run was paced.

The default is classical fourth-order Runge-Kutta. A structure-preserving
(symplectic) scheme for the long-integration regime is designed but not
yet built (DESIGN Section 6.5); ``select_integrator`` refuses those names
with a clear message rather than silently substituting RK4.
"""

import numpy as np

from rigid_body.core.orientation import normalize_quaternion
from rigid_body.dynamics.state import State


# --------------------------------------------------------------------
# Field-wise arithmetic on states and their derivatives (Section 7.1)
# --------------------------------------------------------------------
# A state and a derivative share the same two-field shape, so the same
# arithmetic serves both: scale each field, add field by field.

def scale_state(factor, state):
    """Return the state (or derivative) with each field scaled."""
    return State(
        body_to_space_quaternion=factor * state.body_to_space_quaternion,
        angular_velocity_body=factor * state.angular_velocity_body)


def add_states(first, second):
    """Return the field-wise sum of two states (or derivatives)."""
    return State(
        body_to_space_quaternion=(first.body_to_space_quaternion
                                  + second.body_to_space_quaternion),
        angular_velocity_body=(first.angular_velocity_body
                               + second.angular_velocity_body))


def advance_by(base, factor, rate):
    """Return ``base + factor * rate``: a trial state one move away."""
    return add_states(base, scale_state(factor, rate))


def renormalize(state):
    """Project the quaternion back onto the unit sphere (Section 7.3).

    RK4 takes short straight-line moves along the tangent and lands
    slightly off the sphere each step; left alone ``|q|`` drifts from one
    and a non-unit quaternion applied as a rotation introduces a scaling.
    This restores the constraint once, after the whole step -- not inside
    the stages, whose trial quaternions are meant to be slightly off the
    sphere. ``normalize_quaternion`` divides by a positive magnitude, so
    the sign is untouched and the double cover is undisturbed.
    """
    return State(
        body_to_space_quaternion=normalize_quaternion(
            state.body_to_space_quaternion),
        angular_velocity_body=state.angular_velocity_body)


class RungeKutta4Integrator:
    """The classical fourth-order Runge-Kutta method (DESIGN Section 6.2).

    Samples the derivative four times per step -- once at the start, twice
    at trial midpoints, once at a trial endpoint -- and combines them in
    the standard weighting. Its global error is order four, ``~ C dt^4``,
    a testable claim: halving ``dt`` must cut the error about sixteenfold.
    The two midpoint samples are at trial states off the trajectory, which
    is why the derivative must be pure.
    """

    def advance(self, state, time, dt, derivative_function):
        first = derivative_function(time, state)
        second = derivative_function(
            time + dt / 2.0, advance_by(state, dt / 2.0, first))
        third = derivative_function(
            time + dt / 2.0, advance_by(state, dt / 2.0, second))
        fourth = derivative_function(
            time + dt, advance_by(state, dt, third))

        # Standard weighting, field-wise: first + 2 second + 2 third +
        # fourth, scaled by dt / 6.
        weighted_rate = add_states(
            add_states(first, scale_state(2.0, second)),
            add_states(scale_state(2.0, third), fourth))
        stepped = advance_by(state, dt / 6.0, weighted_rate)

        # Restore |q| = 1 once, after the whole step (Section 7.3).
        return renormalize(stepped)


def select_integrator(integrator_name):
    """Return the integrator a scenario asks for by name (Section 7.1).

    The name is the scenario key of PSEUDOCODE Section 12.3. RK4 is the
    interactive default; the two symplectic schemes are designed for the
    long regime (Section 7.5) but not yet built, so their names are
    refused explicitly rather than silently mapped to RK4.
    """
    if integrator_name in ("implicit_midpoint", "splitting"):
        raise NotImplementedError(
            f"integrator '{integrator_name}' is designed but not yet "
            f"built (PSEUDOCODE Section 7.5); use 'rk4' for now")
    return RungeKutta4Integrator()
