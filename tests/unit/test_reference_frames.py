"""Unit tests for frame presentation (geometry/reference_frames.py).

The module's whole job is to re-express one motion in either frame without
ever creating a second motion, so the tests pin exactly that:

* The two mappings are genuine inverses, and ``express_in_space`` agrees
  with the state's own ``angular_momentum_space``.
* ``to_view`` returns a view-frame vector untouched and carries the other
  frame's vector across.
* The frame invariants are truly frame-free: the same numbers come out
  whether ``omega`` and ``L`` are read in the body or the space frame.
* Under torque-free motion the conserved invariants hold constant while
  ``omega_magnitude`` and the ``omega``-to-``L`` angle are allowed to vary
  -- frame-free is not the same as time-constant.
* The Goal-5 lesson made concrete: the body-frame ``L`` sweeps while the
  space-frame ``L`` stands still, and the body-frame ``L`` companion curve
  rides the fixed-radius ``|L|`` sphere along the polhode.
"""

import math

import numpy as np
import pytest

from rigid_body.core import orientation as ori
from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import integrators as ig
from rigid_body.dynamics import simulation_engine as engine
from rigid_body.geometry import poinsot as po
from rigid_body.geometry import reference_frames as rf


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


def tumbled_state(angular_velocity):
    """A state at a generic (non-identity) orientation, for sharp tests."""
    quaternion = ori.normalize_quaternion(
        np.array([0.5, 0.5, -0.3, 0.7]))
    return st.State(quaternion, np.asarray(angular_velocity, dtype=float))


# --------------------------------------------------------------------
# One mapping, both ways (Section 11.1)
# --------------------------------------------------------------------

def test_the_two_mappings_are_inverses():
    # Carrying a vector out to space and back must return it exactly: the
    # two frames are one rotation and its inverse, nothing more.
    state = tumbled_state([0.3, 0.5, 2.0])
    vector_body = np.array([1.0, -2.0, 0.5])
    round_trip = rf.express_in_body(
        state, rf.express_in_space(state, vector_body))
    np.testing.assert_allclose(round_trip, vector_body, atol=1e-14)


def test_express_in_space_matches_the_orientation_matrix():
    # v_space = R @ v_body, with R the body-to-space matrix of Section 2.2.
    state = tumbled_state([0.3, 0.5, 2.0])
    vector_body = np.array([1.0, -2.0, 0.5])
    matrix = ori.quaternion_to_matrix(state.body_to_space_quaternion)
    np.testing.assert_allclose(
        rf.express_in_space(state, vector_body),
        matrix @ vector_body, atol=1e-14)


def test_express_in_space_reproduces_angular_momentum_space():
    # A cross-check against the state layer: the space-frame angular
    # momentum is exactly the body-frame one carried across (Section 3.2).
    body = make_body(ASYMMETRIC_BOX)
    state = tumbled_state([0.3, 0.5, 2.0])
    carried = rf.express_in_space(
        state, st.angular_momentum_body(state, body))
    np.testing.assert_allclose(
        carried, st.angular_momentum_space(state, body), atol=1e-14)


# --------------------------------------------------------------------
# to_view: express a quantity in the held-still frame (Section 11.2)
# --------------------------------------------------------------------

def test_to_view_returns_a_view_frame_vector_unchanged():
    state = tumbled_state([0.3, 0.5, 2.0])
    space_vector = np.array([0.2, 0.7, -0.4])
    np.testing.assert_array_equal(
        rf.to_view(state, rf.Frame.SPACE, space_vector, rf.Frame.SPACE),
        space_vector)
    body_vector = np.array([1.0, -1.0, 2.0])
    np.testing.assert_array_equal(
        rf.to_view(state, rf.Frame.BODY, body_vector, rf.Frame.BODY),
        body_vector)


def test_to_view_carries_a_vector_across_frames():
    state = tumbled_state([0.3, 0.5, 2.0])
    body_vector = np.array([1.0, -1.0, 2.0])
    # A body vector, seen in the space-frame view, is expressed in space.
    np.testing.assert_allclose(
        rf.to_view(state, rf.Frame.SPACE, body_vector, rf.Frame.BODY),
        rf.express_in_space(state, body_vector), atol=1e-14)
    space_vector = np.array([0.2, 0.7, -0.4])
    # A space vector, seen in the body-frame view, is expressed in body.
    np.testing.assert_allclose(
        rf.to_view(state, rf.Frame.BODY, space_vector, rf.Frame.SPACE),
        rf.express_in_body(state, space_vector), atol=1e-14)


# --------------------------------------------------------------------
# The invariants are frame-free (Section 11.3)
# --------------------------------------------------------------------

def test_invariants_match_the_state_layer():
    body = make_body(ASYMMETRIC_BOX)
    state = tumbled_state([0.3, 0.5, 2.0])
    invariants = rf.frame_invariants(state, body)
    momentum_body = st.angular_momentum_body(state, body)
    assert invariants.kinetic_energy == pytest.approx(
        st.kinetic_energy(state, body))
    assert invariants.twice_energy == pytest.approx(
        2.0 * st.kinetic_energy(state, body))
    assert invariants.momentum_magnitude == pytest.approx(
        float(np.linalg.norm(momentum_body)))
    assert invariants.momentum_squared == pytest.approx(
        float(np.dot(momentum_body, momentum_body)))


def test_omega_to_L_angle_is_frame_independent():
    # The crux of Section 11.3: the angle read from body components equals
    # the angle read from space components -- a change of frame rotates
    # omega and L together and leaves the angle untouched.
    body = make_body(ASYMMETRIC_BOX)
    state = tumbled_state([0.3, 0.5, 2.0])
    invariants = rf.frame_invariants(state, body)

    omega_space = rf.express_in_space(state, state.angular_velocity_body)
    momentum_space = st.angular_momentum_space(state, body)
    cosine = np.dot(omega_space, momentum_space) / (
        np.linalg.norm(omega_space) * np.linalg.norm(momentum_space))
    angle_in_space = math.acos(max(-1.0, min(1.0, float(cosine))))
    assert invariants.omega_to_L_angle == pytest.approx(
        angle_in_space, abs=1e-13)


def test_conserved_invariants_hold_but_omega_may_vary():
    # Under torque-free asymmetric motion the energies and |L| are
    # constant, yet |omega| and the omega-to-L angle genuinely change:
    # frame-free is not the same as time-constant.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.05, 2.5, 0.05]))
    integrator = ig.RungeKutta4Integrator()
    start = rf.frame_invariants(state, body)

    omega_magnitudes = [start.omega_magnitude]
    angles = [start.omega_to_L_angle]
    time = 0.0
    for _ in range(2000):
        state = engine.advance_one_substep(
            state, time, 1.0e-3, body, [], integrator)
        time = time + 1.0e-3
        current = rf.frame_invariants(state, body)
        # Conserved quantities hold across the whole run.
        assert current.twice_energy == pytest.approx(
            start.twice_energy, rel=1e-6)
        assert current.momentum_magnitude == pytest.approx(
            start.momentum_magnitude, rel=1e-6)
        omega_magnitudes.append(current.omega_magnitude)
        angles.append(current.omega_to_L_angle)

    # But omega's magnitude and its angle to L are demonstrably not.
    assert np.ptp(omega_magnitudes) > 1e-3
    assert np.ptp(angles) > 1e-3


# --------------------------------------------------------------------
# The angular momentum in each frame: the Goal-5 lesson (Section 11.4)
# --------------------------------------------------------------------

def test_body_frame_L_sweeps_while_space_frame_L_holds_still():
    # One arrow, two accounts: the space-frame L is dead still, while the
    # body-frame components rise and fall as the body tumbles beneath it.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.2, 2.0, 0.3]))
    integrator = ig.RungeKutta4Integrator()
    initial_space_L = st.angular_momentum_space(state, body)

    body_L_samples = []
    time = 0.0
    for _ in range(1500):
        state = engine.advance_one_substep(
            state, time, 1.0e-3, body, [], integrator)
        time = time + 1.0e-3
        # The space-frame arrow does not move.
        np.testing.assert_allclose(
            st.angular_momentum_space(state, body),
            initial_space_L, atol=1e-4)
        body_L_samples.append(st.angular_momentum_body(state, body))

    # The body-frame arrow does: at least one component swings widely.
    body_L_samples = np.array(body_L_samples)
    assert np.max(np.ptp(body_L_samples, axis=0)) > 0.1


def test_body_frame_L_curve_rides_the_fixed_radius_sphere():
    # L = I omega along the polhode: every companion point sits on the
    # sphere of radius |L|, since |L| is conserved (Section 11.4).
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    invariants = rf.frame_invariants(state, body)
    polhode_points = po.polhode(body, state, sample_count=240).points

    companion = rf.angular_momentum_body_curve(polhode_points, body)
    radii = np.linalg.norm(companion, axis=1)
    np.testing.assert_allclose(
        radii, invariants.momentum_magnitude, rtol=1e-9)


def test_body_frame_L_curve_matches_I_omega_at_the_state():
    # The companion curve is literally L = I omega, so evaluating it at the
    # state's own omega reproduces the state's body-frame angular momentum.
    body = make_body(ASYMMETRIC_BOX)
    state = tumbled_state([0.3, 0.5, 2.0])
    single_point = state.angular_velocity_body.reshape(1, 3)
    companion = rf.angular_momentum_body_curve(single_point, body)
    np.testing.assert_allclose(
        companion[0], st.angular_momentum_body(state, body), atol=1e-14)
