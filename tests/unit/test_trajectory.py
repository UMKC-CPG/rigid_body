"""Unit tests for trajectory retention (dynamics/trajectory.py).

Retention is a bounded, read-only cache over a recomputable motion, so the
tests pin the two things that make it safe and useful:

* The ring buffer keeps the recent window exactly, overwrites the oldest
  when full, isolates each stored state by copy, and never lets a read run
  off the ends (Section 13.4, 13.2).
* The keyframe store reaches any instant of a long run by re-integrating
  from the nearest earlier keyframe, and -- because the state is Markovian
  -- reproduces the original trajectory bit-for-bit, with or without a
  torque acting (Section 13.5, 13.1).
"""

import numpy as np
import pytest

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import torque_models as tm
from rigid_body.dynamics import integrators as ig
from rigid_body.dynamics.simulation_engine import advance_one_substep
from rigid_body.dynamics.trajectory import (
    Trajectory, KeyframeStore, RetainedSample)


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


def marker_state(value):
    """A state whose angular velocity encodes an integer, for easy checks."""
    return st.State(
        IDENTITY_QUATERNION.copy(),
        np.array([float(value), 0.0, 0.0]))


# --------------------------------------------------------------------
# The ring buffer (Section 13.4)
# --------------------------------------------------------------------

def test_appends_read_back_in_order_below_capacity():
    trajectory = Trajectory(capacity=5)
    for step in range(3):
        trajectory.append(marker_state(step), float(step))
    assert len(trajectory) == 3
    assert not trajectory.is_full
    for cursor in range(3):
        state, time = trajectory.read(cursor)
        assert state.angular_velocity_body[0] == cursor
        assert time == cursor


def test_full_buffer_overwrites_the_oldest():
    trajectory = Trajectory(capacity=5)
    for step in range(8):
        trajectory.append(marker_state(step), float(step))
    # Only the five most recent survive; cursor 0 is the oldest of those.
    assert len(trajectory) == 5
    assert trajectory.is_full
    oldest_state, oldest_time = trajectory.oldest()
    newest_state, newest_time = trajectory.newest()
    assert oldest_state.angular_velocity_body[0] == 3
    assert oldest_time == 3
    assert newest_state.angular_velocity_body[0] == 7
    assert newest_time == 7


def test_stored_state_is_isolated_from_the_live_state():
    # Section 13.2: the buffer copies the state in, so mutating the source
    # afterward cannot reach into the retained sample.
    trajectory = Trajectory(capacity=3)
    live = marker_state(5)
    trajectory.append(live, 0.0)
    live.angular_velocity_body[0] = -99.0
    live.body_to_space_quaternion[0] = 0.0
    stored_state, _time = trajectory.newest()
    assert stored_state.angular_velocity_body[0] == 5
    assert stored_state.body_to_space_quaternion[0] == 1.0


def test_read_outside_the_window_raises():
    trajectory = Trajectory(capacity=4)
    trajectory.append(marker_state(0), 0.0)
    with pytest.raises(IndexError):
        trajectory.read(1)
    with pytest.raises(IndexError):
        trajectory.read(-1)


def test_time_span_and_containment():
    trajectory = Trajectory(capacity=4)
    for step in range(3):
        trajectory.append(marker_state(step), 0.5 * step)
    assert trajectory.time_span() == (0.0, 1.0)
    assert trajectory.contains_time(0.5)
    assert not trajectory.contains_time(2.0)


def test_read_at_time_finds_the_nearest_in_window_sample():
    trajectory = Trajectory(capacity=6)
    for step in range(4):
        trajectory.append(marker_state(step), 0.5 * step)   # times 0..1.5
    # An exact hit, and two off-grid targets resolving to the nearer side.
    assert trajectory.read_at_time(0.5)[0].angular_velocity_body[0] == 1
    assert trajectory.read_at_time(0.6)[0].angular_velocity_body[0] == 1
    assert trajectory.read_at_time(0.8)[0].angular_velocity_body[0] == 2
    # Past either end clamps to that end.
    assert trajectory.read_at_time(-1.0)[0].angular_velocity_body[0] == 0
    assert trajectory.read_at_time(9.0)[0].angular_velocity_body[0] == 3


def test_read_at_time_on_empty_buffer_raises():
    with pytest.raises(IndexError):
        Trajectory(capacity=3).read_at_time(0.0)


def test_empty_buffer_reports_empty():
    trajectory = Trajectory(capacity=4)
    assert len(trajectory) == 0
    assert trajectory.newest() is None
    assert trajectory.oldest() is None
    assert trajectory.time_span() is None
    assert not trajectory.contains_time(0.0)


def test_capacity_must_be_positive():
    with pytest.raises(ValueError):
        Trajectory(capacity=0)


def test_from_retention_sizes_by_the_limit():
    class _Retention:
        limit_samples = 7

    trajectory = Trajectory.from_retention(_Retention())
    assert trajectory.capacity == 7


def test_monitor_accumulators_ride_along_when_stashed():
    # The one path-dependent exception (Section 13.3): the loop may stash
    # the monitor's running scalars beside a sample.
    trajectory = Trajectory(capacity=3)
    accumulators = {"energy_predicted_change": 1.5}
    trajectory.append(marker_state(0), 0.0, accumulators)
    retained = trajectory.sample(0)
    assert isinstance(retained, RetainedSample)
    assert retained.monitor_accumulators is accumulators


# --------------------------------------------------------------------
# The keyframe store (Section 13.5): exact by the Markov property
# --------------------------------------------------------------------

def reference_run(body, initial_state, torque_models, time_step, steps,
                  stride):
    """Run the engine, recording every state and feeding a keyframe store.

    Returns the full list of exact states (index i is the state after i
    substeps) and a keyframe store that observed only every stride-th one.
    """
    integrator = ig.RungeKutta4Integrator()
    store = KeyframeStore(
        stride, time_step, body, torque_models, integrator)
    states = [initial_state]
    store.observe(0, initial_state, 0.0)
    state = initial_state
    time = 0.0
    for step_index in range(1, steps + 1):
        state = advance_one_substep(
            state, time, time_step, body, torque_models, integrator)
        time = time + time_step
        states.append(state)
        store.observe(step_index, state, time)
    return states, store


@pytest.mark.parametrize("torque_models", [
    [],
    [tm.GravityTorque([0.0, 0.0, -9.81], [0.0, 0.0, 0.2])]],
    ids=["torque_free", "gravity"])
def test_keyframe_reintegration_is_bit_for_bit_exact(torque_models):
    # The heart of Section 13.5: reading any step by re-integrating from a
    # sparse keyframe reproduces the original trajectory exactly, torque or
    # not, because the state is Markovian.
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    initial = st.State(IDENTITY_QUATERNION, np.array([0.05, 2.5, 0.05]))
    states, store = reference_run(
        body, initial, torque_models, time_step=1.0e-3, steps=300,
        stride=25)

    # Targets on and off the keyframe grid, including a keyframe itself.
    for target_step in (0, 1, 24, 25, 26, 137, 249, 300):
        state, time = store.read_step(target_step)
        np.testing.assert_array_equal(
            state.body_to_space_quaternion,
            states[target_step].body_to_space_quaternion)
        np.testing.assert_array_equal(
            state.angular_velocity_body,
            states[target_step].angular_velocity_body)


def test_keyframe_store_keeps_only_stride_multiples():
    body = make_body(shapes.Cube(0.2))
    initial = st.State(IDENTITY_QUATERNION, np.array([1.0, 2.0, 3.0]))
    _states, store = reference_run(
        body, initial, [], time_step=1.0e-3, steps=100, stride=25)
    # Steps 0, 25, 50, 75, 100 -> five keyframes.
    assert store.keyframe_count == 5


def test_read_at_time_lands_on_the_substep_grid():
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    initial = st.State(IDENTITY_QUATERNION, np.array([0.05, 2.5, 0.05]))
    states, store = reference_run(
        body, initial, [], time_step=1.0e-3, steps=200, stride=20)
    # A time between grid points rounds to the nearest step and matches
    # the exact re-integration there.
    by_time_state, _time = store.read_at_time(0.1373)
    by_step_state, _step_time = store.read_step(137)
    np.testing.assert_array_equal(
        by_time_state.angular_velocity_body,
        by_step_state.angular_velocity_body)
    np.testing.assert_array_equal(
        by_time_state.angular_velocity_body,
        states[137].angular_velocity_body)


def test_reintegration_does_not_disturb_the_stored_keyframe():
    # Reading must not mutate the keyframe it started from, so a second
    # read of the same step gives the same answer.
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    initial = st.State(IDENTITY_QUATERNION, np.array([0.05, 2.5, 0.05]))
    _states, store = reference_run(
        body, initial, [], time_step=1.0e-3, steps=120, stride=20)
    first_state, _ = store.read_step(117)
    second_state, _ = store.read_step(117)
    np.testing.assert_array_equal(
        first_state.angular_velocity_body,
        second_state.angular_velocity_body)


def test_missing_initial_keyframe_is_an_error():
    # If step 0 was never observed there is no keyframe before an early
    # target, and the store says so rather than guessing.
    body = make_body(shapes.Cube(0.2))
    store = KeyframeStore(
        10, 1.0e-3, body, [], ig.RungeKutta4Integrator())
    store.observe(10, marker_state(0), 0.010)
    with pytest.raises(LookupError):
        store.read_step(5)


def test_stride_must_be_positive():
    body = make_body(shapes.Cube(0.2))
    with pytest.raises(ValueError):
        KeyframeStore(0, 1.0e-3, body, [], ig.RungeKutta4Integrator())
