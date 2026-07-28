"""Unit tests for the time-control pacing (dynamics/time_control.py).

The whole promise of the time controls is that they change the *pace* of
the display and nothing about the computed motion. The tests pin that:

* Each pace maps to the right substep count, and slow motion never stalls.
* The pace ordering is FAST > NORMAL > SLOW > 0, so the controls do what
  their names say.
* A single step is a one-shot: the interface re-pauses after it.
* The controls carry no ``dt``, so pacing structurally cannot change the
  step size -- the determinism guarantee at the type level.
"""

from types import SimpleNamespace

import numpy as np
import pytest

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import integrators as ig
from rigid_body.dynamics.simulation_engine import advance_one_substep
from rigid_body.dynamics.trajectory import Trajectory, KeyframeStore
from rigid_body.dynamics import time_control as tc


def controls_with(pace, nominal_substeps):
    """A controls snapshot at a given pace and nominal substep count."""
    return tc.Controls(pace=pace, nominal_substeps=nominal_substeps)


# --------------------------------------------------------------------
# The substep map (Section 15.3)
# --------------------------------------------------------------------

def test_each_pace_maps_to_the_expected_substep_count():
    nominal = 8
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.PAUSED, nominal)) == 0
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.SINGLE_STEP, nominal)) == 1
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.SLOW, nominal)) == nominal // tc.SLOW_FACTOR
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.NORMAL, nominal)) == nominal
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.FAST, nominal)) == nominal * tc.FAST_FACTOR


def test_slow_motion_never_stalls_below_one_step():
    # With a nominal count smaller than the slow factor, integer division
    # would give zero; the map floors it at one so slow motion still moves.
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.SLOW, 1)) == 1
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.SLOW, tc.SLOW_FACTOR - 1)) == 1


def test_pace_ordering_is_monotonic():
    nominal = 12
    paused = tc.substeps_this_frame(controls_with(tc.Pace.PAUSED, nominal))
    slow = tc.substeps_this_frame(controls_with(tc.Pace.SLOW, nominal))
    normal = tc.substeps_this_frame(controls_with(tc.Pace.NORMAL, nominal))
    fast = tc.substeps_this_frame(controls_with(tc.Pace.FAST, nominal))
    assert paused < slow < normal < fast


# --------------------------------------------------------------------
# Single-step is a one-shot (Section 15.3)
# --------------------------------------------------------------------

def test_single_step_re_pauses_after_the_frame():
    assert tc.pace_after_frame(tc.Pace.SINGLE_STEP) is tc.Pace.PAUSED


def test_other_paces_persist_across_a_frame():
    for pace in (tc.Pace.PAUSED, tc.Pace.SLOW, tc.Pace.NORMAL,
                 tc.Pace.FAST):
        assert tc.pace_after_frame(pace) is pace


# --------------------------------------------------------------------
# Determinism: pacing cannot reach dt
# --------------------------------------------------------------------

def test_controls_carry_no_step_size():
    # The controls snapshot has no dt/time_step field, so a time control
    # structurally cannot change the step size -- only the substep count
    # (ARCHITECTURE Section 6.3). This is the determinism guarantee made
    # unreachable at the type level.
    controls = tc.Controls()
    assert not hasattr(controls, "dt")
    assert not hasattr(controls, "time_step")


def test_substeps_is_a_pure_function_of_the_controls():
    # Same controls, same answer, with nothing else consulted.
    controls = controls_with(tc.Pace.FAST, 5)
    first = tc.substeps_this_frame(controls)
    second = tc.substeps_this_frame(controls)
    assert first == second == 5 * tc.FAST_FACTOR


# --------------------------------------------------------------------
# default_controls (Section 15.3)
# --------------------------------------------------------------------

def test_default_controls_start_live_at_the_scenario_pace():
    # default_controls only reads the nominal substeps, so a light stub
    # with the right shape is enough to exercise it.
    scenario = SimpleNamespace(
        fidelity=SimpleNamespace(substeps_per_frame=15))
    controls = tc.default_controls(scenario)
    assert controls.mode is tc.ControlMode.LIVE
    assert controls.pace is tc.Pace.NORMAL
    assert controls.nominal_substeps == 15
    assert controls.pending_edit is None
    assert controls.replay_cursor is None
    assert controls.scale_settings == {}
    # And it paces at the nominal rate out of the box.
    assert tc.substeps_this_frame(controls) == 15


# --------------------------------------------------------------------
# The replay scrubber (Section 15.5): composing the two retention policies
# --------------------------------------------------------------------

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


def recorded_run(window, stride, steps=300, time_step=1.0e-3):
    """Run the engine, returning (reference states, ring buffer, keyframes).

    The ring keeps only the last ``window`` samples; the keyframe store
    keeps every ``stride``-th. Index i of the reference is the state after
    i substeps.
    """
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    integrator = ig.RungeKutta4Integrator()
    initial = st.State(
        np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.05, 2.5, 0.05]))

    reference = [initial]
    trajectory = Trajectory(window)
    keyframes = KeyframeStore(stride, time_step, body, [], integrator)
    trajectory.append(initial, 0.0)
    keyframes.observe(0, initial, 0.0)

    state = initial
    time = 0.0
    for step_index in range(1, steps + 1):
        state = advance_one_substep(
            state, time, time_step, body, [], integrator)
        time = time + time_step
        reference.append(state)
        trajectory.append(state, time)
        keyframes.observe(step_index, state, time)
    return reference, trajectory, keyframes


def test_replay_reads_the_recent_past_from_the_ring():
    # A time inside the retained window resolves from the ring buffer, and
    # matches the reference exactly (a pure lookup, no re-integration).
    reference, trajectory, keyframes = recorded_run(window=50, stride=25)
    state, time = tc.replay_state_at(trajectory, keyframes, 0.280)
    np.testing.assert_array_equal(
        state.angular_velocity_body,
        reference[280].angular_velocity_body)
    assert time == pytest.approx(0.280)


def test_replay_reaches_the_deep_past_through_keyframes():
    # A time older than the window falls through to keyframe
    # re-integration, and is still bit-for-bit exact (the Markov property).
    reference, trajectory, keyframes = recorded_run(window=50, stride=25)
    assert not trajectory.contains_time(0.100)   # genuinely out of window
    state, _time = tc.replay_state_at(trajectory, keyframes, 0.100)
    np.testing.assert_array_equal(
        state.angular_velocity_body,
        reference[100].angular_velocity_body)


def test_replay_clamps_a_future_time_to_the_frontier():
    # Replay cannot run ahead of what the engine has produced: a time past
    # the newest retained state returns that newest state, not an
    # extrapolation.
    reference, trajectory, keyframes = recorded_run(window=50, stride=25)
    state, _time = tc.replay_state_at(trajectory, keyframes, 10.0)
    np.testing.assert_array_equal(
        state.angular_velocity_body,
        reference[300].angular_velocity_body)


def test_replay_without_keyframes_cannot_reach_the_deep_past():
    _reference, trajectory, _keyframes = recorded_run(
        window=50, stride=25)
    # In-window still works with no keyframe store...
    tc.replay_state_at(trajectory, None, 0.280)
    # ...but the deep past is unreachable and says so.
    with pytest.raises(LookupError):
        tc.replay_state_at(trajectory, None, 0.100)


def test_replay_is_read_only_across_the_window_and_the_deep_past():
    # Two reads of the same instant give the same answer, so replay has
    # perturbed nothing -- the read-only property at the control level.
    _reference, trajectory, keyframes = recorded_run(window=50, stride=25)
    for target_time in (0.280, 0.100):
        first, _ = tc.replay_state_at(trajectory, keyframes, target_time)
        second, _ = tc.replay_state_at(trajectory, keyframes, target_time)
        np.testing.assert_array_equal(
            first.angular_velocity_body, second.angular_velocity_body)
