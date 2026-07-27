"""Integration tests for the run loop (dynamics/simulation_engine.py).

These exercise the whole physics stack at once -- body, state, Euler's
equations, the integrator, and the engine emitting to sinks. The central
oracle is the determinism guarantee (ARCHITECTURE Sections 6.4 and 8.6):
the same run yields a bit-for-bit identical trajectory, and attaching an
extra sink or a monitor -- both read-only consumers -- changes not a
single computed state.
"""

import numpy as np

from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics.state import (
    State, kinetic_energy, angular_momentum_body)
from rigid_body.dynamics import integrators as ig
from rigid_body.dynamics import simulation_engine as engine
from rigid_body.sinks.sink_interface import Sink


MOMENTS = np.array([2.0, 3.0, 4.0])
TIME_STEP = 0.005
INTEGRATION_SPAN = 0.5


def asymmetric_body():
    top_class, intermediate_axis = classify_top(MOMENTS)
    return RigidBody(
        principal_moments=MOMENTS,
        principal_axes=np.eye(3),
        total_mass=1.0,
        center_of_mass=np.zeros(3),
        top_class=top_class,
        intermediate_axis=intermediate_axis,
        geometry=None)


def fresh_state():
    return State(np.array([1.0, 0.0, 0.0, 0.0]),
                 np.array([0.3, 0.0, 2.0]))


class RecordingSink(Sink):
    """A test-double sink that keeps a copy of every state it receives."""

    def __init__(self):
        self.states = []
        self.times = []
        self.closed = False

    def receive(self, state, time):
        self.states.append(State(
            state.body_to_space_quaternion.copy(),
            state.angular_velocity_body.copy()))
        self.times.append(time)

    def close(self):
        self.closed = True


class CountingMonitor:
    """A read-only monitor double that counts how often it is updated."""

    def __init__(self):
        self.update_count = 0

    def update(self, state, time):
        self.update_count += 1


def run(sinks, monitor=None):
    return engine.run_batch(
        fresh_state(), asymmetric_body(), [],
        ig.RungeKutta4Integrator(), TIME_STEP, INTEGRATION_SPAN,
        sinks, monitor=monitor)


# --------------------------------------------------------------------
# Determinism and read-only consumers
# --------------------------------------------------------------------

def test_two_identical_runs_agree_bit_for_bit():
    first, second = RecordingSink(), RecordingSink()
    run([first])
    run([second])

    assert len(first.states) == len(second.states) > 0
    for state_a, state_b in zip(first.states, second.states):
        assert np.array_equal(
            state_a.angular_velocity_body, state_b.angular_velocity_body)
        assert np.array_equal(
            state_a.body_to_space_quaternion,
            state_b.body_to_space_quaternion)


def test_an_extra_sink_does_not_change_the_trajectory():
    solo = RecordingSink()
    paired = RecordingSink()
    run([solo])
    run([paired, RecordingSink()])
    for state_a, state_b in zip(solo.states, paired.states):
        assert np.array_equal(
            state_a.angular_velocity_body, state_b.angular_velocity_body)


def test_a_monitor_does_not_change_the_trajectory():
    without = RecordingSink()
    with_monitor = RecordingSink()
    monitor = CountingMonitor()
    run([without])
    run([with_monitor], monitor=monitor)

    assert monitor.update_count == len(with_monitor.states)
    for state_a, state_b in zip(without.states, with_monitor.states):
        assert np.array_equal(
            state_a.angular_velocity_body, state_b.angular_velocity_body)


# --------------------------------------------------------------------
# Emit, finalize, and the atomic substep
# --------------------------------------------------------------------

def test_every_sink_receives_every_state_and_is_closed():
    first, second = RecordingSink(), RecordingSink()
    run([first, second])

    assert first.times == second.times
    assert len(first.states) == len(second.states) > 0
    assert first.closed and second.closed


def test_advance_one_substep_matches_a_direct_integrator_call():
    body = asymmetric_body()
    integrator = ig.RungeKutta4Integrator()
    state = fresh_state()

    from rigid_body.dynamics.equations_of_motion import state_derivative
    expected = integrator.advance(
        state, 0.0, TIME_STEP,
        lambda t, s: state_derivative(t, s, body, []))
    actual = engine.advance_one_substep(
        state, 0.0, TIME_STEP, body, [], integrator)

    np.testing.assert_array_equal(
        actual.angular_velocity_body, expected.angular_velocity_body)
    np.testing.assert_array_equal(
        actual.body_to_space_quaternion,
        expected.body_to_space_quaternion)


# --------------------------------------------------------------------
# Physics sanity through the engine
# --------------------------------------------------------------------

def test_torque_free_batch_run_conserves_energy_and_momentum():
    body = asymmetric_body()
    recorder = RecordingSink()
    run([recorder])

    initial = fresh_state()
    energy_0 = kinetic_energy(initial, body)
    momentum_squared_0 = np.sum(angular_momentum_body(initial, body)**2)

    for state in recorder.states:
        energy = kinetic_energy(state, body)
        momentum_squared = np.sum(angular_momentum_body(state, body)**2)
        assert abs(energy - energy_0) / energy_0 < 1.0e-8
        assert abs(momentum_squared - momentum_squared_0) / (
            momentum_squared_0) < 1.0e-8
