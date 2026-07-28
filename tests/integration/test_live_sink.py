"""Integration tests for the live sink (sinks/live_sink.py).

The live sink is the interactive tier's end of the sink boundary, so the
tests pin the two properties that make it correct there:

* It faithfully carries the engine's state stream: it captures the latest
  state, counts what it received, and fires its per-state and close hooks
  the right number of times.
* It is read-only -- it never mutates a state, and attaching it to a batch
  run leaves the trajectory byte-for-byte unchanged (the determinism
  guarantee), which is what lets the display be toggled without touching
  the physics.
"""

import numpy as np

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import integrators as ig
from rigid_body.dynamics import simulation_engine as engine
from rigid_body.sinks.sink_interface import Sink
from rigid_body.sinks.live_sink import LiveSink


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


# --------------------------------------------------------------------
# It is a sink, and it captures the stream
# --------------------------------------------------------------------

def test_is_a_sink_and_starts_empty():
    sink = LiveSink()
    assert isinstance(sink, Sink)
    assert not sink.has_state()
    assert sink.latest() is None
    assert sink.received_count == 0


def test_receive_captures_the_latest_state_and_time():
    sink = LiveSink()
    first = st.State(IDENTITY_QUATERNION, np.array([1.0, 0.0, 0.0]))
    second = st.State(IDENTITY_QUATERNION, np.array([2.0, 0.0, 0.0]))
    sink.receive(first, 0.1)
    sink.receive(second, 0.2)

    assert sink.received_count == 2
    assert sink.has_state()
    latest_state, latest_time = sink.latest()
    # The most recent state wins, and it is the very object received.
    assert latest_state is second
    assert latest_time == 0.2


def test_on_state_hook_fires_for_every_state():
    seen = []
    sink = LiveSink(on_state=lambda state, time: seen.append(
        (state.angular_velocity_body[0], time)))
    sink.receive(st.State(IDENTITY_QUATERNION, np.array([5.0, 0.0, 0.0])),
                 0.5)
    sink.receive(st.State(IDENTITY_QUATERNION, np.array([6.0, 0.0, 0.0])),
                 0.6)
    assert seen == [(5.0, 0.5), (6.0, 0.6)]


def test_on_close_hook_fires_once():
    closes = []
    sink = LiveSink(on_close=lambda: closes.append(True))
    sink.close()
    sink.close()   # a second close must not fire the hook again
    assert closes == [True]


def test_hooks_are_optional():
    # With no hooks the sink is a pure latest-state capture; receiving and
    # closing must both work without error.
    sink = LiveSink()
    sink.receive(st.State(IDENTITY_QUATERNION, np.array([1.0, 0.0, 0.0])),
                 0.0)
    sink.close()
    assert sink.received_count == 1


# --------------------------------------------------------------------
# Read-only: it cannot disturb the physics
# --------------------------------------------------------------------

def test_receive_does_not_mutate_the_state():
    sink = LiveSink()
    quaternion = IDENTITY_QUATERNION.copy()
    omega = np.array([1.0, -2.0, 0.5])
    state = st.State(quaternion.copy(), omega.copy())
    sink.receive(state, 0.0)
    np.testing.assert_array_equal(
        state.body_to_space_quaternion, quaternion)
    np.testing.assert_array_equal(state.angular_velocity_body, omega)


# --------------------------------------------------------------------
# Through the engine's emit path
# --------------------------------------------------------------------

def test_engine_feeds_every_substep_and_latest_is_the_final_state():
    body = make_body(ASYMMETRIC_BOX)
    initial = st.State(IDENTITY_QUATERNION, np.array([0.1, 2.0, 0.1]))
    seen_times = []
    sink = LiveSink(on_state=lambda state, time: seen_times.append(time))

    final = engine.run_batch(
        initial, body, [], ig.RungeKutta4Integrator(), 1.0e-3, 0.05,
        [sink])

    # 0.05 s at dt = 1e-3 -> 50 substeps, one emit each.
    assert sink.received_count == 50
    assert len(seen_times) == 50
    latest_state, latest_time = sink.latest()
    np.testing.assert_array_equal(
        latest_state.angular_velocity_body, final.angular_velocity_body)
    assert latest_time == seen_times[-1]


def test_attaching_the_live_sink_does_not_change_the_trajectory():
    body = make_body(ASYMMETRIC_BOX)
    initial = st.State(IDENTITY_QUATERNION, np.array([0.1, 2.0, 0.1]))
    integrator = ig.RungeKutta4Integrator()

    without = engine.run_batch(
        initial, body, [], integrator, 1.0e-3, 0.05, [])
    with_live = engine.run_batch(
        initial, body, [], integrator, 1.0e-3, 0.05, [LiveSink()])

    np.testing.assert_array_equal(
        without.body_to_space_quaternion,
        with_live.body_to_space_quaternion)
    np.testing.assert_array_equal(
        without.angular_velocity_body,
        with_live.angular_velocity_body)
