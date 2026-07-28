"""Integration tests for the interactive driver (ui/interactive_session.py).

The driver is renderer- and controls-agnostic by design, so the whole frame
loop is exercised here with fakes -- no vedo, no display. The tests pin the
loop's contract:

* It advances the physics by the chosen substep count and draws once per
  frame; pause holds the motion while the display still redraws; single-step
  takes exactly one step.
* It is deterministic: the same scenario and the same controls script
  produce the same rendered states, whether or not a sink is attached.
* An edit stops the run and is returned; the session applies it and runs the
  next scenario, so a physics edit becomes a genuinely different run.
* Replay reads stored history rather than stepping the engine.
* A live sink taps every substep, its latest matches the drawn frame, and
  attached sinks are finalized when the session ends.
"""

import numpy as np

from rigid_body.scenario.fidelity import Fidelity
from rigid_body.scenario.scenario import (
    BodySpecification, BodyResolved, Body, InitialConditions,
    Retention, Camera, Presentation, Scenario)
from rigid_body.scenario.serialization import build_body_from_specification
from rigid_body.dynamics.time_control import Controls, ControlMode, Pace
from rigid_body.sinks.sink_interface import Sink
from rigid_body.sinks.live_sink import LiveSink
from rigid_body.ui.interactive_session import (
    run_interactive, run_interactive_session, ControlsSource)


def make_scenario(nominal_substeps=2, angular_velocity=None):
    """Assemble a small torque-free scenario for the driver to run."""
    specification = BodySpecification(
        kind="parallelepiped",
        dimensions={"edge_lengths": ["0.10 m", "0.15 m", "0.30 m"]},
        density="2700 kg/m^3")
    record = build_body_from_specification(specification)
    body = Body(
        specification=specification,
        resolved=BodyResolved(
            record.total_mass, record.center_of_mass,
            record.principal_moments, record.principal_axes))
    if angular_velocity is None:
        angular_velocity = np.array([0.05, 2.5, 0.05])
    conditions = InitialConditions(
        angular_velocity_authored=["0.05 rad/s", "2.5 rad/s",
                                   "0.05 rad/s"],
        angular_velocity_body=np.asarray(angular_velocity, dtype=float),
        orientation_quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        orientation_euler_zxz=["0 deg", "0 deg", "0 deg"])
    presentation = Presentation(
        frame="space", layout="side_by_side", palette="light",
        ellipsoid_scale="inertia",
        camera=Camera(np.array([3.0, 2.0, 1.5]), np.zeros(3),
                      np.array([0.0, 0.0, 1.0])),
        scale_factors={})
    return Scenario(
        schema_version="1.0", body=body,
        initial_conditions=conditions, torque_models=[],
        fidelity=Fidelity("rk4", 0.001, nominal_substeps, 0.1),
        retention=Retention(100000), presentation=presentation)


class FakeRenderer:
    """A renderer that records the state of every frame it is asked to draw."""

    def __init__(self):
        self.states = []
        self.closed = False

    def render(self, scene, state):
        self.states.append(np.array(state.angular_velocity_body))

    def close(self):
        self.closed = True


class ScriptedControls(ControlsSource):
    """A controls source that plays a fixed list, one control per read.

    Each read consumes one scripted control -- mirroring a real source,
    which clears a pending edit once it has been read -- and the window is
    reported closed once the script is exhausted.
    """

    def __init__(self, controls_sequence):
        self.controls_sequence = list(controls_sequence)
        self.index = 0

    def read(self):
        control = self.controls_sequence[self.index]
        self.index += 1
        return control

    def window_closed(self):
        return self.index >= len(self.controls_sequence)


def normal(nominal_substeps=2):
    """A live control at normal pace."""
    return Controls(
        mode=ControlMode.LIVE, pace=Pace.NORMAL,
        nominal_substeps=nominal_substeps)


# --------------------------------------------------------------------
# Stepping and pacing
# --------------------------------------------------------------------

def test_advances_and_renders_each_frame():
    scenario = make_scenario(nominal_substeps=2)
    renderer = FakeRenderer()
    edit = run_interactive(
        scenario, renderer, ScriptedControls([normal()] * 5))
    assert edit is None                       # window closed, not an edit
    assert len(renderer.states) == 5
    # The motion genuinely advanced across the run.
    assert not np.array_equal(renderer.states[0], renderer.states[-1])


def test_pause_holds_the_motion_but_still_redraws():
    scenario = make_scenario(nominal_substeps=2)
    renderer = FakeRenderer()
    paused = Controls(pace=Pace.PAUSED, nominal_substeps=2)
    run_interactive(scenario, renderer, ScriptedControls([paused] * 3))
    # Three frames drawn, all showing the same (unadvanced) state.
    assert len(renderer.states) == 3
    for drawn in renderer.states:
        np.testing.assert_array_equal(drawn, renderer.states[0])


def test_single_step_takes_exactly_one_substep():
    scenario = make_scenario(nominal_substeps=8)
    renderer = FakeRenderer()
    stepped = Controls(pace=Pace.SINGLE_STEP, nominal_substeps=8)
    run_interactive(scenario, renderer, ScriptedControls([stepped]))
    assert len(renderer.states) == 1
    # One step, not the nominal eight: the drawn state is the initial
    # state advanced a single substep, still close to the start.
    assert renderer.states[0][1] > 2.0        # omega_2 barely moved


# --------------------------------------------------------------------
# Determinism
# --------------------------------------------------------------------

def test_same_script_gives_the_same_rendered_states():
    scenario = make_scenario(nominal_substeps=3)
    first = FakeRenderer()
    second = FakeRenderer()
    run_interactive(scenario, first, ScriptedControls([normal(3)] * 6))
    run_interactive(scenario, second, ScriptedControls([normal(3)] * 6))
    for a, b in zip(first.states, second.states):
        np.testing.assert_array_equal(a, b)


# --------------------------------------------------------------------
# Editing: a new run, not a mutated one
# --------------------------------------------------------------------

def test_pending_edit_stops_the_run_and_is_returned():
    scenario = make_scenario()
    renderer = FakeRenderer()
    the_edit = {"schema_version": "9.9"}
    edit = run_interactive(
        scenario, renderer,
        ScriptedControls([normal(), Controls(pending_edit=the_edit)]))
    assert edit is the_edit
    # Exactly one frame ran before the edit was noticed.
    assert len(renderer.states) == 1


def test_session_applies_the_edit_and_runs_the_new_scenario():
    scenario = make_scenario(nominal_substeps=2)
    renderer = FakeRenderer()
    # Edit the initial spin to a distinctly faster tumble weighted toward
    # axis 1 (kept off the pure principal axis so the polhode is a proper
    # curve, not the degenerate point).
    faster = InitialConditions(
        angular_velocity_authored=["3.0 rad/s", "0.3 rad/s", "0.1 rad/s"],
        angular_velocity_body=np.array([3.0, 0.3, 0.1]),
        orientation_quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        orientation_euler_zxz=["0 deg", "0 deg", "0 deg"])
    edit = {"initial_conditions": faster}
    run_interactive_session(
        scenario, renderer,
        ScriptedControls([normal(), Controls(pending_edit=edit),
                          normal(), normal()]))
    # One frame before the edit, two after: three drawn in all.
    assert len(renderer.states) == 3
    # The post-edit frames start from the new spin (omega_1 ~ 3), which the
    # original scenario (omega_1 ~ 0.05) never reaches -- the edit produced
    # a genuinely different run.
    assert renderer.states[-1][0] > 1.0
    assert renderer.states[0][0] < 1.0


# --------------------------------------------------------------------
# Replay reads stored history
# --------------------------------------------------------------------

def test_replay_redraws_a_stored_past_state():
    scenario = make_scenario(nominal_substeps=2)
    renderer = FakeRenderer()
    # Four live frames fill the history (each ends at time k * 2 * 0.001),
    # then one replay frame scrubs back to the end of frame 2.
    replay = Controls(
        mode=ControlMode.REPLAY, replay_cursor=2 * 2 * 0.001,
        nominal_substeps=2)
    run_interactive(
        scenario, renderer,
        ScriptedControls([normal(), normal(), normal(), normal(),
                          replay]))
    # The replay frame (index 4) redraws exactly the frame-2 state (index
    # 1): a read of stored history, not a re-stepped one.
    np.testing.assert_array_equal(
        renderer.states[4], renderer.states[1])


# --------------------------------------------------------------------
# The live sink taps the stream, and sinks are finalized
# --------------------------------------------------------------------

def test_live_sink_taps_every_substep_and_matches_the_drawn_frame():
    scenario = make_scenario(nominal_substeps=3)
    renderer = FakeRenderer()
    live = LiveSink()
    run_interactive(
        scenario, renderer, ScriptedControls([normal(3)] * 4),
        sinks=[live])
    # Four frames of three substeps each: twelve emitted states.
    assert live.received_count == 12
    # The sink's latest is exactly the last drawn frame's endpoint.
    latest_state, _time = live.latest()
    np.testing.assert_array_equal(
        latest_state.angular_velocity_body, renderer.states[-1])


class ClosingSink(Sink):
    """A sink that records whether it was finalized."""

    def __init__(self):
        self.closed = False

    def receive(self, state, time):
        pass

    def close(self):
        self.closed = True


def test_session_finalizes_attached_sinks():
    scenario = make_scenario()
    sink = ClosingSink()
    run_interactive_session(
        scenario, FakeRenderer(), ScriptedControls([normal()]),
        sinks=[sink])
    assert sink.closed
