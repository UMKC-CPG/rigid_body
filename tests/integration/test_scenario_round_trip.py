"""Integration tests for the scenario schema and serialization.

These exercise the whole scenario path: resolving authored unit strings to
SI, building a body from its specification, writing a scenario to TOML and
reading it back with its resolved values intact, catching a tampered body
summary, and unpacking a scenario into a batch run. The presentation-zone
invariant of DESIGN Section 11.2 is checked directly -- editing a
presentation field changes not a single computed state.
"""

import numpy as np
import pytest

from rigid_body.dynamics.state import State, kinetic_energy
from rigid_body.sinks.sink_interface import Sink
from rigid_body.scenario.fidelity import Fidelity
from rigid_body.scenario.scenario import (
    BodySpecification, BodyResolved, Body, InitialConditions,
    TorqueSpecification, Retention, Camera, Presentation, Scenario)
from rigid_body.scenario.serialization import (
    build_body_from_specification, load_scenario, save_scenario,
    run_batch_from_scenario, resolve_initial_state,
    _rigid_body_from_resolved)


class RecordingSink(Sink):
    def __init__(self):
        self.states = []

    def receive(self, state, time):
        self.states.append(State(
            state.body_to_space_quaternion.copy(),
            state.angular_velocity_body.copy()))

    def close(self):
        pass


def gravity_torque():
    return TorqueSpecification(
        type="gravity",
        authored={"gravity": ["0 m/s^2", "0 m/s^2", "-9.81 m/s^2"],
                  "pivot_lever_arm_body": ["0 m", "0 m", "0.2 m"]},
        resolved={"gravity": np.array([0.0, 0.0, -9.81]),
                  "pivot_lever_arm_body": np.array([0.0, 0.0, 0.2])})


def make_scenario(torque_models=None, frame="space"):
    """Assemble a full, self-consistent Scenario in code."""
    specification = BodySpecification(
        kind="parallelepiped",
        dimensions={"edge_lengths": ["0.10 m", "0.15 m", "0.30 m"]},
        density="2700 kg/m^3")
    body_record = build_body_from_specification(specification)
    resolved = BodyResolved(
        total_mass=body_record.total_mass,
        center_of_mass=body_record.center_of_mass,
        principal_moments=body_record.principal_moments,
        principal_axes=body_record.principal_axes)
    body = Body(specification=specification, resolved=resolved)

    conditions = InitialConditions(
        angular_velocity_authored=["0.3 rad/s", "0.0 rad/s", "2.0 rad/s"],
        angular_velocity_body=np.array([0.3, 0.0, 2.0]),
        orientation_quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        orientation_euler_zxz=["0 deg", "0 deg", "0 deg"])

    camera = Camera(np.array([3.0, 2.0, 1.5]), np.zeros(3),
                    np.array([0.0, 0.0, 1.0]))
    presentation = Presentation(
        frame=frame, layout="side_by_side", palette="default",
        ellipsoid_scale="inertia", camera=camera,
        scale_factors={"torque": 1.0, "figure_axis_tilt": 1.0,
                       "timescale": 1.0})

    return Scenario(
        schema_version="1.0", body=body,
        initial_conditions=conditions,
        torque_models=[] if torque_models is None else torque_models,
        fidelity=Fidelity("rk4", 0.005, 20, 0.2),
        retention=Retention(100000), presentation=presentation)


# --------------------------------------------------------------------
# Unit resolution and body building
# --------------------------------------------------------------------

def test_body_specification_resolves_units_and_builds():
    # 20 cm = 0.2 m; a cube of edge 0.2 m at density 2700.
    specification = BodySpecification(
        kind="cube", dimensions={"edge": "20 cm"}, density="2700 kg/m^3")
    body = build_body_from_specification(specification)
    assert body.total_mass == pytest.approx(2700.0 * 0.2**3)


def test_moment_only_body_needs_no_shape():
    specification = BodySpecification(
        kind="moments",
        dimensions={"moments": ["2 kg*m^2", "3 kg*m^2", "4 kg*m^2"],
                    "mass": "1.5 kg"})
    body = build_body_from_specification(specification)
    np.testing.assert_allclose(body.principal_moments, [2.0, 3.0, 4.0])
    assert body.geometry is None


# --------------------------------------------------------------------
# Round-trip and the consistency check
# --------------------------------------------------------------------

def test_round_trip_preserves_resolved_and_authored(tmp_path):
    scenario = make_scenario(torque_models=[gravity_torque()])
    path = tmp_path / "scenario.toml"
    save_scenario(scenario, path)
    loaded = load_scenario(path)

    np.testing.assert_allclose(
        loaded.body.resolved.principal_moments,
        scenario.body.resolved.principal_moments)
    np.testing.assert_allclose(
        loaded.initial_conditions.orientation_quaternion,
        scenario.initial_conditions.orientation_quaternion)
    assert loaded.fidelity.time_step == scenario.fidelity.time_step
    # Authored strings survive the round-trip for readability.
    assert loaded.body.specification.dimensions["edge_lengths"] == [
        "0.10 m", "0.15 m", "0.30 m"]
    # Torque parameters resolve on load.
    np.testing.assert_allclose(
        loaded.torque_models[0].resolved["gravity"], [0.0, 0.0, -9.81])


def test_reloaded_run_body_carries_its_shape_for_drawing(tmp_path):
    # The dynamics run on the recorded tensor, but the renderer needs the
    # shape to draw the object (DESIGN 11.3). A reloaded shape body must
    # therefore carry its geometry, or the object is never drawn -- while
    # the resolved tensor it runs on is unchanged.
    scenario = make_scenario()
    path = tmp_path / "scenario.toml"
    save_scenario(scenario, path)
    loaded = load_scenario(path)
    body = _rigid_body_from_resolved(loaded.body)
    assert body.geometry is not None
    np.testing.assert_allclose(
        body.principal_moments, loaded.body.resolved.principal_moments)


def test_reloaded_moments_only_body_has_no_shape(tmp_path):
    # A body given by moments alone has no shape; its ellipsoid is the proxy.
    specification = BodySpecification(
        kind="moments",
        dimensions={"moments": ["2 kg*m^2", "3 kg*m^2", "4 kg*m^2"],
                    "mass": "1.5 kg"})
    record = build_body_from_specification(specification)
    resolved = BodyResolved(
        total_mass=record.total_mass,
        center_of_mass=record.center_of_mass,
        principal_moments=record.principal_moments,
        principal_axes=record.principal_axes)
    body = Body(specification=specification, resolved=resolved)
    assert _rigid_body_from_resolved(body).geometry is None


def test_consistency_check_catches_a_tampered_body(tmp_path):
    scenario = make_scenario()
    scenario.body.resolved.principal_moments = (
        scenario.body.resolved.principal_moments * 2.0)
    path = tmp_path / "bad.toml"
    save_scenario(scenario, path)
    with pytest.raises(ValueError):
        load_scenario(path)


# --------------------------------------------------------------------
# Running a scenario
# --------------------------------------------------------------------

def test_run_batch_from_scenario_conserves_energy():
    scenario = make_scenario()
    recorder = RecordingSink()
    run_batch_from_scenario(scenario, [recorder])

    assert len(recorder.states) > 0
    body = _rigid_body_from_resolved(scenario.body)
    energy_0 = kinetic_energy(resolve_initial_state(scenario), body)
    for state in recorder.states:
        assert abs(kinetic_energy(state, body) - energy_0) / energy_0 < (
            1.0e-6)


def test_editing_the_presentation_zone_does_not_change_the_trajectory():
    from_space = RecordingSink()
    from_body = RecordingSink()
    run_batch_from_scenario(make_scenario(frame="space"), [from_space])
    run_batch_from_scenario(make_scenario(frame="body"), [from_body])

    for state_a, state_b in zip(from_space.states, from_body.states):
        assert np.array_equal(
            state_a.angular_velocity_body, state_b.angular_velocity_body)
