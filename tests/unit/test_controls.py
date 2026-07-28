"""Unit tests for the scenario-editing controls (ui/controls.py).

The editing controls must never mutate the running motion: an edit
produces a *new* scenario the engine would rebuild from (Section 15.4), and
the scaling controls must state every exaggeration on screen (Section
15.6). The tests pin both, and confirm the physics/presentation split holds
-- a presentation edit leaves every physics field untouched.
"""

import dataclasses

import numpy as np

from rigid_body.scenario.fidelity import Fidelity
from rigid_body.scenario.scenario import (
    BodySpecification, BodyResolved, Body, InitialConditions,
    Retention, Camera, Presentation, Scenario)
from rigid_body.scenario.serialization import (
    build_body_from_specification)
from rigid_body.dynamics import time_control as tc
from rigid_body.ui import controls as ui


def make_scenario():
    """Assemble a full, self-consistent Scenario in code (torque-free)."""
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
        frame="space", layout="side_by_side", palette="light",
        ellipsoid_scale="inertia", camera=camera,
        scale_factors={"torque": 1.0, "figure_axis_tilt": 1.0})

    return Scenario(
        schema_version="1.0", body=body,
        initial_conditions=conditions,
        torque_models=[],
        fidelity=Fidelity("rk4", 0.005, 20, 0.2),
        retention=Retention(100000), presentation=presentation)


# --------------------------------------------------------------------
# Editing is a new scenario, not a mutation (Section 15.4)
# --------------------------------------------------------------------

def test_apply_edit_returns_a_new_scenario_and_leaves_the_old_intact():
    scenario = make_scenario()
    original_frame = scenario.presentation.frame

    new_presentation = dataclasses.replace(
        scenario.presentation, frame="body")
    edited = ui.apply_scenario_edit(
        scenario, {"presentation": new_presentation})

    # A fresh record with the change applied...
    assert edited is not scenario
    assert edited.presentation.frame == "body"
    # ...and the original is untouched: a new setting is a new run.
    assert scenario.presentation.frame == original_frame


def test_empty_edit_is_a_no_op():
    scenario = make_scenario()
    assert ui.apply_scenario_edit(scenario, {}) is scenario
    assert ui.apply_scenario_edit(scenario, None) is scenario


def test_editing_a_physics_field_changes_the_run():
    # Replacing the initial conditions is a genuinely different run: the
    # edited scenario carries the new spin, the original keeps the old.
    scenario = make_scenario()
    faster_spin = dataclasses.replace(
        scenario.initial_conditions,
        angular_velocity_body=np.array([0.05, 3.0, 0.05]))
    edited = ui.apply_scenario_edit(
        scenario, {"initial_conditions": faster_spin})

    np.testing.assert_array_equal(
        edited.initial_conditions.angular_velocity_body,
        np.array([0.05, 3.0, 0.05]))
    np.testing.assert_array_equal(
        scenario.initial_conditions.angular_velocity_body,
        np.array([0.3, 0.0, 2.0]))


def test_a_presentation_edit_leaves_every_physics_field_identical():
    # The physics/presentation split (Section 15.1): editing what is shown
    # cannot move a computed quantity. Every physics-zone field is the same
    # object after a presentation-only edit.
    scenario = make_scenario()
    dark_presentation = dataclasses.replace(
        scenario.presentation, palette="dark")
    edited = ui.apply_scenario_edit(
        scenario, {"presentation": dark_presentation})

    assert edited.body is scenario.body
    assert edited.initial_conditions is scenario.initial_conditions
    assert edited.torque_models is scenario.torque_models
    assert edited.fidelity is scenario.fidelity
    assert edited.presentation.palette == "dark"


# --------------------------------------------------------------------
# replace_section: the nested-edit convenience
# --------------------------------------------------------------------

def test_replace_section_edits_a_nested_knob_without_mutation():
    scenario = make_scenario()
    edited = ui.replace_section(
        scenario, "fidelity", substeps_per_frame=40)

    assert edited.fidelity.substeps_per_frame == 40
    # The rest of the fidelity record is carried over unchanged.
    assert edited.fidelity.time_step == scenario.fidelity.time_step
    assert edited.fidelity.integrator == scenario.fidelity.integrator
    # The original scenario and its sub-record are untouched.
    assert scenario.fidelity.substeps_per_frame == 20
    assert edited.fidelity is not scenario.fidelity


def test_edited_fidelity_flows_into_the_pacing():
    # Editing the nominal substeps and rebuilding the controls changes the
    # pace -- the editing controls and the time controls meeting up.
    scenario = make_scenario()
    edited = ui.replace_section(
        scenario, "fidelity", substeps_per_frame=40)
    controls = tc.default_controls(edited)
    assert tc.substeps_this_frame(controls) == 40


# --------------------------------------------------------------------
# Honest labels for scaling controls (Section 15.6)
# --------------------------------------------------------------------

def test_scale_labels_name_every_exaggeration():
    labels = ui.scale_setting_labels(
        {"figure_axis_tilt": 5.0, "torque": 2.5})
    assert "figure axis tilt scaled 5x" in labels
    assert "torque scaled 2.5x" in labels


def test_scale_labels_skip_an_unexaggerated_factor():
    # A factor of exactly one moves nothing off its physical value, so it
    # is left unlabeled; anything else must be named (Principle 12).
    labels = ui.scale_setting_labels(
        {"torque": 1.0, "timescale": 10.0})
    assert labels == ["timescale scaled 10x"]


def test_scale_labels_are_empty_when_nothing_is_scaled():
    assert ui.scale_setting_labels({}) == []
    assert ui.scale_setting_labels(
        {"torque": 1.0, "timescale": 1.0}) == []
