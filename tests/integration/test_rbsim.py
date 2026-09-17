"""Integration tests for the interactive entry script (src/scripts/rbsim.py).

The interactive path is graphics-bound, but its wiring is not: the driver is
renderer- and controls-agnostic, so ``run_interactive_job`` is exercised
here with a fake renderer and the input-free auto controls -- no vedo, no
display -- and one guarded test drives the real offscreen renderer end to
end where a GL context exists.
"""

import os

import numpy as np
import pytest

from rigid_body.scenario.fidelity import Fidelity
from rigid_body.scenario.scenario import (
    BodySpecification, BodyResolved, Body, InitialConditions,
    Retention, Camera, Presentation, Scenario)
from rigid_body.scenario.serialization import (
    build_body_from_specification, save_scenario)
from rigid_body.ui.vedo_controls import AutoControlsSource


# The command's body is a module in the package (ARCHITECTURE 3.9);
# the script in src/scripts/ is only a front for it.
from rigid_body.cli import rbsim  # noqa: E402


def write_scenario(path):
    """Write a small torque-free scenario for the interactive tier to run."""
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
    conditions = InitialConditions(
        angular_velocity_authored=["0.05 rad/s", "2.5 rad/s",
                                   "0.05 rad/s"],
        angular_velocity_body=np.array([0.05, 2.5, 0.05]),
        orientation_quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        orientation_euler_zxz=["0 deg", "0 deg", "0 deg"])
    presentation = Presentation(
        frame="space", layout="side_by_side", palette="light",
        ellipsoid_scale="inertia",
        camera=Camera(np.array([3.0, 2.0, 1.5]), np.zeros(3),
                      np.array([0.0, 0.0, 1.0])),
        scale_factors={})
    scenario = Scenario(
        schema_version="1.0", body=body,
        initial_conditions=conditions, torque_models=[],
        fidelity=Fidelity("rk4", 0.002, 5, 0.2),
        retention=Retention(100000), presentation=presentation)
    save_scenario(scenario, path)


class FakeRenderer:
    """A renderer that records the state of every frame it draws."""

    def __init__(self):
        self.states = []
        self.closed = False

    def render(self, scene, state):
        self.states.append(np.array(state.angular_velocity_body))

    def close(self):
        self.closed = True


# --------------------------------------------------------------------
# The wiring, with fakes
# --------------------------------------------------------------------

def test_job_renders_the_requested_frames_and_advances(tmp_path):
    scenario_path = str(tmp_path / "spin.toml")
    write_scenario(scenario_path)
    renderer = FakeRenderer()
    controls = AutoControlsSource(nominal_substeps=5, max_frames=4)

    rbsim.run_interactive_job(
        scenario_path, renderer=renderer, controls_source=controls)

    assert len(renderer.states) == 4
    assert controls.frames_read == 4
    # The motion advanced across the run.
    assert not np.array_equal(renderer.states[0], renderer.states[-1])


def test_job_run_is_deterministic(tmp_path):
    scenario_path = str(tmp_path / "spin.toml")
    write_scenario(scenario_path)
    first = FakeRenderer()
    second = FakeRenderer()
    rbsim.run_interactive_job(
        scenario_path, renderer=first,
        controls_source=AutoControlsSource(5, max_frames=5))
    rbsim.run_interactive_job(
        scenario_path, renderer=second,
        controls_source=AutoControlsSource(5, max_frames=5))
    for a, b in zip(first.states, second.states):
        np.testing.assert_array_equal(a, b)


def test_an_injected_renderer_is_left_open_for_the_caller(tmp_path):
    # When the caller passes its own renderer, the job must not close it --
    # the caller owns its lifecycle (and may screenshot it afterward).
    scenario_path = str(tmp_path / "spin.toml")
    write_scenario(scenario_path)
    renderer = FakeRenderer()
    rbsim.run_interactive_job(
        scenario_path, renderer=renderer,
        controls_source=AutoControlsSource(5, max_frames=2))
    assert not renderer.closed


# --------------------------------------------------------------------
# Palette resolution
# --------------------------------------------------------------------

def test_resolve_palette_known_and_unknown():
    from rigid_body.render import palettes as pal
    assert rbsim.resolve_palette("dark").name == "dark"
    # An unknown palette falls back to light rather than refusing to open.
    assert rbsim.resolve_palette("chartreuse") is pal.LIGHT_PALETTE


# --------------------------------------------------------------------
# The real offscreen renderer, end to end (skips without GL)
# --------------------------------------------------------------------

def test_offscreen_run_completes_with_the_real_renderer(tmp_path):
    scenario_path = str(tmp_path / "spin.toml")
    write_scenario(scenario_path)
    try:
        controls = rbsim.run_interactive_job(
            scenario_path, window_size=(640, 480), offscreen=True,
            frames=3)
    except Exception as problem:                  # pragma: no cover
        pytest.skip(f"no offscreen render context: {problem}")
    assert controls.frames_read == 3


def test_save_frames_writes_one_png_per_frame(tmp_path):
    # The headless capture path: a PNG per frame plus a final screenshot,
    # so a scenario can be seen on a node with no display.
    scenario_path = str(tmp_path / "spin.toml")
    write_scenario(scenario_path)
    frames_dir = str(tmp_path / "frames")
    final_png = str(tmp_path / "final.png")
    try:
        rbsim.run_interactive_job(
            scenario_path, window_size=(320, 240), frames=4,
            save_frames=frames_dir, screenshot=final_png)
    except Exception as problem:                  # pragma: no cover
        pytest.skip(f"no offscreen render context: {problem}")
    written = [name for name in os.listdir(frames_dir)
               if name.endswith(".png")]
    assert len(written) == 4
    assert os.path.exists(final_png)
