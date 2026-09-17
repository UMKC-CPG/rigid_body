"""Integration tests for the batch entry script (src/scripts/rbbatch.py).

The batch script ties the whole Tier-2 path together -- load a scenario,
run it deterministically, write HDF5 with an XDMF companion and embedded
provenance -- so the tests exercise that path end to end through the
script's testable core, ``run_batch_job``:

* The run writes the expected number of samples and reports them.
* The output is self-describing: the scenario is embedded as provenance
  and reloads to the same initial condition.
* The XDMF companion and the conservation report can each be switched off.
* The run is deterministic: the same scenario yields byte-for-byte
  identical output.
"""

import os

import numpy as np

import h5py

from rigid_body.scenario.fidelity import Fidelity
from rigid_body.scenario.scenario import (
    BodySpecification, BodyResolved, Body, InitialConditions,
    Retention, Camera, Presentation, Scenario)
from rigid_body.scenario.serialization import (
    build_body_from_specification, save_scenario, load_scenario)
from rigid_body.sinks import hdf5_sink as h5sink


# The command's body is a module in the package (ARCHITECTURE 3.9);
# the script in src/scripts/ is only a front for it.
from rigid_body.cli import rbbatch  # noqa: E402


def write_scenario(path, torque_models=None):
    """Write a small, self-consistent torque-free scenario to ``path``."""
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
        angular_velocity_authored=[
            "0.05 rad/s", "2.5 rad/s", "0.05 rad/s"],
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
        initial_conditions=conditions,
        torque_models=[] if torque_models is None else torque_models,
        fidelity=Fidelity("rk4", 0.001, 10, 0.1),
        retention=Retention(100000), presentation=presentation)
    save_scenario(scenario, path)
    return scenario


# --------------------------------------------------------------------
# The end-to-end run
# --------------------------------------------------------------------

def test_run_writes_the_expected_trajectory(tmp_path):
    scenario_path = str(tmp_path / "spin.toml")
    output_path = str(tmp_path / "spin.h5")
    write_scenario(scenario_path)

    result = rbbatch.run_batch_job(scenario_path, output_path)

    # 0.1 s span at dt = 0.001 -> 100 substeps, one sample each.
    assert result.sample_count == 100
    assert result.output_path == output_path
    with h5py.File(output_path, "r") as handle:
        assert handle[h5sink.TIME_DATASET].shape == (100,)
        assert handle.attrs["sample_count"] == 100


def test_output_embeds_reloadable_provenance(tmp_path):
    scenario_path = str(tmp_path / "spin.toml")
    output_path = str(tmp_path / "spin.h5")
    write_scenario(scenario_path)
    rbbatch.run_batch_job(scenario_path, output_path)

    with h5py.File(output_path, "r") as handle:
        embedded = handle.attrs["scenario_toml"]

    # The embedded provenance is a complete scenario: writing it back out
    # and reloading it reproduces the run's initial condition.
    reloaded_path = str(tmp_path / "reloaded.toml")
    with open(reloaded_path, "w") as reloaded_file:
        reloaded_file.write(embedded)
    reloaded = load_scenario(reloaded_path)
    np.testing.assert_array_equal(
        reloaded.initial_conditions.angular_velocity_body,
        np.array([0.05, 2.5, 0.05]))
    assert reloaded.fidelity.time_step == 0.001


def test_conservation_report_is_present_and_tiny(tmp_path):
    # A torque-free run conserves energy and momentum, so the reported
    # drift trend is essentially machine zero.
    scenario_path = str(tmp_path / "spin.toml")
    write_scenario(scenario_path)
    result = rbbatch.run_batch_job(
        scenario_path, str(tmp_path / "spin.h5"))

    assert result.drift_trend is not None
    assert abs(result.drift_trend.energy_growth_rate) < 1e-9
    assert abs(result.drift_trend.momentum_growth_rate) < 1e-9


# --------------------------------------------------------------------
# The switches
# --------------------------------------------------------------------

def test_xdmf_can_be_suppressed(tmp_path):
    scenario_path = str(tmp_path / "spin.toml")
    output_path = str(tmp_path / "spin.h5")
    write_scenario(scenario_path)

    with_xdmf = rbbatch.run_batch_job(
        scenario_path, output_path, write_xdmf=True)
    assert with_xdmf.xdmf_path is not None
    assert os.path.exists(with_xdmf.xdmf_path)

    output_two = str(tmp_path / "spin_two.h5")
    without = rbbatch.run_batch_job(
        scenario_path, output_two, write_xdmf=False)
    assert without.xdmf_path is None
    assert not os.path.exists(h5sink._xdmf_path_for(output_two))


def test_monitor_can_be_disabled(tmp_path):
    scenario_path = str(tmp_path / "spin.toml")
    write_scenario(scenario_path)
    result = rbbatch.run_batch_job(
        scenario_path, str(tmp_path / "spin.h5"), enable_monitor=False)
    assert result.drift_trend is None


def test_default_output_path_uses_the_scenario_stem():
    path = rbbatch.default_output_path("/runs/dzhanibekov.toml", "/out")
    assert path == os.path.join("/out", "dzhanibekov.h5")


# --------------------------------------------------------------------
# Determinism
# --------------------------------------------------------------------

def test_two_runs_are_byte_for_byte_identical(tmp_path):
    # The whole point of the batch tier: the same scenario reproduces the
    # same trajectory, so two runs write identical data (ARCH 6.4).
    scenario_path = str(tmp_path / "spin.toml")
    write_scenario(scenario_path)
    first_path = str(tmp_path / "first.h5")
    second_path = str(tmp_path / "second.h5")

    rbbatch.run_batch_job(scenario_path, first_path)
    rbbatch.run_batch_job(scenario_path, second_path)

    with h5py.File(first_path, "r") as first, \
            h5py.File(second_path, "r") as second:
        np.testing.assert_array_equal(
            first[h5sink.ORIENTATION_DATASET][:],
            second[h5sink.ORIENTATION_DATASET][:])
        np.testing.assert_array_equal(
            first[h5sink.ANGULAR_VELOCITY_DATASET][:],
            second[h5sink.ANGULAR_VELOCITY_DATASET][:])
