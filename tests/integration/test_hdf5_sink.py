"""Integration tests for the HDF5 recording sink (sinks/hdf5_sink.py).

The sink is the batch tier's whole output path, so the tests check that it
preserves the stream faithfully and cannot perturb the physics:

* What lands on disk is exactly what the engine emitted -- byte-for-byte,
  including across several buffer flushes and a partial final one.
* Attaching the sink does not change the trajectory (it is read-only).
* The file is self-describing: schema, units, quaternion convention, the
  embedded scenario provenance, and the sample count are all present.
* An empty run and a double close are both handled cleanly.
* The XDMF companion is well-formed, has one timestep per sample, and
  points back at the HDF5 datasets with matching times.
"""

import os
import xml.etree.ElementTree as ElementTree

import numpy as np

import h5py

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import integrators as ig
from rigid_body.dynamics import simulation_engine as engine
from rigid_body.sinks.sink_interface import Sink
from rigid_body.sinks import hdf5_sink as h5sink
from rigid_body.sinks.hdf5_sink import Hdf5Sink


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


class RecordingSink(Sink):
    """A trivial in-memory sink that keeps a copy of every emitted state.

    Used as the reference the HDF5 file is compared against: whatever this
    collects, the file must reproduce exactly.
    """

    def __init__(self):
        self.times = []
        self.orientations = []
        self.angular_velocities = []

    def receive(self, state, time):
        self.times.append(time)
        self.orientations.append(
            np.array(state.body_to_space_quaternion, dtype=float))
        self.angular_velocities.append(
            np.array(state.angular_velocity_body, dtype=float))


# --------------------------------------------------------------------
# What lands on disk is exactly what the engine emitted
# --------------------------------------------------------------------

def test_disk_matches_the_emitted_stream(tmp_path):
    body = make_body(ASYMMETRIC_BOX)
    initial = st.State(IDENTITY_QUATERNION, np.array([0.05, 2.5, 0.05]))
    path = str(tmp_path / "trajectory.h5")

    # A small flush size forces several flushes plus a partial final one,
    # exercising the resizable-append path rather than a single write.
    reference = RecordingSink()
    recorder = Hdf5Sink(path, flush_every=64)
    engine.run_batch(
        initial, body, [], ig.RungeKutta4Integrator(), 1.0e-3, 0.5,
        [reference, recorder])

    with h5py.File(path, "r") as handle:
        disk_time = handle[h5sink.TIME_DATASET][:]
        disk_orientation = handle[h5sink.ORIENTATION_DATASET][:]
        disk_omega = handle[h5sink.ANGULAR_VELOCITY_DATASET][:]

    # Byte-for-byte: float64 round-trips through HDF5 without loss.
    np.testing.assert_array_equal(
        disk_time, np.array(reference.times))
    np.testing.assert_array_equal(
        disk_orientation, np.array(reference.orientations))
    np.testing.assert_array_equal(
        disk_omega, np.array(reference.angular_velocities))


def test_hand_fed_states_round_trip_exactly(tmp_path):
    # No integrator in the loop: feed known states straight in and read
    # them back, so any corruption would show as an exact-equality break.
    path = str(tmp_path / "hand.h5")
    states = [
        st.State(np.array([1.0, 0.0, 0.0, 0.0]),
                 np.array([0.1, 0.2, 0.3])),
        st.State(np.array([0.0, 1.0, 0.0, 0.0]),
                 np.array([-1.0, 4.0, 2.5])),
        st.State(np.array([0.5, 0.5, 0.5, 0.5]),
                 np.array([7.0, -3.0, 0.0]))]
    times = [0.0, 0.25, 0.5]

    recorder = Hdf5Sink(path, flush_every=2)
    for state, time in zip(states, times):
        recorder.receive(state, time)
    recorder.close()

    with h5py.File(path, "r") as handle:
        np.testing.assert_array_equal(
            handle[h5sink.TIME_DATASET][:], np.array(times))
        np.testing.assert_array_equal(
            handle[h5sink.ORIENTATION_DATASET][:],
            np.array([s.body_to_space_quaternion for s in states]))
        np.testing.assert_array_equal(
            handle[h5sink.ANGULAR_VELOCITY_DATASET][:],
            np.array([s.angular_velocity_body for s in states]))
        assert handle.attrs["sample_count"] == 3


# --------------------------------------------------------------------
# Read-only: the sink cannot change the trajectory
# --------------------------------------------------------------------

def test_recording_does_not_change_the_trajectory(tmp_path):
    body = make_body(ASYMMETRIC_BOX)
    initial = st.State(IDENTITY_QUATERNION, np.array([0.1, 2.0, 0.1]))
    integrator = ig.RungeKutta4Integrator()

    without = engine.run_batch(
        initial, body, [], integrator, 1.0e-3, 0.5, [])
    recorder = Hdf5Sink(str(tmp_path / "run.h5"))
    with_sink = engine.run_batch(
        initial, body, [], integrator, 1.0e-3, 0.5, [recorder])

    np.testing.assert_array_equal(
        without.body_to_space_quaternion,
        with_sink.body_to_space_quaternion)
    np.testing.assert_array_equal(
        without.angular_velocity_body,
        with_sink.angular_velocity_body)


# --------------------------------------------------------------------
# The file is self-describing
# --------------------------------------------------------------------

def test_provenance_and_metadata_are_recorded(tmp_path):
    path = str(tmp_path / "meta.h5")
    scenario_toml = "[body]\nname = \"box\"\n"
    recorder = Hdf5Sink(
        path, scenario_toml=scenario_toml,
        attributes={"created_by": "test"})
    recorder.receive(
        st.State(IDENTITY_QUATERNION, np.array([1.0, 0.0, 0.0])), 0.0)
    recorder.close()

    with h5py.File(path, "r") as handle:
        assert handle.attrs["format_version"] == h5sink.FORMAT_VERSION
        assert "SI" in handle.attrs["units"]
        assert "scalar-first" in handle.attrs["quaternion_convention"]
        assert handle.attrs["scenario_toml"] == scenario_toml
        assert handle.attrs["created_by"] == "test"
        assert handle.attrs["sample_count"] == 1


# --------------------------------------------------------------------
# Edge cases: empty run and double close
# --------------------------------------------------------------------

def test_empty_run_writes_valid_empty_datasets(tmp_path):
    path = str(tmp_path / "empty.h5")
    recorder = Hdf5Sink(path)
    recorder.close()

    with h5py.File(path, "r") as handle:
        assert handle[h5sink.TIME_DATASET].shape == (0,)
        assert handle[h5sink.ORIENTATION_DATASET].shape == (0, 4)
        assert handle.attrs["sample_count"] == 0
    assert os.path.exists(h5sink._xdmf_path_for(path))


def test_close_is_idempotent(tmp_path):
    path = str(tmp_path / "twice.h5")
    recorder = Hdf5Sink(path)
    recorder.receive(
        st.State(IDENTITY_QUATERNION, np.array([1.0, 0.0, 0.0])), 0.0)
    recorder.close()
    # A second close must not raise or corrupt anything.
    recorder.close()
    with h5py.File(path, "r") as handle:
        assert handle.attrs["sample_count"] == 1


# --------------------------------------------------------------------
# The XDMF companion
# --------------------------------------------------------------------

def test_xdmf_companion_describes_the_trajectory(tmp_path):
    body = make_body(ASYMMETRIC_BOX)
    initial = st.State(IDENTITY_QUATERNION, np.array([0.05, 2.5, 0.05]))
    path = str(tmp_path / "traj.h5")
    recorder = Hdf5Sink(path, flush_every=32)
    engine.run_batch(
        initial, body, [], ig.RungeKutta4Integrator(), 1.0e-3, 0.2,
        [recorder])

    xdmf_path = h5sink._xdmf_path_for(path)
    assert os.path.exists(xdmf_path)

    tree = ElementTree.parse(xdmf_path)
    grids = tree.findall(".//Grid[@GridType='Uniform']")
    with h5py.File(path, "r") as handle:
        sample_count = int(handle.attrs["sample_count"])
        first_time = float(handle[h5sink.TIME_DATASET][0])

    # One timestep grid per recorded sample.
    assert len(grids) == sample_count

    # The first grid's Time matches the first recorded time, and its
    # data source points back at the HDF5 file by basename.
    first_time_element = grids[0].find("Time")
    assert float(first_time_element.get("Value")) == first_time
    sources = [item.text for item in tree.findall(".//DataItem")
               if item.text and ".h5:/" in item.text]
    assert any(os.path.basename(path) in source for source in sources)
    assert any(
        h5sink.ANGULAR_VELOCITY_DATASET in source for source in sources)
