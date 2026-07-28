"""The HDF5 recording sink: the batch tier's output, spilled to disk.

This is the code form of PSEUDOCODE Section 13.6 (ARCHITECTURE Sections
3.7 and 9.4). When a run must be kept in full -- to export a long span
(VISION Goal 11) or hand a complete session to post-processing -- the
bounded in-memory window of Section 13.4 is not enough, so the whole state
stream goes through the sink boundary and lands on disk. The batch tier
*is* an engine whose only sink is this one (Section 1.3); a recording
interactive session is the same engine with this sink attached besides.

Two properties shape the design:

* **Bounded memory while the disk accumulates everything.** The engine
  calls :meth:`receive` once per substep without knowing the run length
  ahead of time, so the datasets are resizable and each ``receive``
  appends to a small in-memory batch that is flushed to disk when full.
  Memory stays at the batch size no matter how long the run.

* **Read-only, so it cannot disturb the physics.** A sink may look at a
  state but must never feed a value back into the loop; a trajectory that
  depended on which sinks were attached would break the determinism
  guarantee (ARCHITECTURE Section 6.4).

The seven state numbers are written verbatim (a scalar-first body-to-space
unit quaternion and the body-frame angular velocity, both bare SI), the
full scenario is embedded as provenance so any result traces back to the
run that produced it (ARCHITECTURE Section 9.4), and an XDMF companion is
written beside the HDF5 file so ParaView can read the trajectory directly.
"""

import os
import xml.etree.ElementTree as ElementTree

import numpy as np
import h5py

from rigid_body.sinks.sink_interface import Sink


# The on-disk schema version, stored as a root attribute. Bump it if the
# dataset names or shapes ever change, so a reader can refuse a file it
# does not understand.
FORMAT_VERSION = 1

# Dataset names, kept in one place so the writer and the XDMF companion
# cannot drift apart.
TIME_DATASET = "time"
ORIENTATION_DATASET = "body_to_space_quaternion"
ANGULAR_VELOCITY_DATASET = "angular_velocity_body"


class Hdf5Sink(Sink):
    """Record the full state stream to an HDF5 file, with an XDMF companion.

    Constructed with the output path; thereafter the engine drives it
    through the sink protocol -- ``receive`` once per substep and ``close``
    once at the end. ``scenario_toml`` is the run's scenario serialized to
    TOML, embedded as provenance; ``attributes`` is any extra string
    metadata to store on the file root. ``flush_every`` sets how many
    samples are buffered in memory before a write, trading a little memory
    for far fewer disk operations; ``write_xdmf`` controls whether the
    ParaView descriptor is emitted.
    """

    def __init__(self, file_path, scenario_toml=None, attributes=None,
                 flush_every=2048, write_xdmf=True):
        self.file_path = file_path
        self.scenario_toml = scenario_toml
        self.extra_attributes = dict(attributes or {})
        self.flush_every = int(flush_every)
        self.write_xdmf = write_xdmf

        # The number of samples written so far, and whether close() has
        # already run (so a double close is harmless).
        self.sample_count = 0
        self._closed = False

        # In-memory batch buffers; each receive appends one row, and a
        # full batch is flushed to disk. Keeping three parallel lists
        # avoids allocating per-sample arrays.
        self._time_buffer = []
        self._orientation_buffer = []
        self._angular_velocity_buffer = []

        # Open the file and create the three resizable, chunked datasets.
        # An unbounded first axis lets the run be any length; the chunk
        # rows match the flush size so a flush is one contiguous write.
        self._file = h5py.File(file_path, "w")
        chunk_rows = max(self.flush_every, 1)
        self._time = self._file.create_dataset(
            TIME_DATASET, shape=(0,), maxshape=(None,),
            chunks=(chunk_rows,), dtype="float64")
        self._orientation = self._file.create_dataset(
            ORIENTATION_DATASET, shape=(0, 4), maxshape=(None, 4),
            chunks=(chunk_rows, 4), dtype="float64")
        self._angular_velocity = self._file.create_dataset(
            ANGULAR_VELOCITY_DATASET, shape=(0, 3), maxshape=(None, 3),
            chunks=(chunk_rows, 3), dtype="float64")

        self._write_provenance_attributes()

    def _write_provenance_attributes(self):
        """Record the schema, unit, and convention metadata on the root.

        These make the file self-describing (ARCHITECTURE Section 9.4): a
        reader learns the schema version, that quantities are bare SI, and
        that the quaternion is scalar-first and maps body to space. The
        scenario TOML, when supplied, is the provenance that ties the
        result back to the run that produced it.
        """
        root = self._file.attrs
        root["format_version"] = FORMAT_VERSION
        root["units"] = "SI (radians, radians/second, seconds)"
        root["quaternion_convention"] = (
            "scalar-first unit quaternion, body-to-space")
        if self.scenario_toml is not None:
            root["scenario_toml"] = self.scenario_toml
        for name, value in self.extra_attributes.items():
            root[name] = value

    def receive(self, state, time):
        """Append one emitted state to the in-memory batch.

        Reads the seven state numbers and the emission time -- never
        writing anything back -- and flushes the batch to disk once it
        reaches ``flush_every`` samples, keeping memory bounded.
        """
        self._time_buffer.append(float(time))
        self._orientation_buffer.append(
            np.asarray(state.body_to_space_quaternion, dtype=float))
        self._angular_velocity_buffer.append(
            np.asarray(state.angular_velocity_body, dtype=float))

        if len(self._time_buffer) >= self.flush_every:
            self._flush_batch()

    def _flush_batch(self):
        """Write the buffered samples to disk and clear the buffers.

        Grows each dataset by the batch length and writes the batch into
        the new tail region in one contiguous operation, then empties the
        in-memory buffers so memory returns to nearly zero.
        """
        batch_length = len(self._time_buffer)
        if batch_length == 0:
            return

        new_count = self.sample_count + batch_length
        self._time.resize((new_count,))
        self._orientation.resize((new_count, 4))
        self._angular_velocity.resize((new_count, 3))

        tail = slice(self.sample_count, new_count)
        self._time[tail] = np.asarray(self._time_buffer, dtype=float)
        self._orientation[tail] = np.stack(self._orientation_buffer)
        self._angular_velocity[tail] = np.stack(
            self._angular_velocity_buffer)

        self.sample_count = new_count
        self._time_buffer.clear()
        self._orientation_buffer.clear()
        self._angular_velocity_buffer.clear()

    def close(self):
        """Flush the last partial batch, finalize metadata, close the file.

        Safe to call more than once. Records the final sample count, emits
        the XDMF companion if requested, and releases the file handle.
        """
        if self._closed:
            return

        self._flush_batch()
        self._file.attrs["sample_count"] = self.sample_count

        if self.write_xdmf:
            # Read the time column back to stamp each XDMF timestep; this
            # is an end-of-run cost proportional to the (already O(N))
            # descriptor, not a burden carried during the run.
            times = np.asarray(self._time[:]) if self.sample_count else (
                np.empty(0))
            self._emit_xdmf_companion(times)

        self._file.close()
        self._closed = True

    def _emit_xdmf_companion(self, times):
        """Write the XDMF descriptor beside the HDF5 file for ParaView.

        The trajectory is described as a temporal collection of single
        points, one per timestep, each carrying the angular velocity and
        the orientation quaternion as node attributes hyper-slabbed out of
        the HDF5 datasets. ParaView can then scrub the time series and plot
        the recorded quantities directly (ARCHITECTURE Section 9.4).
        """
        hdf5_basename = os.path.basename(self.file_path)
        xdmf_path = _xdmf_path_for(self.file_path)
        document = _build_xdmf_tree(
            hdf5_basename, times, self.sample_count)
        ElementTree.ElementTree(document).write(
            xdmf_path, encoding="utf-8", xml_declaration=True)


# --------------------------------------------------------------------
# XDMF companion construction (module-level so it is easy to test)
# --------------------------------------------------------------------

def _xdmf_path_for(hdf5_path):
    """Return the companion ``.xdmf`` path for an HDF5 output path."""
    root, _extension = os.path.splitext(hdf5_path)
    return root + ".xdmf"


def _build_xdmf_tree(hdf5_basename, times, sample_count):
    """Build the XDMF element tree for a point trajectory.

    Returns the root ``Xdmf`` element of a temporal collection whose grids
    each pin one timestep to the origin and attach that step's angular
    velocity and quaternion by an HDF5 hyperslab. The geometry is a single
    fixed point because the rotational state carries no translation
    (DESIGN Section 2.1); the interesting quantities ride as attributes.
    """
    xdmf = ElementTree.Element("Xdmf", Version="3.0")
    domain = ElementTree.SubElement(xdmf, "Domain")
    collection = ElementTree.SubElement(
        domain, "Grid", Name="trajectory", GridType="Collection",
        CollectionType="Temporal")

    for index in range(sample_count):
        grid = ElementTree.SubElement(
            collection, "Grid", Name=f"step_{index}", GridType="Uniform")
        ElementTree.SubElement(
            grid, "Time", Value=repr(float(times[index])))
        ElementTree.SubElement(
            grid, "Topology", TopologyType="Polyvertex",
            NumberOfElements="1")
        geometry = ElementTree.SubElement(
            grid, "Geometry", GeometryType="XYZ")
        origin = ElementTree.SubElement(
            geometry, "DataItem", Dimensions="1 3", Format="XML")
        origin.text = "0.0 0.0 0.0"

        _append_hyperslab_attribute(
            grid, "angular_velocity", "Vector",
            hdf5_basename, ANGULAR_VELOCITY_DATASET,
            index, sample_count, width=3)
        _append_hyperslab_attribute(
            grid, "orientation_quaternion", "Vector",
            hdf5_basename, ORIENTATION_DATASET,
            index, sample_count, width=4)

    return xdmf


def _append_hyperslab_attribute(grid, name, attribute_type,
                                hdf5_basename, dataset_name,
                                index, sample_count, width):
    """Attach one timestep's row of a dataset as a node attribute.

    The HyperSlab selects row ``index`` (start ``index 0``, stride
    ``1 1``, count ``1 width``) out of the full ``sample_count x width``
    dataset, so each timestep reads exactly its own values without copying
    the data out of the HDF5 file.
    """
    attribute = ElementTree.SubElement(
        grid, "Attribute", Name=name, AttributeType=attribute_type,
        Center="Node")
    hyperslab = ElementTree.SubElement(
        attribute, "DataItem", ItemType="HyperSlab",
        Dimensions=f"1 {width}", Type="HyperSlab")
    selection = ElementTree.SubElement(
        hyperslab, "DataItem", Dimensions="3 2", Format="XML")
    # Three rows: start (row index, column 0), stride (1, 1), count
    # (1 row, `width` columns).
    selection.text = f"{index} 0  1 1  1 {width}"
    source = ElementTree.SubElement(
        hyperslab, "DataItem", Dimensions=f"{sample_count} {width}",
        NumberType="Float", Precision="8", Format="HDF")
    source.text = f"{hdf5_basename}:/{dataset_name}"
