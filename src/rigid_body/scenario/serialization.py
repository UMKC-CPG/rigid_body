"""Save and restore a scenario exactly, as TOML (PSEUDOCODE Section 12).

This module is the only one that reads or writes a scenario file. It
resolves authored unit strings to bare SI through the units boundary, maps
between the TOML document and the schema records of ``scenario``, checks
the recorded body summary against a fresh recomputation, and unpacks a
scenario into a batch run.

TOML is read with the standard library ``tomllib`` on Python 3.11+ and the
``tomli`` backport on 3.10; writing uses ``tomli_w`` (ARCHITECTURE Section
9.1). Resolved floats are written at full precision so the identical IEEE
double is recovered on load, which is what the bit-for-bit reproducibility
guarantee depends on (DESIGN Section 11.7).
"""

try:
    import tomllib
except ModuleNotFoundError:            # Python 3.10 has no tomllib
    import tomli as tomllib

import numpy as np
import tomli_w

from rigid_body.core.units import parse_quantity
from rigid_body.body import shapes
from rigid_body.body.rigid_body_model import (
    RigidBody, classify_top, build_body_from_shape,
    build_body_from_moments)
from rigid_body.dynamics.state import State
from rigid_body.dynamics.torque_models import GravityTorque, ViscousDamping
from rigid_body.dynamics.integrators import select_integrator
from rigid_body.dynamics.simulation_engine import run_batch
from rigid_body.scenario.fidelity import Fidelity
from rigid_body.scenario.scenario import (
    BodySpecification, BodyResolved, Body, InitialConditions,
    TorqueSpecification, Retention, Camera, Presentation, Scenario)


CURRENT_SCHEMA_VERSION = "1.0"

# The consistency check tolerates this relative difference between the
# recorded body summary and a fresh recomputation (DESIGN Section 11.3).
BODY_CONSISTENCY_RELATIVE_TOLERANCE = 1.0e-6

# The SI unit each shape dimension is authored in.
_SHAPE_DIMENSION_UNITS = {
    "sphere": {"radius": "m"},
    "ellipsoid": {"semi_axes": "m"},
    "parallelepiped": {"edge_lengths": "m"},
    "cube": {"edge": "m"},
    "cylinder": {"radius": "m", "height": "m"},
    "cone": {"base_radius": "m", "height": "m"},
    "tetrahedron": {"edge": "m"},
    "octahedron": {"edge": "m"},
    "dodecahedron": {"edge": "m"},
    "icosahedron": {"edge": "m"},
    "moments": {"moments": "kg*m^2", "mass": "kg"},
}

# The SI unit each torque parameter is authored in.
_TORQUE_PARAMETER_UNITS = {
    "gravity": {"gravity": "m/s^2", "pivot_lever_arm_body": "m"},
    "viscous_damping": {"damping_coefficient": "N*m*s"},
}


# --------------------------------------------------------------------
# Resolving authored quantities to SI
# --------------------------------------------------------------------

def _resolve(authored, unit):
    """Resolve an authored value -- a string or a list of strings -- to SI.

    A single string becomes a float; a list becomes a NumPy array.
    """
    if isinstance(authored, list):
        return np.array([parse_quantity(item, unit) for item in authored])
    return parse_quantity(authored, unit)


def _resolve_named(authored_by_name, units_by_name):
    """Resolve a dict of authored values using a dict of expected units."""
    return {name: _resolve(value, units_by_name[name])
            for name, value in authored_by_name.items()}


# --------------------------------------------------------------------
# Building a body from its specification
# --------------------------------------------------------------------

def _construct_shape(kind, resolved_dimensions):
    """Build a shape primitive from resolved SI dimensions."""
    values = resolved_dimensions
    if kind == "sphere":
        return shapes.Sphere(values["radius"])
    if kind == "ellipsoid":
        first, second, third = values["semi_axes"]
        return shapes.Ellipsoid(float(first), float(second), float(third))
    if kind == "parallelepiped":
        first, second, third = values["edge_lengths"]
        return shapes.Parallelepiped(
            float(first), float(second), float(third))
    if kind == "cube":
        return shapes.Cube(values["edge"])
    if kind == "cylinder":
        return shapes.Cylinder(values["radius"], values["height"])
    if kind == "cone":
        return shapes.Cone(values["base_radius"], values["height"])
    if kind == "tetrahedron":
        return shapes.Tetrahedron(values["edge"])
    if kind == "octahedron":
        return shapes.Octahedron(values["edge"])
    if kind == "dodecahedron":
        return shapes.Dodecahedron(values["edge"])
    if kind == "icosahedron":
        return shapes.Icosahedron(values["edge"])
    raise ValueError(f"unknown shape kind: {kind!r}")


def build_body_from_specification(specification):
    """Resolve a body specification and build the RigidBody it names."""
    resolved_dimensions = _resolve_named(
        specification.dimensions,
        _SHAPE_DIMENSION_UNITS[specification.kind])

    if specification.kind == "moments":
        mass = resolved_dimensions.get("mass", 1.0)
        return build_body_from_moments(resolved_dimensions["moments"], mass)

    density = parse_quantity(specification.density, "kg/m^3")
    pivot = None
    if specification.pivot_from_center_of_mass is not None:
        pivot = _resolve(specification.pivot_from_center_of_mass, "m")
    shape = _construct_shape(specification.kind, resolved_dimensions)
    return build_body_from_shape(shape, density, pivot)


def _rigid_body_from_resolved(body):
    """Reconstruct the RigidBody the dynamics consume from the summary.

    The dynamics saw only the tensor, so the recorded tensor is what
    reproduces the run (DESIGN Section 11.3), whatever method produced it.
    """
    moments = np.asarray(body.resolved.principal_moments, dtype=float)
    top_class, intermediate_axis = classify_top(moments)
    return RigidBody(
        principal_moments=moments,
        principal_axes=np.asarray(body.resolved.principal_axes,
                                  dtype=float),
        total_mass=float(body.resolved.total_mass),
        center_of_mass=np.asarray(body.resolved.center_of_mass,
                                  dtype=float),
        top_class=top_class,
        intermediate_axis=intermediate_axis,
        geometry=None)


def _check_body_consistency(body):
    """Recompute the body from its spec and check the recorded summary.

    The storage form of the consistency oracle (DESIGN Section 11.3): a
    mismatch means the specification, the provider, or the file has
    drifted, and it is reported rather than silently resolved.
    """
    recomputed = build_body_from_specification(body.specification)
    recorded = np.asarray(body.resolved.principal_moments, dtype=float)
    scale = max(float(np.max(np.abs(recorded))), 1.0)
    difference = float(np.max(np.abs(recomputed.principal_moments
                                     - recorded)))
    if difference > BODY_CONSISTENCY_RELATIVE_TOLERANCE * scale:
        raise ValueError(
            "recorded body moments disagree with the specification: "
            f"recomputed {recomputed.principal_moments}, "
            f"recorded {recorded}")


# --------------------------------------------------------------------
# Unpacking a scenario for a run
# --------------------------------------------------------------------

def resolve_initial_state(scenario):
    """Build the seven-number State the engine starts from (DESIGN 11.4)."""
    conditions = scenario.initial_conditions
    return State(
        np.asarray(conditions.orientation_quaternion, dtype=float),
        np.asarray(conditions.angular_velocity_body, dtype=float))


def build_torque_models(scenario):
    """Construct the torque model objects from the recorded parameters."""
    models = []
    for specification in scenario.torque_models:
        if specification.type == "gravity":
            models.append(GravityTorque(
                specification.resolved["gravity"],
                specification.resolved["pivot_lever_arm_body"]))
        elif specification.type == "viscous_damping":
            models.append(ViscousDamping(
                specification.resolved["damping_coefficient"]))
        else:
            raise ValueError(
                f"unknown torque type: {specification.type!r}")
    return models


def build_run_components(scenario):
    """Resolve a scenario into the pieces a run is assembled from.

    Returns ``(initial_state, body, torque_models, integrator)`` -- the
    same resolved pieces :func:`run_batch_from_scenario` runs with. Exposed
    so a caller can build something alongside the run from the identical
    inputs, such as a conservation monitor over the run's own body and
    torques (the batch entry script does this).
    """
    initial_state = resolve_initial_state(scenario)
    body = _rigid_body_from_resolved(scenario.body)
    torque_models = build_torque_models(scenario)
    integrator = select_integrator(scenario.fidelity.integrator)
    return initial_state, body, torque_models, integrator


def run_batch_from_scenario(scenario, sinks, monitor=None):
    """Run the batch tier from a scenario (PSEUDOCODE Section 1.3)."""
    initial_state, body, torque_models, integrator = (
        build_run_components(scenario))
    return run_batch(
        initial_state, body, torque_models, integrator,
        scenario.fidelity.time_step, scenario.fidelity.integration_span,
        sinks, monitor=monitor)


# --------------------------------------------------------------------
# TOML load
# --------------------------------------------------------------------

def _scenario_from_document(document):
    """Build a Scenario from a parsed TOML document, resolving units."""
    body_document = document["body"]
    specification = BodySpecification(
        kind=body_document["specification"]["kind"],
        dimensions={
            name: value
            for name, value in body_document["specification"].items()
            if name not in ("kind", "density", "pivot_from_com")},
        density=body_document["specification"].get("density"),
        pivot_from_center_of_mass=body_document["specification"].get(
            "pivot_from_com"))
    resolved = BodyResolved(
        total_mass=body_document["resolved"]["total_mass"],
        center_of_mass=np.array(
            body_document["resolved"]["center_of_mass"]),
        principal_moments=np.array(
            body_document["resolved"]["principal_moments"]),
        principal_axes=np.array(
            body_document["resolved"]["principal_axes"]))
    body = Body(specification=specification, resolved=resolved)

    conditions_document = document["initial_conditions"]
    initial_conditions = InitialConditions(
        angular_velocity_authored=conditions_document[
            "angular_velocity_body"],
        angular_velocity_body=np.array(
            conditions_document["angular_velocity_body_si"]),
        orientation_quaternion=np.array(
            conditions_document["orientation_quaternion"]),
        orientation_euler_zxz=conditions_document.get(
            "orientation_euler_zxz"),
        orientation_axis_angle=conditions_document.get(
            "orientation_axis_angle"))

    torque_models = []
    for table in document.get("torque_models", []):
        torque_type = table["type"]
        authored = {name: value for name, value in table.items()
                    if name != "type"}
        resolved_parameters = _resolve_named(
            authored, _TORQUE_PARAMETER_UNITS[torque_type])
        torque_models.append(TorqueSpecification(
            type=torque_type, authored=authored,
            resolved=resolved_parameters))

    fidelity = Fidelity(**document["fidelity"])
    retention = Retention(
        limit_samples=document["retention"]["limit_samples"])

    presentation_document = document["presentation"]
    camera_document = presentation_document["camera"]
    presentation = Presentation(
        frame=presentation_document["frame"],
        layout=presentation_document["layout"],
        palette=presentation_document["palette"],
        ellipsoid_scale=presentation_document["ellipsoid_scale"],
        camera=Camera(
            position=np.array(camera_document["position"]),
            target=np.array(camera_document["target"]),
            up=np.array(camera_document["up"])),
        scale_factors=presentation_document.get("scale_factors", {}))

    return Scenario(
        schema_version=document["schema_version"],
        body=body,
        initial_conditions=initial_conditions,
        torque_models=torque_models,
        fidelity=fidelity,
        retention=retention,
        presentation=presentation)


def load_scenario(path):
    """Load a scenario from a TOML file and check its body consistency."""
    with open(path, "rb") as scenario_file:
        document = tomllib.load(scenario_file)
    scenario = _scenario_from_document(document)
    _check_body_consistency(scenario.body)
    return scenario


# --------------------------------------------------------------------
# TOML save
# --------------------------------------------------------------------

def _as_list(array):
    """Convert a NumPy array to plain Python floats for TOML output."""
    values = np.asarray(array)
    if values.ndim == 1:
        return [float(item) for item in values]
    return [[float(item) for item in row] for row in values]


def _document_from_scenario(scenario):
    """Build a plain-dict TOML document from a Scenario."""
    specification = scenario.body.specification
    specification_table = {"kind": specification.kind}
    specification_table.update(specification.dimensions)
    if specification.density is not None:
        specification_table["density"] = specification.density
    if specification.pivot_from_center_of_mass is not None:
        specification_table["pivot_from_com"] = (
            specification.pivot_from_center_of_mass)

    resolved = scenario.body.resolved
    body_table = {
        "specification": specification_table,
        "resolved": {
            "total_mass": float(resolved.total_mass),
            "center_of_mass": _as_list(resolved.center_of_mass),
            "principal_moments": _as_list(resolved.principal_moments),
            "principal_axes": _as_list(resolved.principal_axes)}}

    conditions = scenario.initial_conditions
    conditions_table = {
        "angular_velocity_body": conditions.angular_velocity_authored,
        "angular_velocity_body_si": _as_list(
            conditions.angular_velocity_body),
        "orientation_quaternion": _as_list(
            conditions.orientation_quaternion)}
    if conditions.orientation_euler_zxz is not None:
        conditions_table["orientation_euler_zxz"] = (
            conditions.orientation_euler_zxz)
    if conditions.orientation_axis_angle is not None:
        conditions_table["orientation_axis_angle"] = (
            conditions.orientation_axis_angle)

    torque_tables = []
    for specification in scenario.torque_models:
        table = {"type": specification.type}
        table.update(specification.authored)
        torque_tables.append(table)

    fidelity = scenario.fidelity
    presentation = scenario.presentation
    document = {
        "schema_version": scenario.schema_version,
        "body": body_table,
        "initial_conditions": conditions_table,
        "torque_models": torque_tables,
        "fidelity": {
            "integrator": fidelity.integrator,
            "time_step": float(fidelity.time_step),
            "substeps_per_frame": int(fidelity.substeps_per_frame),
            "integration_span": float(fidelity.integration_span)},
        "retention": {"limit_samples": int(scenario.retention.limit_samples)},
        "presentation": {
            "frame": presentation.frame,
            "layout": presentation.layout,
            "palette": presentation.palette,
            "ellipsoid_scale": presentation.ellipsoid_scale,
            "camera": {
                "position": _as_list(presentation.camera.position),
                "target": _as_list(presentation.camera.target),
                "up": _as_list(presentation.camera.up)},
            "scale_factors": presentation.scale_factors}}
    return document


def save_scenario(scenario, path):
    """Write a scenario to a TOML file, authored and resolved forms both."""
    document = _document_from_scenario(scenario)
    with open(path, "wb") as scenario_file:
        tomli_w.dump(document, scenario_file)


def scenario_to_toml(scenario):
    """Return the scenario serialized to a TOML string.

    The same document :func:`save_scenario` writes, rendered to text rather
    than a file. The batch entry script embeds this string in the HDF5
    output as provenance, so any result traces back to the exact run that
    produced it (ARCHITECTURE Section 9.4).
    """
    return tomli_w.dumps(_document_from_scenario(scenario))
