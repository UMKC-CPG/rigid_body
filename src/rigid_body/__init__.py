"""The rigid_body library: interactive and batch rigid-body rotation.

This package holds everything reusable in the project — the physics
core, the derived display geometry, the scenario schema, and the
presentation layer. The user-facing command-line entry points live
separately under ``src/scripts/`` (ARCHITECTURE Section 1).

The subpackages mirror the architecture layout:

    core/       Units and rotation mathematics (the foundation)
    body/       Shapes, density, and inertia computation
    dynamics/   Equations of motion, torques, integrators, the engine
    analysis/   Conservation monitoring and analytic solutions
    geometry/   Derived display geometry (Poinsot, reference frames)
    scenario/   The scenario schema and its serialization
    sinks/      Consumers of a computed trajectory
    render/     Scene description, palettes, and the vedo backend
    ui/         Interactive controls and time control

Read the design chain in ``dev/`` (VISION, ARCHITECTURE, DESIGN,
PSEUDOCODE) for the reasoning behind every module here.
"""
