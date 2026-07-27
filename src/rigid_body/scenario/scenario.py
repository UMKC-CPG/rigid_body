"""The scenario schema: plain-data records mirroring the TOML layout.

These dataclasses are the in-memory form of the TOML scenario of
PSEUDOCODE Section 12.3. Following the decision of Section 12.6, every
physical quantity is held in *both* forms: the authored string a human
wrote (``"0.5 kg*m^2"``, ``"30 deg"``) for readability, and the resolved
bare-SI value the engine actually reads. Serialization keeps the two in
step; nothing here parses units or touches a file -- that is the job of
``serialization``.

The records divide into the physics zone (``BodySpecification``,
``BodyResolved``, ``InitialConditions``, ``TorqueSpecification``,
``Fidelity``, ``Retention``) and the presentation zone (``Presentation``,
``Camera``). Only the physics zone feeds the engine, so editing the
presentation zone never moves the trajectory (DESIGN Section 11.2).
"""

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# --------------------------------------------------------------------
# The body: a human-editable specification and a resolved summary
# --------------------------------------------------------------------

@dataclass
class BodySpecification:
    """The human-editable handle on a body (DESIGN Section 11.3).

    ``kind`` names the primitive (``"cube"``, ``"parallelepiped"``, ...) or
    ``"moments"`` for a body entered by its principal moments alone.
    ``dimensions`` maps each dimension name to its authored string (or a
    list of strings for a vector dimension such as ``edge_lengths``).
    ``density`` is the authored density, or ``None`` for a moment-only
    body. ``pivot_from_center_of_mass`` is the authored lever arm to a
    fixed pivot, or ``None`` for a free body.
    """

    kind: str
    dimensions: dict
    density: Optional[str] = None
    pivot_from_center_of_mass: Optional[list] = None


@dataclass
class BodyResolved:
    """The four provider items the dynamics consume (DESIGN Section 3.7).

    Bare SI, recomputed from the specification on load and checked against
    the recorded values (DESIGN Section 11.3). For a moment-only body this
    summary simply is the entry.
    """

    total_mass: float
    center_of_mass: np.ndarray
    principal_moments: np.ndarray
    principal_axes: np.ndarray


@dataclass
class Body:
    """A body in its two layers: specification and resolved summary."""

    specification: BodySpecification
    resolved: BodyResolved


# --------------------------------------------------------------------
# Initial conditions
# --------------------------------------------------------------------

@dataclass
class InitialConditions:
    """The authored initial orientation and spin, plus their resolved form.

    The orientation is authored one of two ways -- a z-x-z Euler triple in
    ``orientation_euler_zxz`` or an axis and angle in
    ``orientation_axis_angle`` -- and always recorded resolved as
    ``orientation_quaternion`` (scalar-first, unit), which pins the
    double-cover sign. ``angular_velocity_authored`` holds the authored
    body-frame components and ``angular_velocity_body`` their resolved SI,
    including any deliberate Dzhanibekov tilt (DESIGN Section 11.4).
    """

    angular_velocity_authored: list
    angular_velocity_body: np.ndarray
    orientation_quaternion: np.ndarray
    orientation_euler_zxz: Optional[list] = None
    orientation_axis_angle: Optional[dict] = None


# --------------------------------------------------------------------
# Torque models
# --------------------------------------------------------------------

@dataclass
class TorqueSpecification:
    """One torque model as recorded: its type and its parameters.

    ``type`` names the model (``"gravity"``, ``"viscous_damping"``).
    ``authored`` maps each parameter name to its authored string (or list
    of strings), and ``resolved`` maps the same names to the resolved SI
    values. The scenario's ordered list of these is recorded physics: the
    order is iterated as written (DESIGN Section 11.5).
    """

    type: str
    authored: dict
    resolved: dict


# --------------------------------------------------------------------
# Retention and presentation
# --------------------------------------------------------------------

@dataclass
class Retention:
    """The bounded trajectory-retention limit (DESIGN Section 12)."""

    limit_samples: int


@dataclass
class Camera:
    """The viewpoint: where the eye sits, independent of the frame."""

    position: np.ndarray
    target: np.ndarray
    up: np.ndarray


@dataclass
class Presentation:
    """What is shown of a trajectory already determined by the physics.

    Editing anything here never moves the trajectory (DESIGN Section 11.2):
    the frame and layout choice, the palette, the ellipsoid scale, the
    camera, and the labeled exaggeration factors.
    """

    frame: str
    layout: str
    palette: str
    ellipsoid_scale: str
    camera: Camera
    scale_factors: dict = field(default_factory=dict)


# --------------------------------------------------------------------
# The whole scenario
# --------------------------------------------------------------------

@dataclass
class Scenario:
    """A complete, reproducible description of a run (VISION Goal 11)."""

    schema_version: str
    body: Body
    initial_conditions: InitialConditions
    torque_models: list
    fidelity: "Fidelity"
    retention: Retention
    presentation: Presentation
