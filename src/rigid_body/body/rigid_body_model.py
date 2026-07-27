"""The assembled rigid body: the record the dynamics consume.

This is the code form of the ``RigidBody`` record of PSEUDOCODE Section
4.1 and the classification of Section 4.6 (DESIGN Sections 3.3, 3.8, and
1.4). After a body is built, the dynamics see only these few numbers --
the three principal moments and their axes, the mass, and the
classification -- and never consult the shape again (DESIGN Section 3).

The record holds only quantities fixed in the body frame, computed once
at construction and reused thereafter; that constancy is exactly what the
word "rigid" means (DESIGN Section 3.8). Changing any of it builds a new
body rather than mutating this one, which keeps the scenario record
unambiguous about which body produced which trajectory.

The general constructor ``build_body`` -- diagonalization, the pivot
shift, and validation (PSEUDOCODE Sections 4.2, 4.4, 4.5, 4.7) -- is
assembled in a later step; this module supplies the record itself and the
classification the equations of motion and the Poinsot geometry need.
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np


# Two principal moments are treated as equal when they agree to this
# relative tolerance. The distinction is physical (DESIGN Section 3.3):
# only strictly distinct moments give an intermediate axis and the
# Dzhanibekov flip, so the threshold is kept tight.
MOMENT_EQUALITY_RELATIVE_TOLERANCE = 1.0e-9


class TopClass(Enum):
    """The degeneracy pattern of the principal moments (DESIGN 3.3).

    ``SPHERICAL`` -- all three equal (sphere and every Platonic solid):
    constant angular velocity, no free precession. ``SYMMETRIC`` --
    exactly two equal (cylinder, cone): steady precession. ``ASYMMETRIC``
    -- all three distinct (box, general ellipsoid): the full Poinsot
    picture and the only class that can exhibit the intermediate-axis
    instability.
    """

    SPHERICAL = auto()
    SYMMETRIC = auto()
    ASYMMETRIC = auto()


@dataclass(eq=False)
class RigidBody:
    """A body reduced to the quantities the dynamics consume.

    ``principal_moments`` are ``(I_1, I_2, I_3)`` and ``principal_axes``
    are their axis directions as the columns of a rotation matrix.
    ``intermediate_axis`` is the 1-based index of the middle moment for an
    asymmetric top and ``None`` otherwise. ``geometry`` is the drawable
    shape, or ``None`` for a body entered by its moments alone.

    Treated as immutable: a change constructs a new ``RigidBody``.
    """

    principal_moments: np.ndarray
    principal_axes: np.ndarray
    total_mass: float
    center_of_mass: np.ndarray
    top_class: TopClass
    intermediate_axis: Optional[int]
    geometry: object


def classify_top(principal_moments,
                 relative_tolerance=MOMENT_EQUALITY_RELATIVE_TOLERANCE):
    """Classify a body by the equality pattern of its moments.

    Returns ``(top_class, intermediate_axis)``. For an asymmetric top the
    intermediate axis is named explicitly (DESIGN Section 1.4), because
    the Dzhanibekov flip (VISION Goal 2) is an instability about that axis
    and exists only when the three moments are strictly distinct.
    """
    moment_1, moment_2, moment_3 = (float(value)
                                    for value in principal_moments)
    equal_12 = np.isclose(moment_1, moment_2, rtol=relative_tolerance,
                          atol=0.0)
    equal_23 = np.isclose(moment_2, moment_3, rtol=relative_tolerance,
                          atol=0.0)
    equal_13 = np.isclose(moment_1, moment_3, rtol=relative_tolerance,
                          atol=0.0)

    if equal_12 and equal_23:
        return TopClass.SPHERICAL, None
    if equal_12 or equal_23 or equal_13:
        return TopClass.SYMMETRIC, None

    # All three distinct: the intermediate axis is the one carrying the
    # median moment. argsort gives ascending indices; the middle one is
    # the median, reported 1-based to match the physics notation.
    ascending = np.argsort(np.asarray(principal_moments, dtype=float))
    intermediate_axis = int(ascending[1]) + 1
    return TopClass.ASYMMETRIC, intermediate_axis
