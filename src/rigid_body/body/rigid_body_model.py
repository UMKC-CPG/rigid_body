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

The constructors ``build_body_from_shape`` and ``build_body_from_moments``
(PSEUDOCODE Section 4.7) assemble a body from a primitive through the
closed-form provider or from directly entered moments, applying the
physical-validity checks of Section 4.2, the optional parallel-axis pivot
shift of Section 4.5, and the diagonalization of Section 4.4.
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import NamedTuple, Optional

import numpy as np

from rigid_body.body.analytic_inertia import analytic_inertia


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


class PrincipalFrame(NamedTuple):
    """The principal moments and the axes they lie along.

    ``axes`` are the columns of a right-handed rotation matrix; the moment
    ``moments[k]`` lies along column ``k``.
    """

    moments: np.ndarray
    axes: np.ndarray


def validate_inertia_tensor(inertia_tensor, tolerance=1.0e-9):
    """Check that a tensor is symmetric (DESIGN Section 3.1).

    Symmetry follows immediately from the definition of the inertia
    integral; a violation means the numbers are not an inertia tensor.
    """
    if not np.allclose(inertia_tensor, inertia_tensor.T, atol=tolerance):
        raise ValueError("inertia tensor is not symmetric")


def validate_moments(principal_moments):
    """Check the two physical constraints on principal moments.

    Every physical body has strictly positive moments that satisfy the
    triangle inequalities (DESIGN Section 3.1). A failure is reported with
    which constraint failed and by how much, rather than integrated into
    nonsense.
    """
    moment_1, moment_2, moment_3 = (float(v) for v in principal_moments)
    if not (moment_1 > 0.0 and moment_2 > 0.0 and moment_3 > 0.0):
        raise ValueError(
            f"a principal moment is not positive: {principal_moments}")
    triples = [(moment_1, moment_2, moment_3),
               (moment_2, moment_3, moment_1),
               (moment_3, moment_1, moment_2)]
    for first, second, third in triples:
        if first + second < third:
            raise ValueError(
                f"triangle inequality fails: {first} + {second} < "
                f"{third}, short by {third - first - second}")


def parallel_axis_shift(inertia_about_center, mass, displacement):
    """Shift an inertia tensor from the center of mass to a new point.

    The parallel-axis theorem (DESIGN Section 3.5): ``I_new = I_com +
    M (|d|^2 Identity - d (outer) d)`` with ``d`` the displacement from the
    center of mass. The result is in general no longer diagonal in the old
    axes, so it must be re-diagonalized (below) before Euler's equations
    can use it.
    """
    displacement = np.asarray(displacement, dtype=float)
    distance_squared = float(np.dot(displacement, displacement))
    return inertia_about_center + mass * (
        distance_squared * np.eye(3)
        - np.outer(displacement, displacement))


def _is_diagonal(inertia_tensor, tolerance=1.0e-12):
    """True when the off-diagonal entries are negligibly small."""
    off_diagonal = inertia_tensor - np.diag(np.diag(inertia_tensor))
    scale = max(float(np.max(np.abs(np.diag(inertia_tensor)))), 1.0)
    return float(np.max(np.abs(off_diagonal))) <= tolerance * scale


def principal_frame_of(inertia_tensor):
    """Return the principal moments and axes of an inertia tensor.

    For a tensor already diagonal -- every primitive in its natural
    orientation -- the axes are the coordinate axes and no eigenvalue
    problem is solved (DESIGN Section 3.4). Otherwise a symmetric
    eigensolver is used, and a left-handed eigenvector set is corrected to
    right-handed by negating one axis, since a reflection is not a
    rotation.

    Known limitation: when two or three moments coincide the eigenvectors
    of the degenerate subspace are not unique, and DESIGN Section 3.4 asks
    that they be aligned to the body's geometric symmetry axis. That
    canonicalization is not yet applied here; it matters only for a
    re-diagonalized symmetric top (an off-axis pivot), since every
    natural-orientation primitive takes the diagonal shortcut above.
    """
    if _is_diagonal(inertia_tensor):
        return PrincipalFrame(
            np.diag(inertia_tensor).copy(), np.eye(3))

    moments, axes = np.linalg.eigh(inertia_tensor)
    if np.linalg.det(axes) < 0.0:
        axes = axes.copy()
        axes[:, 0] = -axes[:, 0]
    return PrincipalFrame(moments, axes)


def build_body_from_shape(shape, density,
                          pivot_from_center_of_mass=None):
    """Assemble a RigidBody from a primitive shape (PSEUDOCODE 4.7).

    Computes the inertia through the closed-form provider, optionally
    shifts it to a pivot by the parallel-axis theorem, diagonalizes, checks
    physical validity, and classifies. When a pivot is given it becomes the
    rotation origin, and the stored center of mass is measured from it --
    which is exactly the pivot-to-center-of-mass lever arm the gravity
    torque model needs (DESIGN Section 5.3).
    """
    properties = analytic_inertia(shape, density)
    inertia_tensor = properties.inertia_tensor
    center_of_mass = properties.center_of_mass

    if pivot_from_center_of_mass is not None:
        displacement = np.asarray(pivot_from_center_of_mass, dtype=float)
        inertia_tensor = parallel_axis_shift(
            inertia_tensor, properties.mass, displacement)
        center_of_mass = -displacement

    validate_inertia_tensor(inertia_tensor)
    frame = principal_frame_of(inertia_tensor)
    validate_moments(frame.moments)
    top_class, intermediate_axis = classify_top(frame.moments)
    return RigidBody(
        principal_moments=frame.moments,
        principal_axes=frame.axes,
        total_mass=properties.mass,
        center_of_mass=center_of_mass,
        top_class=top_class,
        intermediate_axis=intermediate_axis,
        geometry=shape)


def build_body_from_moments(principal_moments, total_mass,
                            center_of_mass=None):
    """Assemble a RigidBody from directly entered moments (DESIGN 3.6).

    For a body specified by its moments alone -- the Chandler wobble needs
    only the ratio of the Earth's moments -- there is no shape to draw. The
    moments are already principal, so the axes are the coordinate axes.
    """
    moments = np.asarray(principal_moments, dtype=float)
    validate_moments(moments)
    top_class, intermediate_axis = classify_top(moments)
    if center_of_mass is None:
        center_of_mass = np.zeros(3)
    return RigidBody(
        principal_moments=moments,
        principal_axes=np.eye(3),
        total_mass=float(total_mass),
        center_of_mass=np.asarray(center_of_mass, dtype=float),
        top_class=top_class,
        intermediate_axis=intermediate_axis,
        geometry=None)
