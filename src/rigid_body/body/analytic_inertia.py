"""Closed-form inertia for the uniform primitives (DESIGN Section 3.2).

This is the closed-form implementation of the inertia-provider operation
(ARCHITECTURE Section 5.1, PSEUDOCODE Section 4.3): given a shape from
``shapes`` and a uniform density, return the mass, the center of mass,
and the inertia tensor. A future ``numerical_inertia`` will implement the
same operation by integrating a density field, and no consumer will be
able to tell which produced a tensor -- that interchangeability is the
seam VISION Principle 10 asks for.

For a primitive stated in its natural orientation the tensor is diagonal,
so no eigenvalue problem is solved downstream (DESIGN Section 3.4). Each
shape is defined about its own center of mass, so the returned center of
mass is the origin; only a later mesh provider needs a nonzero one.

The five Platonic constants and the primitive closed forms were verified
numerically before being trusted (DESIGN Section 3.2). The icosahedron in
particular is ``(3 + sqrt(5)) / 20 * M a^2`` -- the value a factor of two
apart from a plausible wrong one -- and the tests for this module check
every closed form against the exact tetrahedron decomposition in
``dev/spikes/platonic_inertia.py``.
"""

import math
from typing import NamedTuple

import numpy as np

from rigid_body.body import shapes


# The golden ratio enters the dodecahedron and icosahedron constants.
SQRT_5 = math.sqrt(5.0)


class InertiaProperties(NamedTuple):
    """The three items the inertia provider returns (ARCHITECTURE 5.1).

    ``mass`` is the total mass in kilograms, ``center_of_mass`` is a
    3-vector in the shape's construction frame (the origin for every
    primitive here), and ``inertia_tensor`` is the 3x3 tensor about that
    center of mass -- diagonal for a primitive in natural orientation.
    """

    mass: float
    center_of_mass: np.ndarray
    inertia_tensor: np.ndarray


def volume_of(shape):
    """Return the volume of a primitive shape, in cubic meters.

    Needed to turn a uniform density into a mass. Each formula is the
    standard closed-form volume of the solid.
    """
    if isinstance(shape, shapes.Sphere):
        return (4.0 / 3.0) * math.pi * shape.radius**3
    if isinstance(shape, shapes.Ellipsoid):
        return ((4.0 / 3.0) * math.pi
                * shape.semi_axis_a * shape.semi_axis_b
                * shape.semi_axis_c)
    if isinstance(shape, shapes.Parallelepiped):
        return shape.edge_a * shape.edge_b * shape.edge_c
    if isinstance(shape, shapes.Cube):
        return shape.edge**3
    if isinstance(shape, shapes.Cylinder):
        return math.pi * shape.radius**2 * shape.height
    if isinstance(shape, shapes.Cone):
        return (1.0 / 3.0) * math.pi * shape.base_radius**2 * shape.height
    if isinstance(shape, shapes.Tetrahedron):
        return math.sqrt(2.0) / 12.0 * shape.edge**3
    if isinstance(shape, shapes.Octahedron):
        return math.sqrt(2.0) / 3.0 * shape.edge**3
    if isinstance(shape, shapes.Dodecahedron):
        return (15.0 + 7.0 * SQRT_5) / 4.0 * shape.edge**3
    if isinstance(shape, shapes.Icosahedron):
        return 5.0 * (3.0 + SQRT_5) / 12.0 * shape.edge**3
    raise TypeError(f"volume_of: unsupported shape {type(shape).__name__}")


def _isotropic(factor, mass, edge):
    """Return the three equal moments of a spherical top.

    Every Platonic solid is isotropic (DESIGN Section 3.3), so its inertia
    is ``factor * M * edge^2`` about all three principal axes.
    """
    moment = factor * mass * edge**2
    return np.array([moment, moment, moment])


def principal_moments_of(shape, mass):
    """Return the three principal moments ``(I_1, I_2, I_3)`` of a shape.

    These are the closed forms of DESIGN Section 3.2, about the shape's
    center of mass, for the given total ``mass``. The result is ordered by
    the shape's own axes 1, 2, 3 and is not sorted, because which axis is
    intermediate is physically meaningful (DESIGN Section 1.4).
    """
    if isinstance(shape, shapes.Sphere):
        moment = (2.0 / 5.0) * mass * shape.radius**2
        return np.array([moment, moment, moment])
    if isinstance(shape, shapes.Ellipsoid):
        semi_a = shape.semi_axis_a
        semi_b = shape.semi_axis_b
        semi_c = shape.semi_axis_c
        return (mass / 5.0) * np.array([
            semi_b**2 + semi_c**2,
            semi_c**2 + semi_a**2,
            semi_a**2 + semi_b**2,
        ])
    if isinstance(shape, shapes.Parallelepiped):
        edge_a, edge_b, edge_c = shape.edge_a, shape.edge_b, shape.edge_c
        return (mass / 12.0) * np.array([
            edge_b**2 + edge_c**2,
            edge_c**2 + edge_a**2,
            edge_a**2 + edge_b**2,
        ])
    if isinstance(shape, shapes.Cube):
        return _isotropic(1.0 / 6.0, mass, shape.edge)
    if isinstance(shape, shapes.Cylinder):
        radius, height = shape.radius, shape.height
        transverse = (mass / 12.0) * (3.0 * radius**2 + height**2)
        axial = 0.5 * mass * radius**2
        return np.array([transverse, transverse, axial])
    if isinstance(shape, shapes.Cone):
        radius, height = shape.base_radius, shape.height
        transverse = (3.0 * mass / 80.0) * (4.0 * radius**2 + height**2)
        axial = (3.0 / 10.0) * mass * radius**2
        return np.array([transverse, transverse, axial])
    if isinstance(shape, shapes.Tetrahedron):
        return _isotropic(1.0 / 20.0, mass, shape.edge)
    if isinstance(shape, shapes.Octahedron):
        return _isotropic(1.0 / 10.0, mass, shape.edge)
    if isinstance(shape, shapes.Dodecahedron):
        factor = (95.0 + 39.0 * SQRT_5) / 300.0
        return _isotropic(factor, mass, shape.edge)
    if isinstance(shape, shapes.Icosahedron):
        # (3 + sqrt(5)) / 20, NOT / 10: the factor-of-two trap DESIGN
        # Section 3.2 warns of, confirmed against the exact decomposition.
        factor = (3.0 + SQRT_5) / 20.0
        return _isotropic(factor, mass, shape.edge)
    raise TypeError(
        f"principal_moments_of: unsupported shape "
        f"{type(shape).__name__}")


def analytic_inertia(shape, density):
    """Return the mass, center of mass, and inertia tensor of a shape.

    The closed-form provider (ARCHITECTURE Section 5.1). Mass is the
    uniform ``density`` times the volume; the tensor is diagonal because
    the shape is in its natural principal-axis orientation; the center of
    mass is the origin because every primitive is defined about it.
    """
    mass = density * volume_of(shape)
    moments = principal_moments_of(shape, mass)
    inertia_tensor = np.diag(moments)
    center_of_mass = np.zeros(3)
    return InertiaProperties(mass, center_of_mass, inertia_tensor)
