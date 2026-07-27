"""Shape primitives: the geometry a body is built from.

Each primitive is a small immutable record of the dimensions that define
it, expressed in its own principal-axis frame centered on its center of
mass (DESIGN Section 3.2). A shape carries only geometry and no physics;
its inertia is a closed form computed in ``analytic_inertia`` and, once
computed, the shape is never consulted again (DESIGN Section 3).

The primitives here are exactly the ``kind`` values a scenario may name
(PSEUDOCODE Section 12.3): the round and rectangular solids, and the
five Platonic solids -- each of which is a *spherical top*, with all
three principal moments equal, however little it looks like a sphere
(DESIGN Section 3.3).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Sphere:
    """A solid sphere of the given radius."""

    radius: float


@dataclass(frozen=True)
class Ellipsoid:
    """A solid ellipsoid with semi-axes along body axes 1, 2, 3."""

    semi_axis_a: float
    semi_axis_b: float
    semi_axis_c: float


@dataclass(frozen=True)
class Parallelepiped:
    """A rectangular box with FULL edge lengths along axes 1, 2, 3.

    The edges are the full lengths of the box, not half-lengths, matching
    the closed form of DESIGN Section 3.2.
    """

    edge_a: float
    edge_b: float
    edge_c: float


@dataclass(frozen=True)
class Cube:
    """A cube of the given edge length.

    Written out separately from the box because it is a spherical top: a
    freely spinning cube tumbles exactly as a sphere does (DESIGN Section
    3.3), the demonstration that a body's symmetry, not its silhouette,
    decides its motion.
    """

    edge: float


@dataclass(frozen=True)
class Cylinder:
    """A solid cylinder, symmetry axis along body axis 3."""

    radius: float
    height: float


@dataclass(frozen=True)
class Cone:
    """A solid cone, symmetry axis along body axis 3.

    Its center of mass lies at height ``h / 4`` above the base, not at
    ``h / 2``, and its moments are taken about that center of mass
    (DESIGN Section 3.2). This is the one primitive with a trap in it.
    """

    base_radius: float
    height: float


@dataclass(frozen=True)
class Tetrahedron:
    """A regular tetrahedron of the given edge length (a spherical top)."""

    edge: float


@dataclass(frozen=True)
class Octahedron:
    """A regular octahedron of the given edge length (a spherical top)."""

    edge: float


@dataclass(frozen=True)
class Dodecahedron:
    """A regular dodecahedron of the given edge (a spherical top)."""

    edge: float


@dataclass(frozen=True)
class Icosahedron:
    """A regular icosahedron of the given edge (a spherical top)."""

    edge: float
