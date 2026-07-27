"""Unit tests for the closed-form inertia (body/analytic_inertia.py).

The strongest oracle here is the Platonic set. Each Platonic solid is a
spherical top, and its inertia factor ``I / (M a^2)`` is checked against
the *exact tetrahedron decomposition* in ``dev/spikes/platonic_inertia``
-- an independent computation, so a wrong constant (the factor-of-two
icosahedron trap of DESIGN Section 3.2 among them) cannot slip through.
The round and rectangular solids are checked against their closed forms,
the physical validity of DESIGN Section 3.1, and internal cross-checks
such as a cube being a box with equal edges.
"""

import importlib.util
import math
import os

import numpy as np
import pytest
from scipy.spatial import ConvexHull

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai


# Load the certifying spike by path; it lives outside the package under
# dev/spikes and supplies the exact decomposition used as the oracle.
_SPIKE_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'dev', 'spikes', 'platonic_inertia.py'))
_spec = importlib.util.spec_from_file_location(
    'platonic_inertia_spike', _SPIKE_PATH)
platonic_spike = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(platonic_spike)


# Map each Platonic name to its shape class; every one takes a single
# ``edge`` argument.
PLATONIC_SHAPES = {
    'tetrahedron': shapes.Tetrahedron,
    'cube': shapes.Cube,
    'octahedron': shapes.Octahedron,
    'dodecahedron': shapes.Dodecahedron,
    'icosahedron': shapes.Icosahedron,
}


# --------------------------------------------------------------------
# The Platonic oracle: closed form vs exact decomposition
# --------------------------------------------------------------------

@pytest.mark.parametrize('name', list(PLATONIC_SHAPES))
def test_platonic_moments_match_exact_decomposition(name):
    edge = 0.37
    mass = 2.4

    # Independent ground truth: the exact tetrahedron decomposition,
    # returned as the tensor I / (M * edge^2). Its mean eigenvalue is the
    # isotropic inertia factor.
    vertices = platonic_spike.platonic_vertices()[name]
    exact_tensor = platonic_spike.exact_inertia_factor(vertices)
    expected_factor = float(np.linalg.eigvalsh(exact_tensor).mean())

    moments = ai.principal_moments_of(PLATONIC_SHAPES[name](edge), mass)

    # A spherical top: all three principal moments equal.
    np.testing.assert_allclose(moments, moments[0], rtol=1e-12)

    my_factor = moments[0] / (mass * edge**2)
    assert my_factor == pytest.approx(expected_factor, rel=1e-9)


def test_icosahedron_uses_the_over_twenty_constant():
    # Guard the specific factor-of-two trap: (3 + sqrt(5)) / 20, not / 10.
    moments = ai.principal_moments_of(shapes.Icosahedron(1.0), 1.0)
    expected = (3.0 + math.sqrt(5.0)) / 20.0
    assert moments[0] == pytest.approx(expected, rel=1e-12)
    # The wrong value would be exactly double.
    assert moments[0] != pytest.approx(2.0 * expected, rel=1e-3)


@pytest.mark.parametrize('name', list(PLATONIC_SHAPES))
def test_platonic_volume_matches_the_convex_hull(name):
    # The closed-form volume, checked against the exact hull volume of the
    # same solid scaled to unit edge (an independent oracle).
    vertices = platonic_spike.platonic_vertices()[name]
    edge = platonic_spike.shortest_edge_length(vertices)
    hull_volume_per_unit_edge = ConvexHull(vertices).volume / edge**3

    my_volume = ai.volume_of(PLATONIC_SHAPES[name](1.0))
    assert my_volume == pytest.approx(hull_volume_per_unit_edge, rel=1e-9)


# --------------------------------------------------------------------
# The round and rectangular solids
# --------------------------------------------------------------------

def test_sphere_is_isotropic_two_fifths():
    mass, radius = 3.0, 0.5
    moments = ai.principal_moments_of(shapes.Sphere(radius), mass)
    expected = (2.0 / 5.0) * mass * radius**2
    np.testing.assert_allclose(moments, expected, rtol=1e-12)


def test_cube_is_a_box_with_equal_edges():
    mass, edge = 2.0, 0.3
    as_cube = ai.principal_moments_of(shapes.Cube(edge), mass)
    as_box = ai.principal_moments_of(
        shapes.Parallelepiped(edge, edge, edge), mass)
    np.testing.assert_allclose(as_cube, as_box, rtol=1e-12)
    np.testing.assert_allclose(
        as_cube, (1.0 / 6.0) * mass * edge**2, rtol=1e-12)


def test_box_moments_follow_the_closed_form():
    mass = 1.5
    edge_a, edge_b, edge_c = 0.2, 0.3, 0.5
    moments = ai.principal_moments_of(
        shapes.Parallelepiped(edge_a, edge_b, edge_c), mass)
    expected = (mass / 12.0) * np.array([
        edge_b**2 + edge_c**2,
        edge_c**2 + edge_a**2,
        edge_a**2 + edge_b**2,
    ])
    np.testing.assert_allclose(moments, expected, rtol=1e-12)


def test_cylinder_transverse_and_axial_moments():
    mass, radius, height = 4.0, 0.4, 1.0
    moments = ai.principal_moments_of(
        shapes.Cylinder(radius, height), mass)
    transverse = (mass / 12.0) * (3.0 * radius**2 + height**2)
    axial = 0.5 * mass * radius**2
    np.testing.assert_allclose(
        moments, [transverse, transverse, axial], rtol=1e-12)


def test_cone_moments_are_taken_about_the_center_of_mass():
    mass, radius, height = 2.0, 0.5, 0.9
    moments = ai.principal_moments_of(shapes.Cone(radius, height), mass)
    transverse = (3.0 * mass / 80.0) * (4.0 * radius**2 + height**2)
    axial = (3.0 / 10.0) * mass * radius**2
    np.testing.assert_allclose(
        moments, [transverse, transverse, axial], rtol=1e-12)


def test_ellipsoid_reduces_to_a_sphere_when_axes_are_equal():
    mass, radius = 2.5, 0.6
    moments = ai.principal_moments_of(
        shapes.Ellipsoid(radius, radius, radius), mass)
    expected = (2.0 / 5.0) * mass * radius**2
    np.testing.assert_allclose(moments, expected, rtol=1e-12)


# --------------------------------------------------------------------
# Physical validity (DESIGN Section 3.1) and the full provider
# --------------------------------------------------------------------

@pytest.mark.parametrize('shape', [
    shapes.Sphere(0.5),
    shapes.Ellipsoid(0.2, 0.3, 0.5),
    shapes.Parallelepiped(0.2, 0.3, 0.5),
    shapes.Cylinder(0.4, 1.0),
    shapes.Cone(0.5, 0.9),
    shapes.Icosahedron(0.3),
])
def test_moments_are_positive_and_obey_the_triangle_inequalities(shape):
    moments = ai.principal_moments_of(shape, 1.0)
    assert np.all(moments > 0.0)
    first, second, third = moments
    # No distribution of mass can fail these (DESIGN Section 3.1).
    assert first + second >= third - 1e-12
    assert second + third >= first - 1e-12
    assert third + first >= second - 1e-12


def test_provider_returns_mass_centered_diagonal_tensor():
    density, edge = 2700.0, 0.1
    result = ai.analytic_inertia(shapes.Cube(edge), density)

    assert result.mass == pytest.approx(density * edge**3, rel=1e-12)
    np.testing.assert_allclose(result.center_of_mass, np.zeros(3))

    # Diagonal, and equal to the principal moments on the diagonal.
    off_diagonal = result.inertia_tensor - np.diag(
        np.diag(result.inertia_tensor))
    np.testing.assert_allclose(off_diagonal, np.zeros((3, 3)), atol=1e-15)
    expected_moment = (1.0 / 6.0) * result.mass * edge**2
    np.testing.assert_allclose(
        np.diag(result.inertia_tensor), expected_moment, rtol=1e-12)
