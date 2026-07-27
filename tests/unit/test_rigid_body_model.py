"""Unit tests for body classification (body/rigid_body_model.py).

The classification is checked against solids whose type is known by
construction: every Platonic solid and the sphere are spherical tops, a
cylinder is a symmetric top, and a box with distinct edges is asymmetric
with a definite intermediate axis (DESIGN Sections 3.3 and 1.4).
"""

import numpy as np

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import classify_top, TopClass


def moments_of(shape, density=1000.0):
    return np.diag(ai.analytic_inertia(shape, density).inertia_tensor)


def test_cube_is_a_spherical_top():
    top_class, intermediate_axis = classify_top(moments_of(shapes.Cube(0.2)))
    assert top_class is TopClass.SPHERICAL
    assert intermediate_axis is None


def test_icosahedron_is_a_spherical_top():
    top_class, intermediate_axis = classify_top(
        moments_of(shapes.Icosahedron(0.2)))
    assert top_class is TopClass.SPHERICAL
    assert intermediate_axis is None


def test_cylinder_is_a_symmetric_top():
    top_class, intermediate_axis = classify_top(
        moments_of(shapes.Cylinder(0.4, 1.0)))
    assert top_class is TopClass.SYMMETRIC
    assert intermediate_axis is None


def test_box_with_distinct_edges_is_asymmetric():
    # Edges 0.2, 0.3, 0.5 give moments ordered I_1 > I_2 > I_3, so the
    # median moment sits on axis 2 -- the intermediate axis.
    top_class, intermediate_axis = classify_top(
        moments_of(shapes.Parallelepiped(0.2, 0.3, 0.5)))
    assert top_class is TopClass.ASYMMETRIC
    assert intermediate_axis == 2


def test_intermediate_axis_is_the_median_moment():
    # Moments (2, 4, 3): the median 3 sits on axis 3.
    top_class, intermediate_axis = classify_top(np.array([2.0, 4.0, 3.0]))
    assert top_class is TopClass.ASYMMETRIC
    assert intermediate_axis == 3
