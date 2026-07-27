"""Unit tests for the torque models (dynamics/torque_models.py).

The oracles are exact: viscous damping is exactly ``-c omega`` and removes
energy; gravity through a pivot is ``r x F`` computed by hand, zero when
the lever arm is parallel to gravity and ``m g l`` for a horizontal lever;
and the total of an empty model list is the zero vector -- torque-free
motion is not a special case (DESIGN Sections 5.2, 5.3, 5.4).
"""

import numpy as np
import pytest

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import torque_models as tm


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])
GRAVITY_DOWN = np.array([0.0, 0.0, -9.81])


def make_body(shape, density=1000.0):
    properties = ai.analytic_inertia(shape, density)
    moments = np.diag(properties.inertia_tensor)
    top_class, intermediate_axis = classify_top(moments)
    return RigidBody(
        principal_moments=moments,
        principal_axes=np.eye(3),
        total_mass=properties.mass,
        center_of_mass=properties.center_of_mass,
        top_class=top_class,
        intermediate_axis=intermediate_axis,
        geometry=shape)


def test_viscous_damping_opposes_omega_and_removes_energy():
    model = tm.ViscousDamping(0.5)
    angular_velocity = np.array([1.0, -2.0, 0.5])
    state = st.State(IDENTITY_QUATERNION, angular_velocity)
    body = make_body(shapes.Cylinder(0.4, 1.0))

    torque = model.torque_body(0.0, state, body)
    np.testing.assert_allclose(torque, -0.5 * angular_velocity)

    # The power delivered is negative whenever the body is spinning.
    assert np.dot(torque, angular_velocity) < 0.0


def test_gravity_is_zero_when_the_lever_is_along_gravity():
    # Lever arm along +z, gravity along -z: parallel, so no torque -- the
    # upright top (DESIGN Section 5.3).
    body = make_body(shapes.Cone(0.3, 1.0))
    model = tm.GravityTorque(GRAVITY_DOWN, [0.0, 0.0, 0.25])
    state = st.State(IDENTITY_QUATERNION, np.zeros(3))
    torque = model.torque_body(0.0, state, body)
    np.testing.assert_allclose(torque, np.zeros(3), atol=1e-12)


def test_gravity_torque_from_a_horizontal_lever():
    # Lever arm along +x, gravity along -z at the identity orientation:
    # tau = [l, 0, 0] x [0, 0, -m g] = [0, l m g, 0].
    body = make_body(shapes.Cube(0.2))
    lever = 0.3
    model = tm.GravityTorque(GRAVITY_DOWN, [lever, 0.0, 0.0])
    state = st.State(IDENTITY_QUATERNION, np.zeros(3))
    torque = model.torque_body(0.0, state, body)
    expected_about_y = lever * body.total_mass * 9.81
    np.testing.assert_allclose(
        torque, [0.0, expected_about_y, 0.0], atol=1e-12)


def test_total_of_an_empty_list_is_the_zero_vector():
    body = make_body(shapes.Cube(0.2))
    state = st.State(IDENTITY_QUATERNION, np.array([1.0, 0.0, 0.0]))
    total = tm.total_torque_body(0.0, state, body, [])
    np.testing.assert_allclose(total, np.zeros(3))


def test_total_is_the_sum_of_the_models():
    body = make_body(shapes.Cube(0.2))
    state = st.State(IDENTITY_QUATERNION, np.array([1.0, -1.0, 0.5]))
    gravity = tm.GravityTorque(GRAVITY_DOWN, [0.3, 0.0, 0.0])
    damping = tm.ViscousDamping(0.2)
    total = tm.total_torque_body(0.0, state, body, [gravity, damping])
    expected = (gravity.torque_body(0.0, state, body)
                + damping.torque_body(0.0, state, body))
    np.testing.assert_allclose(total, expected)
