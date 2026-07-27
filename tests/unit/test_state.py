"""Unit tests for the state and its derived quantities (dynamics/state).

The oracles are exact identities: the body-frame momentum is componentwise
``I_k omega_k``; a rotation preserves the length of the momentum vector, so
``|L|`` is frame-independent; and ``2T = omega . L`` is an algebraic
identity that holds at every instant (DESIGN Section 2.5).
"""

import numpy as np
import pytest

from rigid_body.core import orientation as ori
from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])


def make_body(shape, density=1000.0):
    """Assemble a RigidBody record from a shape, for use as an oracle."""
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


def test_angular_momentum_body_is_componentwise():
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    angular_velocity = np.array([1.0, -2.0, 0.5])
    state = st.State(IDENTITY_QUATERNION, angular_velocity)
    momentum = st.angular_momentum_body(state, body)
    np.testing.assert_allclose(
        momentum, body.principal_moments * angular_velocity)


def test_space_momentum_equals_body_momentum_at_identity():
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    state = st.State(IDENTITY_QUATERNION, np.array([1.0, -2.0, 0.5]))
    np.testing.assert_allclose(
        st.angular_momentum_space(state, body),
        st.angular_momentum_body(state, body), atol=1e-12)


def test_momentum_magnitude_is_frame_independent():
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    quaternion = ori.normalize_quaternion(
        np.array([0.5, 0.5, -0.3, 0.7]))
    state = st.State(quaternion, np.array([1.0, -2.0, 0.5]))
    momentum_body = st.angular_momentum_body(state, body)
    momentum_space = st.angular_momentum_space(state, body)
    assert np.linalg.norm(momentum_space) == pytest.approx(
        np.linalg.norm(momentum_body))


def test_kinetic_energy_follows_the_formula():
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    angular_velocity = np.array([1.0, -2.0, 0.5])
    state = st.State(IDENTITY_QUATERNION, angular_velocity)
    expected = 0.5 * np.sum(body.principal_moments * angular_velocity**2)
    assert st.kinetic_energy(state, body) == pytest.approx(expected)


def test_poinsot_identity_twice_energy_equals_omega_dot_momentum():
    # 2T = omega . L holds at every instant by definition (DESIGN 7.6).
    body = make_body(shapes.Parallelepiped(0.2, 0.3, 0.5))
    angular_velocity = np.array([1.0, -2.0, 0.5])
    state = st.State(IDENTITY_QUATERNION, angular_velocity)
    twice_energy = 2.0 * st.kinetic_energy(state, body)
    momentum_body = st.angular_momentum_body(state, body)
    assert twice_energy == pytest.approx(
        np.dot(angular_velocity, momentum_body))
