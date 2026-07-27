"""Unit tests for Euler's equations (dynamics/equations_of_motion.py).

The strongest oracle is the certifying spike ``free_top_elliptic``: its
``euler_derivative`` is the torque-free right-hand side, which the code
here must reproduce exactly, and its growth-rate formula is the one
``instability_growth_rate`` implements. Two conservation identities are
also checked directly at the derivative level -- under torque-free motion
both the energy and the squared angular momentum have zero instantaneous
rate -- along with the purity of the derivative and the interface guard
that zeroes the growth rate for a symmetric or spherical top.
"""

import importlib.util
import math
import os

import numpy as np
import pytest

from rigid_body.core import orientation as ori
from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import torque_models as tm
from rigid_body.dynamics import equations_of_motion as eom


# Load the certifying spike by path; its euler_derivative is the oracle.
_SPIKE_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'dev', 'spikes', 'free_top_elliptic.py'))
_spec = importlib.util.spec_from_file_location(
    'free_top_elliptic_spike', _SPIKE_PATH)
free_top_spike = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(free_top_spike)


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])


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


ASYMMETRIC_BOX = shapes.Parallelepiped(0.2, 0.3, 0.5)


# --------------------------------------------------------------------
# Euler's equations against the spike, and conservation at the derivative
# --------------------------------------------------------------------

def test_torque_free_acceleration_matches_the_spike():
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.array([0.3, 0.5, 2.0])
    mine = eom.angular_acceleration_body(
        angular_velocity, body, np.zeros(3))
    theirs = free_top_spike.euler_derivative(
        0.0, angular_velocity, body.principal_moments)
    np.testing.assert_allclose(mine, theirs, atol=1e-12)


def test_torque_free_conserves_energy_instantaneously():
    # dT/dt = omega . (I omega_dot) = 0 under torque-free motion.
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.array([0.3, 0.5, 2.0])
    rate = eom.angular_acceleration_body(
        angular_velocity, body, np.zeros(3))
    energy_change = np.dot(
        angular_velocity, body.principal_moments * rate)
    assert energy_change == pytest.approx(0.0, abs=1e-12)


def test_torque_free_conserves_momentum_magnitude():
    # d|L|^2/dt = 2 L . (I omega_dot) = 0 under torque-free motion.
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.array([0.3, 0.5, 2.0])
    rate = eom.angular_acceleration_body(
        angular_velocity, body, np.zeros(3))
    momentum_body = body.principal_moments * angular_velocity
    momentum_change = np.dot(
        momentum_body, body.principal_moments * rate)
    assert momentum_change == pytest.approx(0.0, abs=1e-12)


def test_spin_about_a_principal_axis_is_an_equilibrium():
    body = make_body(ASYMMETRIC_BOX)
    for axis in range(3):
        angular_velocity = np.zeros(3)
        angular_velocity[axis] = 2.0
        rate = eom.angular_acceleration_body(
            angular_velocity, body, np.zeros(3))
        np.testing.assert_allclose(rate, np.zeros(3), atol=1e-12)


# --------------------------------------------------------------------
# The full derivative
# --------------------------------------------------------------------

def test_state_derivative_stitches_the_two_halves():
    body = make_body(ASYMMETRIC_BOX)
    quaternion = ori.normalize_quaternion(
        np.array([0.5, 0.5, -0.3, 0.7]))
    angular_velocity = np.array([0.3, 0.5, 2.0])
    state = st.State(quaternion, angular_velocity)

    derivative = eom.state_derivative(0.0, state, body, [])
    np.testing.assert_allclose(
        derivative.body_to_space_quaternion,
        ori.orientation_derivative(quaternion, angular_velocity),
        atol=1e-14)
    np.testing.assert_allclose(
        derivative.angular_velocity_body,
        eom.angular_acceleration_body(
            angular_velocity, body, np.zeros(3)),
        atol=1e-14)


def test_state_derivative_does_not_mutate_the_state():
    body = make_body(shapes.Cube(0.2))
    quaternion = ori.normalize_quaternion(np.array([0.5, 0.5, -0.3, 0.7]))
    angular_velocity = np.array([1.0, -1.0, 0.5])
    state = st.State(quaternion.copy(), angular_velocity.copy())
    models = [tm.GravityTorque([0.0, 0.0, -9.81], [0.3, 0.0, 0.0]),
              tm.ViscousDamping(0.2)]

    eom.state_derivative(0.0, state, body, models)

    np.testing.assert_array_equal(
        state.body_to_space_quaternion, quaternion)
    np.testing.assert_array_equal(
        state.angular_velocity_body, angular_velocity)


# --------------------------------------------------------------------
# The intermediate-axis growth rate and its interface guard
# --------------------------------------------------------------------

def test_growth_rate_matches_the_spike_formula():
    body = make_body(ASYMMETRIC_BOX)
    spin_rate = 2.0
    mine = eom.instability_growth_rate(body, spin_rate)
    moment_min, moment_mid, moment_max = np.sort(body.principal_moments)
    expected = spin_rate * math.sqrt(
        (moment_max - moment_mid) * (moment_mid - moment_min)
        / (moment_min * moment_max))
    assert mine == pytest.approx(expected, rel=1e-12)
    assert mine > 0.0


def test_growth_rate_is_zero_for_a_symmetric_top():
    body = make_body(shapes.Cylinder(0.4, 1.0))
    assert eom.instability_growth_rate(body, 3.0) == 0.0


def test_growth_rate_is_zero_for_a_spherical_top():
    body = make_body(shapes.Cube(0.2))
    assert eom.instability_growth_rate(body, 3.0) == 0.0


def test_time_to_flip_is_infinite_without_instability():
    assert eom.estimate_time_to_flip(0.0, 1.0e-4) == math.inf


def test_time_to_flip_follows_the_logarithmic_estimate():
    growth_rate, initial_tilt = 0.5, 1.0e-4
    expected = (1.0 / growth_rate) * math.log(1.0 / initial_tilt)
    assert eom.estimate_time_to_flip(growth_rate, initial_tilt) == (
        pytest.approx(expected))


def test_energy_rate_is_the_power_delivered():
    # Viscous damping delivers power Gamma . omega = -c |omega|^2.
    body = make_body(shapes.Cube(0.2))
    angular_velocity = np.array([1.0, -2.0, 0.5])
    state = st.State(IDENTITY_QUATERNION, angular_velocity)
    torque = tm.ViscousDamping(0.3).torque_body(0.0, state, body)
    rate = eom.energy_rate(state, torque)
    assert rate == pytest.approx(-0.3 * np.dot(
        angular_velocity, angular_velocity))
