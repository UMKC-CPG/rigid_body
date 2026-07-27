"""Unit tests for the integrators (dynamics/integrators.py).

The centerpiece is the end-to-end oracle: integrate the torque-free
``state_derivative`` with RK4 and compare the angular-velocity trajectory
to the exact Jacobi-elliptic solution that ``free_top_elliptic`` certifies
(DESIGN Section 8.4). A wrong derivative, a wrong RK4 weighting, or a
misplaced renormalization would all diverge from it. The test also checks
the fourth-order convergence claim, that the integrator conserves energy
and squared angular momentum to a tight bound over the run, and that the
quaternion stays on the unit sphere.
"""

import importlib.util
import os

import numpy as np
import pytest

from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import equations_of_motion as eom
from rigid_body.dynamics import integrators as ig


_SPIKE_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'dev', 'spikes', 'free_top_elliptic.py'))
_spec = importlib.util.spec_from_file_location(
    'free_top_elliptic_spike', _SPIKE_PATH)
free_top_spike = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(free_top_spike)


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])

# An asymmetric body with moments already ordered I_1 < I_2 < I_3, and an
# initial spin anchored at omega_2 = 0 -- the phase the elliptic solution
# is written from (spin nearest axis 3, the main branch).
MOMENTS = np.array([2.0, 3.0, 4.0])
INITIAL_SPIN = np.array([0.3, 0.0, 2.0])


def asymmetric_body():
    top_class, intermediate_axis = classify_top(MOMENTS)
    return RigidBody(
        principal_moments=MOMENTS,
        principal_axes=np.eye(3),
        total_mass=1.0,
        center_of_mass=np.zeros(3),
        top_class=top_class,
        intermediate_axis=intermediate_axis,
        geometry=None)


def integrate_free_top(step_count, span):
    """Integrate the torque-free top and return sampled (times, omega)."""
    body = asymmetric_body()
    integrator = ig.RungeKutta4Integrator()

    def derivative(time, state):
        return eom.state_derivative(time, state, body, [])

    dt = span / step_count
    state = st.State(IDENTITY_QUATERNION.copy(), INITIAL_SPIN.copy())
    times = [0.0]
    angular_velocities = [state.angular_velocity_body]
    time = 0.0
    for _ in range(step_count):
        state = integrator.advance(state, time, dt, derivative)
        time += dt
        times.append(time)
        angular_velocities.append(state.angular_velocity_body)
    return np.array(times), np.array(angular_velocities), state


# --------------------------------------------------------------------
# Field-wise arithmetic and renormalization
# --------------------------------------------------------------------

def test_scale_and_add_states_are_field_wise():
    first = st.State(np.array([1.0, 2.0, 3.0, 4.0]),
                     np.array([1.0, 0.0, -1.0]))
    scaled = ig.scale_state(2.0, first)
    np.testing.assert_allclose(
        scaled.body_to_space_quaternion, [2.0, 4.0, 6.0, 8.0])
    np.testing.assert_allclose(
        scaled.angular_velocity_body, [2.0, 0.0, -2.0])

    summed = ig.add_states(first, scaled)
    np.testing.assert_allclose(
        summed.body_to_space_quaternion, [3.0, 6.0, 9.0, 12.0])
    np.testing.assert_allclose(
        summed.angular_velocity_body, [3.0, 0.0, -3.0])


def test_renormalize_gives_unit_quaternion_keeping_sign_and_omega():
    raw = st.State(np.array([-2.0, 0.0, 0.0, 0.0]),
                   np.array([1.0, 2.0, 3.0]))
    result = ig.renormalize(raw)
    assert np.linalg.norm(result.body_to_space_quaternion) == (
        pytest.approx(1.0))
    assert result.body_to_space_quaternion[0] < 0.0   # sign kept
    np.testing.assert_allclose(
        result.angular_velocity_body, [1.0, 2.0, 3.0])


# --------------------------------------------------------------------
# The end-to-end oracle: RK4 vs the analytic elliptic solution
# --------------------------------------------------------------------

def test_rk4_reproduces_the_analytic_free_top():
    parameters = free_top_spike.elliptic_parameters(MOMENTS, INITIAL_SPIN)
    span = 2.0 * parameters['period']

    times, angular_velocities, _ = integrate_free_top(4000, span)
    analytic = free_top_spike.evaluate_analytic(parameters, times)

    velocity_scale = np.max(np.abs(analytic))
    max_deviation = np.max(np.abs(angular_velocities - analytic))
    assert max_deviation / velocity_scale < 1.0e-5


def test_rk4_convergence_is_fourth_order():
    # Halving dt should cut the error by roughly sixteen (order four).
    parameters = free_top_spike.elliptic_parameters(MOMENTS, INITIAL_SPIN)
    span = parameters['period']

    def peak_error(step_count):
        times, omega, _ = integrate_free_top(step_count, span)
        analytic = free_top_spike.evaluate_analytic(parameters, times)
        return np.max(np.abs(omega - analytic))

    coarse = peak_error(250)
    fine = peak_error(500)
    # Order four predicts a ratio near 16; require clearly better than
    # third order, and confirm both errors are above the round-off floor.
    assert coarse > 1.0e-12 and fine > 1.0e-14
    assert coarse / fine > 10.0


def test_integration_conserves_energy_and_momentum():
    body = asymmetric_body()
    parameters = free_top_spike.elliptic_parameters(MOMENTS, INITIAL_SPIN)
    span = 2.0 * parameters['period']

    initial = st.State(IDENTITY_QUATERNION.copy(), INITIAL_SPIN.copy())
    energy_0 = st.kinetic_energy(initial, body)
    momentum_squared_0 = np.sum(
        st.angular_momentum_body(initial, body)**2)

    _, angular_velocities, _ = integrate_free_top(4000, span)
    moments = body.principal_moments
    energy = 0.5 * np.sum(moments * angular_velocities**2, axis=1)
    momentum_squared = np.sum(
        (moments * angular_velocities)**2, axis=1)

    assert np.max(np.abs(energy - energy_0)) / energy_0 < 1.0e-6
    assert np.max(np.abs(momentum_squared - momentum_squared_0)) / (
        momentum_squared_0) < 1.0e-6


def test_quaternion_stays_on_the_unit_sphere():
    parameters = free_top_spike.elliptic_parameters(MOMENTS, INITIAL_SPIN)
    span = 2.0 * parameters['period']
    _, _, final_state = integrate_free_top(4000, span)
    assert np.linalg.norm(
        final_state.body_to_space_quaternion) == pytest.approx(1.0)


# --------------------------------------------------------------------
# Selection
# --------------------------------------------------------------------

def test_select_returns_rk4_by_default():
    assert isinstance(
        ig.select_integrator('rk4'), ig.RungeKutta4Integrator)


@pytest.mark.parametrize('name', ['implicit_midpoint', 'splitting'])
def test_select_refuses_the_unbuilt_symplectic_schemes(name):
    with pytest.raises(NotImplementedError):
        ig.select_integrator(name)
