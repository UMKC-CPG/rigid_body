"""Unit tests for the analytic solutions (analysis/analytic_solutions.py).

The asymmetric-top forms are checked against the certified spike
``free_top_elliptic`` on both branches. The steady principal rotation and
the symmetric-top body rate are cross-checked by integrating the real
equations of motion and comparing. The heavy-top quadratic is checked
against its defining equation and its existence threshold.
"""

import importlib.util
import math
import os

import numpy as np
import pytest

from rigid_body.core import orientation as ori
from rigid_body.body import shapes
from rigid_body.body.rigid_body_model import (
    RigidBody, classify_top, build_body_from_shape)
from rigid_body.dynamics.state import State
from rigid_body.dynamics import equations_of_motion as eom
from rigid_body.dynamics import integrators as ig
from rigid_body.analysis import analytic_solutions as analytic


_SPIKE_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'dev', 'spikes', 'free_top_elliptic.py'))
_spec = importlib.util.spec_from_file_location(
    'free_top_elliptic_spike', _SPIKE_PATH)
free_top_spike = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(free_top_spike)


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])
MOMENTS = np.array([2.0, 3.0, 4.0])


def asymmetric_body(moments=MOMENTS):
    top_class, intermediate_axis = classify_top(moments)
    return RigidBody(
        principal_moments=np.asarray(moments, dtype=float),
        principal_axes=np.eye(3), total_mass=1.0,
        center_of_mass=np.zeros(3), top_class=top_class,
        intermediate_axis=intermediate_axis, geometry=None)


# --------------------------------------------------------------------
# The asymmetric top against the certified spike
# --------------------------------------------------------------------

@pytest.mark.parametrize("initial_spin", [
    [0.3, 0.0, 2.0],    # main branch: spin near axis 3
    [2.0, 0.0, 0.3],    # complementary branch: spin near axis 1
])
def test_asymmetric_parameters_match_the_spike(initial_spin):
    mine = analytic.free_asymmetric_top_parameters(MOMENTS, initial_spin)
    theirs = free_top_spike.elliptic_parameters(MOMENTS, initial_spin)
    np.testing.assert_allclose(
        mine.coefficients, theirs["coefficients"], rtol=1e-12)
    assert mine.axis_functions == tuple(theirs["axis_functions"])
    assert mine.rate == pytest.approx(theirs["rate"], rel=1e-12)
    assert mine.modulus_squared == pytest.approx(
        theirs["modulus_sq"], rel=1e-12)
    assert mine.period == pytest.approx(theirs["period"], rel=1e-12)


def test_asymmetric_evaluation_matches_the_spike():
    initial_spin = [0.3, 0.0, 2.0]
    mine_parameters = analytic.free_asymmetric_top_parameters(
        MOMENTS, initial_spin)
    their_parameters = free_top_spike.elliptic_parameters(
        MOMENTS, initial_spin)
    times = np.linspace(0.0, 2.0 * mine_parameters.period, 500)
    mine = analytic.evaluate_free_asymmetric_top(mine_parameters, times)
    theirs = free_top_spike.evaluate_analytic(their_parameters, times)
    np.testing.assert_allclose(mine, theirs, atol=1e-12)


def test_asymmetric_top_rejects_unordered_moments():
    with pytest.raises(ValueError):
        analytic.free_asymmetric_top_parameters(
            [3.0, 2.0, 4.0], [0.3, 0.0, 2.0])


def test_asymmetric_top_rejects_an_unanchored_spin():
    with pytest.raises(ValueError):
        analytic.free_asymmetric_top_parameters(
            MOMENTS, [0.3, 0.5, 2.0])


# --------------------------------------------------------------------
# Steady principal rotation, cross-checked by integration
# --------------------------------------------------------------------

def test_steady_rotation_matches_the_integrated_motion():
    body = asymmetric_body()
    initial = State(IDENTITY_QUATERNION.copy(), np.array([0.0, 0.0, 3.0]))
    integrator = ig.RungeKutta4Integrator()

    def derivative(time, state):
        return eom.state_derivative(time, state, body, [])

    state = initial
    dt, steps = 0.001, 500
    for _ in range(steps):
        state = integrator.advance(state, 0.0, dt, derivative)
    final_time = dt * steps

    analytic_state = analytic.steady_principal_rotation(initial, final_time)
    # omega is unchanged, and the orientation matches (up to the double
    # cover, so compare the rotation of a probe vector).
    np.testing.assert_allclose(
        state.angular_velocity_body, [0.0, 0.0, 3.0], atol=1e-9)
    probe = np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(
        ori.rotate_body_to_space(state.body_to_space_quaternion, probe),
        ori.rotate_body_to_space(
            analytic_state.body_to_space_quaternion, probe),
        atol=1e-6)


# --------------------------------------------------------------------
# The symmetric top
# --------------------------------------------------------------------

def test_symmetric_body_rate_matches_the_integrated_precession():
    body = build_body_from_shape(shapes.Cylinder(0.4, 1.0), 1000.0)
    spin_axial = 5.0
    transverse = 0.2
    initial = State(IDENTITY_QUATERNION.copy(),
                    np.array([transverse, 0.0, spin_axial]))

    body_rate, _ = analytic.symmetric_top_precession_rates(initial, body)

    integrator = ig.RungeKutta4Integrator()

    def derivative(time, state):
        return eom.state_derivative(time, state, body, [])

    state = initial
    dt, steps = 0.0005, 400
    angles = []
    times = []
    time = 0.0
    for _ in range(steps):
        state = integrator.advance(state, time, dt, derivative)
        time += dt
        omega = state.angular_velocity_body
        angles.append(math.atan2(omega[1], omega[0]))
        times.append(time)

    unwrapped = np.unwrap(angles)
    fitted_rate = np.polyfit(times, unwrapped, 1)[0]
    assert fitted_rate == pytest.approx(body_rate, rel=1e-3)


def test_symmetric_body_rate_runs_opposite_for_prolate_and_oblate():
    # The body-frame precession runs opposite ways for the two shapes. By
    # the formula Omega_body = omega_3 (I_3 - I_1) / I_1 -- confirmed
    # against the integrated motion above -- a rod-like (prolate) top with
    # I_3 < I_1 has a negative rate and a disk-like (oblate) top a
    # positive one. (This is the sign DESIGN Section 8.3's prose states
    # backwards; the formula and the integration are the authority.)
    prolate = build_body_from_shape(shapes.Cylinder(0.2, 2.0), 1000.0)
    oblate = build_body_from_shape(shapes.Cylinder(1.0, 0.2), 1000.0)
    spin = State(IDENTITY_QUATERNION, np.array([0.1, 0.0, 3.0]))
    prolate_rate, _ = analytic.symmetric_top_precession_rates(spin, prolate)
    oblate_rate, _ = analytic.symmetric_top_precession_rates(spin, oblate)
    assert prolate_rate < 0.0
    assert oblate_rate > 0.0


# --------------------------------------------------------------------
# The heavy symmetric top
# --------------------------------------------------------------------

def test_heavy_top_roots_satisfy_the_quadratic():
    body = build_body_from_shape(shapes.Cylinder(0.1, 1.0), 1000.0)
    spin_axial, tilt = 1000.0, math.pi / 4.0
    gravity, lever = 9.81, 0.5
    result = analytic.heavy_top_steady_precession(
        body, spin_axial, tilt, gravity, lever)
    assert result["exists"]

    moment_transverse = float(body.principal_moments[0])
    moment_axial = float(body.principal_moments[2])
    for root in (result["slow_root"], result["fast_root"]):
        value = (moment_transverse * math.cos(tilt) * root**2
                 - moment_axial * spin_axial * root
                 + body.total_mass * gravity * lever)
        assert value == pytest.approx(0.0, abs=1e-6)


def test_heavy_top_slow_root_is_the_familiar_precession():
    # The familiar M g l / (I_3 omega_3) is only the leading-order slow
    # root, so a genuinely fast spin is needed to approach it closely.
    body = build_body_from_shape(shapes.Cylinder(0.1, 1.0), 1000.0)
    spin_axial, tilt = 3000.0, math.pi / 6.0
    gravity, lever = 9.81, 0.5
    result = analytic.heavy_top_steady_precession(
        body, spin_axial, tilt, gravity, lever)
    moment_axial = float(body.principal_moments[2])
    familiar = body.total_mass * gravity * lever / (
        moment_axial * spin_axial)
    assert result["slow_root"] == pytest.approx(familiar, rel=1e-2)


def test_heavy_top_has_no_steady_precession_when_too_slow():
    body = build_body_from_shape(shapes.Cylinder(0.1, 1.0), 1000.0)
    # A tiny spin cannot sustain steady precession at this tilt.
    result = analytic.heavy_top_steady_precession(
        body, 0.01, math.pi / 4.0, 9.81, 0.5)
    assert result["exists"] is False
