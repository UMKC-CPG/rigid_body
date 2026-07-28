"""Unit tests for the Poinsot construction (geometry/poinsot.py).

The Poinsot geometry is judged against the invariants it is built from and
against the real motion it describes:

* The contact point ``rho`` lies exactly on the momental ellipsoid, and
  the invariable plane's normal is parallel to ``L`` at the fixed distance
  ``sqrt(2T) / |L|``.
* Every polhode point -- analytic or numerically intersected -- satisfies
  BOTH invariant quadrics to machine precision, in all three top classes.
* The certified analytic polhode and the independent numerical
  quadric-intersection trace the *same* closed curve.
* The polhode is the curve the real dynamics rides: an RK4 torque-free
  trajectory keeps every ``omega`` on the analytic polhode.
* The herpolhode lies in the invariable plane, and its band collapses to a
  single circle for a symmetric top but is a true annulus for an
  asymmetric one.
"""

import math

import numpy as np
import pytest

from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import integrators as ig
from rigid_body.dynamics import simulation_engine as engine
from rigid_body.geometry import poinsot as po


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])


def make_body(shape, density=1000.0):
    """Build a natural-orientation RigidBody from a closed-form shape."""
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


def invariants_of(state, body):
    """Return ``(2T, |L|^2)`` for a state -- the two Poinsot invariants."""
    twice_kinetic_energy = 2.0 * st.kinetic_energy(state, body)
    momentum_body = body.principal_moments * state.angular_velocity_body
    momentum_squared = float(np.sum(momentum_body**2))
    return twice_kinetic_energy, momentum_squared


def quadric_residuals(points, body):
    """Return the max deviation of ``points`` from the two quadrics.

    For points that lie on the polhode both should be zero: the energy
    quadric ``sum I_k w_k^2`` equals ``2T`` and the momentum quadric
    ``sum I_k^2 w_k^2`` equals ``|L|^2``, evaluated against the first
    point's own invariant values.
    """
    moments = body.principal_moments
    squared = np.asarray(points, dtype=float)**2
    energy_values = squared @ moments
    momentum_values = squared @ moments**2
    return (
        float(np.max(np.abs(energy_values - energy_values[0]))),
        float(np.max(np.abs(momentum_values - momentum_values[0]))))


# --------------------------------------------------------------------
# The momental ellipsoid and the contact point
# --------------------------------------------------------------------

def test_momental_ellipsoid_semi_axes_invert_the_moments():
    # Semi-axis 1/sqrt(I_k), so the LONGEST semi-axis lies along the
    # SMALLEST moment (DESIGN Section 9.2).
    body = make_body(ASYMMETRIC_BOX)
    ellipsoid = po.momental_ellipsoid(body)
    np.testing.assert_allclose(
        ellipsoid.semi_axes,
        1.0 / np.sqrt(body.principal_moments), rtol=1e-12)
    smallest_moment_axis = int(np.argmin(body.principal_moments))
    longest_semi_axis = int(np.argmax(ellipsoid.semi_axes))
    assert longest_semi_axis == smallest_moment_axis


def test_contact_point_lies_on_the_momental_ellipsoid():
    # rho . I . rho = 1 by construction (Section 10.2).
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    rho = po.poinsot_contact_point(state, body)
    on_surface = float(np.sum(body.principal_moments * rho**2))
    assert on_surface == pytest.approx(1.0, abs=1e-12)


def test_contact_point_is_parallel_to_omega():
    # rho is just omega rescaled, so it points the same way.
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.array([0.3, 0.5, 2.0])
    state = st.State(IDENTITY_QUATERNION, angular_velocity)
    rho = po.poinsot_contact_point(state, body)
    cross = np.cross(rho, angular_velocity)
    np.testing.assert_allclose(cross, np.zeros(3), atol=1e-14)


# --------------------------------------------------------------------
# The invariable plane
# --------------------------------------------------------------------

def test_invariable_plane_normal_is_along_angular_momentum():
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    plane = po.invariable_plane(state, body)
    momentum_space = st.angular_momentum_space(state, body)
    expected_normal = momentum_space / np.linalg.norm(momentum_space)
    np.testing.assert_allclose(plane.normal, expected_normal, atol=1e-14)
    assert float(np.linalg.norm(plane.normal)) == pytest.approx(1.0)


def test_invariable_plane_distance_is_root_two_t_over_l():
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    plane = po.invariable_plane(state, body)
    twice_kinetic_energy, momentum_squared = invariants_of(state, body)
    expected = math.sqrt(twice_kinetic_energy) / math.sqrt(
        momentum_squared)
    assert plane.distance == pytest.approx(expected, rel=1e-12)


def test_invariable_plane_is_fixed_through_a_torque_free_run():
    # The whole point of the plane: under torque-free motion its normal
    # and distance hold still (DESIGN Section 9.3). A wandering normal
    # would be numerical drift, not physics.
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.array([0.05, 2.5, 0.05])
    state = st.State(IDENTITY_QUATERNION, angular_velocity)
    integrator = ig.RungeKutta4Integrator()

    initial_plane = po.invariable_plane(state, body)
    time = 0.0
    max_normal_swing = 0.0
    max_distance_change = 0.0
    for _ in range(2000):
        state = engine.advance_one_substep(
            state, time, 1.0e-3, body, [], integrator)
        time = time + 1.0e-3
        plane = po.invariable_plane(state, body)
        swing = po_angle(initial_plane.normal, plane.normal)
        max_normal_swing = max(max_normal_swing, swing)
        max_distance_change = max(
            max_distance_change,
            abs(plane.distance - initial_plane.distance))

    assert max_normal_swing < 1.0e-4
    assert max_distance_change < 1.0e-6


def po_angle(first, second):
    """Angle between two unit-ish vectors, clipped for safety."""
    cosine = float(np.dot(first, second)) / (
        np.linalg.norm(first) * np.linalg.norm(second))
    return math.acos(max(-1.0, min(1.0, cosine)))


# --------------------------------------------------------------------
# The polhode: on both quadrics, all three classes
# --------------------------------------------------------------------

def test_asymmetric_polhode_lies_on_both_quadrics():
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    curve = po.polhode(body, state, sample_count=240)
    assert curve.kind == "elliptic"
    twice_kinetic_energy, momentum_squared = invariants_of(state, body)
    energy_drift, momentum_drift = quadric_residuals(curve.points, body)
    assert energy_drift < 1e-10
    assert momentum_drift < 1e-10
    # And the invariants match the state, not just each other.
    assert float(curve.points[0] @ (body.principal_moments
                 * curve.points[0])) == pytest.approx(
                     twice_kinetic_energy, rel=1e-10)


def test_symmetric_polhode_is_a_circle_on_the_quadrics():
    body = make_body(shapes.Cylinder(0.4, 1.0))
    state = st.State(IDENTITY_QUATERNION, np.array([0.5, 0.2, 3.0]))
    curve = po.polhode(body, state, sample_count=120)
    assert curve.kind == "circle"
    energy_drift, momentum_drift = quadric_residuals(curve.points, body)
    assert energy_drift < 1e-10
    assert momentum_drift < 1e-10


def test_spherical_polhode_is_a_single_point():
    body = make_body(shapes.Cube(0.2))
    angular_velocity = np.array([1.0, 2.0, 3.0])
    state = st.State(IDENTITY_QUATERNION, angular_velocity)
    curve = po.polhode(body, state, sample_count=50)
    assert curve.kind == "point"
    assert curve.points.shape == (1, 3)
    np.testing.assert_allclose(curve.points[0], angular_velocity)


def test_asymmetric_polhode_is_a_closed_loop():
    # The analytic sampler spans one full body period, so the last point
    # returns to the first: the polhode is a closed curve.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    curve = po.polhode(body, state, sample_count=400)
    np.testing.assert_allclose(
        curve.points[0], curve.points[-1], atol=1e-6)


# --------------------------------------------------------------------
# Degenerate case: spin about a principal axis is a point, not a crash
# --------------------------------------------------------------------

@pytest.mark.parametrize("axis", [0, 1, 2])
def test_asymmetric_spin_about_a_principal_axis_is_a_point(axis):
    # A steady rotation about any principal axis of an asymmetric top has a
    # single-point polhode (Section 9.2); the elliptic reconstruction would
    # take a square root of a rounding-sized negative at that boundary, so
    # the code must recognize the point. This includes the intermediate
    # axis, whose separatrix would otherwise give an infinite period.
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.zeros(3)
    angular_velocity[axis] = 3.0
    curve = po.polhode(
        body, st.State(IDENTITY_QUATERNION, angular_velocity),
        sample_count=240)
    assert curve.kind == "point"
    assert curve.points.shape == (1, 3)
    np.testing.assert_array_equal(curve.points[0], angular_velocity)
    # The point sits on both invariant quadrics (trivially, being omega).
    energy_drift, momentum_drift = quadric_residuals(curve.points, body)
    assert energy_drift < 1e-12
    assert momentum_drift < 1e-12


def test_near_principal_axis_spin_is_treated_as_a_point():
    # A spin only negligibly off a principal axis is a point too, well
    # inside the tolerance that separates it from a visible loop.
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.array([3.0, 3.0e-7, 1.0e-7])
    curve = po.polhode(
        body, st.State(IDENTITY_QUATERNION, angular_velocity),
        sample_count=240)
    assert curve.kind == "point"


def test_appreciably_off_axis_spin_stays_an_elliptic_loop():
    # A spin clearly off the principal axis is a real polhode loop, not a
    # point: the degeneracy guard must not swallow genuine curves.
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.array([3.0, 0.3, 0.1])
    curve = po.polhode(
        body, st.State(IDENTITY_QUATERNION, angular_velocity),
        sample_count=240)
    assert curve.kind == "elliptic"
    assert curve.points.shape[0] == 240
    energy_drift, momentum_drift = quadric_residuals(curve.points, body)
    assert energy_drift < 1e-9
    assert momentum_drift < 1e-9


def test_principal_axis_spin_predicate():
    assert po._is_principal_axis_spin(np.array([0.0, 0.0, 0.0]))
    assert po._is_principal_axis_spin(np.array([5.0, 0.0, 0.0]))
    assert po._is_principal_axis_spin(np.array([0.0, -2.0, 0.0]))
    assert not po._is_principal_axis_spin(np.array([3.0, 0.3, 0.1]))
    assert not po._is_principal_axis_spin(np.array([1.0, 1.0, 1.0]))


# --------------------------------------------------------------------
# The numerical fallback agrees with the certified analytic form
# --------------------------------------------------------------------

def test_numeric_intersection_satisfies_both_quadrics():
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    twice_kinetic_energy, momentum_squared = invariants_of(state, body)
    curve = po.intersect_quadrics(
        body, twice_kinetic_energy, momentum_squared, sample_count=240)
    energy_values = curve**2 @ body.principal_moments
    momentum_values = curve**2 @ body.principal_moments**2
    assert float(np.max(np.abs(
        energy_values - twice_kinetic_energy))) < 1e-12
    assert float(np.max(np.abs(
        momentum_values - momentum_squared))) < 1e-12


def resample_uniformly(points, count):
    """Resample a closed loop to ``count`` points evenly along its length.

    The two polhode samplers space their points differently along the
    curve, so a fair "same curve?" comparison first regularizes the
    spacing: walk the closed loop by cumulative chord length and
    interpolate each component onto an evenly spaced arc-length grid.
    """
    closed = np.vstack([points, points[:1]])
    segment_lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    arc_length = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    targets = np.linspace(0.0, arc_length[-1], count, endpoint=False)
    return np.stack(
        [np.interp(targets, arc_length, closed[:, axis])
         for axis in range(3)], axis=1)


def test_numeric_and_analytic_polhodes_trace_the_same_curve():
    # Two independent routes to the polhode must land on one curve: after
    # regularizing the spacing, every analytic sample has a numerical
    # sample essentially on top of it.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    twice_kinetic_energy, momentum_squared = invariants_of(state, body)
    analytic = po.polhode(body, state, sample_count=600).points
    numeric = resample_uniformly(
        po.intersect_quadrics(
            body, twice_kinetic_energy, momentum_squared,
            sample_count=4000),
        4000)

    # Nearest-neighbor distance from each analytic point to the numeric
    # loop; small because both trace the same closed curve.
    differences = analytic[:, None, :] - numeric[None, :, :]
    nearest = np.min(np.linalg.norm(differences, axis=2), axis=1)
    assert float(np.max(nearest)) < 3e-3


# --------------------------------------------------------------------
# The polhode is the curve the real dynamics rides
# --------------------------------------------------------------------

def test_simulated_omega_stays_on_the_polhode():
    # The strongest tie to the physics: an RK4 torque-free trajectory of
    # the real Euler equations keeps omega on the analytic polhode curve.
    body = make_body(ASYMMETRIC_BOX)
    initial = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    curve = po.polhode(body, initial, sample_count=800).points
    integrator = ig.RungeKutta4Integrator()

    state = initial
    time = 0.0
    max_distance = 0.0
    for _ in range(1500):
        state = engine.advance_one_substep(
            state, time, 1.0e-3, body, [], integrator)
        time = time + 1.0e-3
        differences = curve - state.angular_velocity_body
        nearest = float(np.min(np.linalg.norm(differences, axis=1)))
        max_distance = max(max_distance, nearest)
    assert max_distance < 2e-2


# --------------------------------------------------------------------
# The herpolhode
# --------------------------------------------------------------------

def test_herpolhode_point_lies_in_the_invariable_plane():
    # Each contact point mapped to space sits at the fixed plane distance
    # along the plane normal, throughout a torque-free run.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.2, 2.0, 0.3]))
    integrator = ig.RungeKutta4Integrator()
    plane = po.invariable_plane(state, body)

    time = 0.0
    for _ in range(500):
        state = engine.advance_one_substep(
            state, time, 1.0e-3, body, [], integrator)
        time = time + 1.0e-3
        space_point = po.herpolhode_point(state, body)
        height = float(np.dot(space_point, plane.normal))
        assert height == pytest.approx(plane.distance, abs=1e-4)


def test_herpolhode_band_is_a_circle_for_a_symmetric_top():
    # A symmetric top's polhode is a circle at constant |rho|, so the
    # herpolhode is a single circle: inner and outer radii coincide.
    body = make_body(shapes.Cylinder(0.4, 1.0))
    state = st.State(IDENTITY_QUATERNION, np.array([0.5, 0.2, 3.0]))
    twice_kinetic_energy, _ = invariants_of(state, body)
    plane = po.invariable_plane(state, body)
    curve = po.polhode(body, state, sample_count=240)
    bounds = po.herpolhode_bounds(
        curve.points, twice_kinetic_energy, plane.distance)
    assert bounds.inner_radius == pytest.approx(
        bounds.outer_radius, abs=1e-9)


def test_herpolhode_band_is_an_annulus_for_an_asymmetric_top():
    # An asymmetric top's contact point approaches and recedes from L, so
    # the band has a genuine inner and outer radius.
    body = make_body(ASYMMETRIC_BOX)
    state = st.State(IDENTITY_QUATERNION, np.array([0.3, 0.5, 2.0]))
    twice_kinetic_energy, _ = invariants_of(state, body)
    plane = po.invariable_plane(state, body)
    curve = po.polhode(body, state, sample_count=240)
    bounds = po.herpolhode_bounds(
        curve.points, twice_kinetic_energy, plane.distance)
    assert 0.0 <= bounds.inner_radius
    assert bounds.inner_radius < bounds.outer_radius - 1e-6
