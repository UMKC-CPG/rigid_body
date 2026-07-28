"""Unit tests for the conservation monitor (analysis/conservation_monitor).

The monitor's contract is subtle, so the tests pin each clause of DESIGN
Section 7 to a sharp oracle:

* Fed the *exact* analytic motion, which conserves energy and momentum to
  machine precision, every drift rate must read essentially zero.
* Fed a good numerical trajectory, the drift rates must be small.
* The decisive one -- balance, not constancy: under viscous damping the
  raw energy falls a long way, yet the monitor's *balance* residual stays
  tiny, because the fall is exactly what the rate law predicts. The naive
  constancy check would light up; the monitor does not.
* Under gravity the space-frame angular momentum genuinely swings, yet the
  momentum balance residual stays tiny for the same reason.
* The monitor never mutates the state it is handed.
* ``poinsot_identity_residual`` reads zero even on a physically wrong
  state, which is exactly why it is a code oracle and not a live monitor.
* The trend tracker reports a positive growth slope for a secular residual
  and near-zero for a bounded one.
"""

import numpy as np
import pytest

from rigid_body.core import orientation as ori
from rigid_body.body import shapes
from rigid_body.body import analytic_inertia as ai
from rigid_body.body.rigid_body_model import RigidBody, classify_top
from rigid_body.dynamics import state as st
from rigid_body.dynamics import torque_models as tm
from rigid_body.dynamics import integrators as ig
from rigid_body.dynamics import simulation_engine as engine
from rigid_body.analysis import analytic_solutions as anly
from rigid_body.analysis import conservation_monitor as cm


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])


def make_body(shape, density=1000.0):
    """Build a natural-orientation RigidBody from a closed-form shape.

    The same helper the dynamics tests use: the analytic inertia is
    already diagonal in the geometry frame, so the principal axes are the
    identity and no diagonalization is needed.
    """
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


def integrate_with_monitor(initial_state, body, torque_models,
                           time_step, step_count):
    """Run RK4 for ``step_count`` steps, feeding each state to a monitor.

    Returns ``(monitor, final_report, final_state)``. This mirrors the
    engine's loop (PSEUDOCODE Section 1.3): advance one substep, then
    hand the advanced state to the monitor read-only.
    """
    integrator = ig.RungeKutta4Integrator()
    monitor = cm.ConservationMonitor(initial_state, body, torque_models)
    state = initial_state
    time = 0.0
    report = None
    for _ in range(step_count):
        state = engine.advance_one_substep(
            state, time, time_step, body, torque_models, integrator)
        time = time + time_step
        report = monitor.update(state, time)
    return monitor, report, state


# --------------------------------------------------------------------
# Fed the exact analytic motion: every drift rate must vanish
# --------------------------------------------------------------------

def test_exact_steady_rotation_reports_zero_drift():
    # A body spinning steadily about a principal axis conserves energy and
    # L_space exactly, so the monitor -- fed the closed-form states -- must
    # report machine-zero drift on all three channels (Section 8.2).
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.array([0.0, 0.0, 2.5])
    initial_state = st.State(IDENTITY_QUATERNION, angular_velocity)
    monitor = cm.ConservationMonitor(initial_state, body, [])

    report = None
    for step in range(1, 41):
        time = 0.05 * step
        exact_state = anly.steady_principal_rotation(initial_state, time)
        report = monitor.update(exact_state, time)

    assert report.energy_drift_rate == pytest.approx(0.0, abs=1e-12)
    assert report.magnitude_drift_rate == pytest.approx(0.0, abs=1e-12)
    assert report.direction_drift_rate == pytest.approx(0.0, abs=1e-10)


# --------------------------------------------------------------------
# Fed a good numerical trajectory: the drift rates are small
# --------------------------------------------------------------------

def test_torque_free_numerical_run_has_small_drift():
    # RK4 on a tumbling asymmetric box conserves energy and L_space well,
    # so all three relative-per-time drift rates stay small.
    body = make_body(ASYMMETRIC_BOX)
    # A spin near the intermediate axis with a deliberate small tilt --
    # the Dzhanibekov setup, the most demanding torque-free motion.
    angular_velocity = np.array([0.02, 3.0, 0.02])
    initial_state = st.State(IDENTITY_QUATERNION, angular_velocity)

    _monitor, report, _final = integrate_with_monitor(
        initial_state, body, [], time_step=1.0e-3, step_count=2000)

    assert report.energy_drift_rate < 1.0e-4
    assert report.magnitude_drift_rate < 1.0e-4
    assert report.direction_drift_rate < 1.0e-4


# --------------------------------------------------------------------
# Balance, not constancy: the decisive test
# --------------------------------------------------------------------

def test_damping_balance_holds_though_energy_falls():
    # Under viscous damping the energy genuinely falls a long way. A
    # naive constancy check would read that fall as a huge error; the
    # monitor, checking the balance against d(energy)/dt = Gamma . omega,
    # stays tiny (DESIGN Section 7.2).
    body = make_body(shapes.Cube(0.2))
    angular_velocity = np.array([1.0, -2.0, 3.0])
    initial_state = st.State(IDENTITY_QUATERNION, angular_velocity)
    models = [tm.ViscousDamping(0.5)]

    monitor, report, final_state = integrate_with_monitor(
        initial_state, body, models, time_step=1.0e-3, step_count=2000)

    energy_initial = st.kinetic_energy(initial_state, body)
    energy_final = st.kinetic_energy(final_state, body)
    elapsed = 2.0  # 2000 steps of 1e-3 simulated seconds.

    # The physics: the body has spun down appreciably.
    assert energy_final < 0.7 * energy_initial

    # What a naive constancy monitor would report, for contrast.
    constancy_drift = (
        abs(energy_final - energy_initial)
        / monitor.energy_scale / elapsed)
    # The balance monitor's actual reading.
    assert report.energy_drift_rate < 1.0e-4
    # Balance beats constancy by orders of magnitude: the fall is physics,
    # not error, and only the balance check knows the difference.
    assert report.energy_drift_rate < 1.0e-3 * constancy_drift


def test_gravity_momentum_balance_holds_though_L_swings():
    # A heavy top: gravity torques the body, so the space-frame angular
    # momentum genuinely swings. The magnitude and direction balance
    # residuals stay tiny because the swing is exactly the predicted
    # integral of Gamma_space (DESIGN Section 7.3).
    body = make_body(shapes.Cylinder(0.3, 1.0))
    # Tip the spin axis off vertical so gravity exerts a real torque.
    tilt = ori.quaternion_from_axis_angle([1.0, 0.0, 0.0], 0.4)
    angular_velocity = np.array([0.0, 0.0, 30.0])
    initial_state = st.State(tilt, angular_velocity)
    # Lever arm from the pivot up the figure axis to the center of mass.
    models = [tm.GravityTorque([0.0, 0.0, -9.81], [0.0, 0.0, 0.25])]

    monitor, report, final_state = integrate_with_monitor(
        initial_state, body, models, time_step=5.0e-4, step_count=2000)

    momentum_initial = st.angular_momentum_space(initial_state, body)
    momentum_final = st.angular_momentum_space(final_state, body)

    # The physics: the angular-momentum vector has moved appreciably.
    swing = cm._angle_between(momentum_initial, momentum_final)
    assert swing > 0.05

    # The balance residuals nonetheless stay small.
    assert report.magnitude_drift_rate < 1.0e-3
    assert report.direction_drift_rate < 1.0e-3


# --------------------------------------------------------------------
# The monitor is read-only, and works through the engine unchanged
# --------------------------------------------------------------------

def test_update_does_not_mutate_the_state():
    body = make_body(ASYMMETRIC_BOX)
    quaternion = ori.normalize_quaternion(
        np.array([0.5, 0.5, -0.3, 0.7]))
    angular_velocity = np.array([0.3, 0.5, 2.0])
    state = st.State(quaternion.copy(), angular_velocity.copy())
    monitor = cm.ConservationMonitor(state, body, [])

    monitor.update(state, 0.01)

    np.testing.assert_array_equal(
        state.body_to_space_quaternion, quaternion)
    np.testing.assert_array_equal(
        state.angular_velocity_body, angular_velocity)


def test_monitor_does_not_change_the_trajectory():
    # Attaching the monitor to a batch run must not alter the physics:
    # the loop is read-only, so the final state is bit-for-bit the same
    # with and without it (ARCHITECTURE Section 6.4).
    body = make_body(ASYMMETRIC_BOX)
    angular_velocity = np.array([0.1, 2.0, 0.1])
    initial_state = st.State(IDENTITY_QUATERNION, angular_velocity)
    integrator = ig.RungeKutta4Integrator()

    without = engine.run_batch(
        initial_state, body, [], integrator, 1.0e-3, 0.5, [])
    monitor = cm.ConservationMonitor(initial_state, body, [])
    with_monitor = engine.run_batch(
        initial_state, body, [], integrator, 1.0e-3, 0.5, [],
        monitor=monitor)

    np.testing.assert_array_equal(
        without.body_to_space_quaternion,
        with_monitor.body_to_space_quaternion)
    np.testing.assert_array_equal(
        without.angular_velocity_body,
        with_monitor.angular_velocity_body)


# --------------------------------------------------------------------
# The Poinsot identity: a code oracle, never a live diagnostic
# --------------------------------------------------------------------

def test_poinsot_identity_is_zero_by_construction():
    # 2T = omega . L holds by definition, so the residual is machine zero
    # for any state -- even a physically arbitrary one.
    body = make_body(ASYMMETRIC_BOX)
    quaternion = ori.normalize_quaternion(
        np.array([0.2, -0.5, 0.7, 0.1]))
    angular_velocity = np.array([1.3, -0.4, 2.2])
    state = st.State(quaternion, angular_velocity)
    residual = cm.poinsot_identity_residual(state, body)
    assert residual == pytest.approx(0.0, abs=1e-12)


def test_poinsot_identity_stays_zero_on_a_wrong_trajectory():
    # The point of Section 8.4: the identity reads zero even where the
    # motion is nonsense, which is why it is useless as a drift monitor
    # and kept out of the live monitor.
    body = make_body(ASYMMETRIC_BOX)
    for _ in range(5):
        quaternion = ori.normalize_quaternion(
            np.array([0.9, -0.2, 0.3, -0.6]))
        # An arbitrary, dynamically inconsistent angular velocity.
        angular_velocity = np.array([5.0, -3.0, 8.0])
        state = st.State(quaternion, angular_velocity)
        assert cm.poinsot_identity_residual(state, body) == (
            pytest.approx(0.0, abs=1e-12))


# --------------------------------------------------------------------
# The secular-trend tracker
# --------------------------------------------------------------------

def test_trend_reports_positive_slope_for_a_growing_residual():
    # A residual that grows linearly with time is the secular signature
    # the long-integration regime must catch (DESIGN Section 7.4): the
    # fitted growth slope must be clearly positive and near the true rate.
    tracker = cm.TrendTracker()
    true_energy_rate, true_momentum_rate = 0.3, 0.7
    for step in range(1, 101):
        time = 0.1 * step
        tracker.record(
            time, true_energy_rate * time, true_momentum_rate * time)
    summary = tracker.summary()
    assert summary.energy_growth_rate == pytest.approx(
        true_energy_rate, rel=1e-6)
    assert summary.momentum_growth_rate == pytest.approx(
        true_momentum_rate, rel=1e-6)


def test_trend_reports_near_zero_slope_for_a_bounded_residual():
    # A bounded, oscillating residual -- the symplectic signature -- has
    # no secular growth, so the fitted slope is essentially zero.
    tracker = cm.TrendTracker()
    for step in range(1, 201):
        time = 0.1 * step
        bounded = 1.0e-6 * np.sin(time)
        tracker.record(time, bounded, bounded)
    summary = tracker.summary()
    assert abs(summary.energy_growth_rate) < 1.0e-7
    assert abs(summary.momentum_growth_rate) < 1.0e-7
