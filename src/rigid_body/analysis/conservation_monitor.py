"""The live conservation monitor: physics or numerical artifact?

This is the code form of PSEUDOCODE Section 8 (DESIGN Section 7). The
monitor is the on-screen face of VISION Principle 2 -- at any moment a
student can tell whether what they see is real motion or integration
error. It watches the two quantities the state derives, the
``kinetic_energy`` and the space-frame ``angular_momentum_space``
(Section 3.2), and it recomputes them from the seven numbers the
integrator just advanced, so it never compares the state against a
private copy that could go stale (DESIGN Section 7.1).

The one decision that makes it work under applied torque is that it
checks the *balance*, not constancy. Energy and momentum are not constant
once gravity or damping is switched on, but they obey exact rate laws
either way (Section 5.5), so the monitor accumulates the change those
laws predict and compares it against the change the state actually shows.
The difference is the residual: identically zero in exact arithmetic for
any torque, and integration error alone when it departs from zero.

The monitor is strictly read-only with respect to the state -- it reports
drift, it never corrects it (Section 8.3). ``poinsot_identity_residual``
lives here too, but as a *test oracle* only: it is an algebraic identity
that reads zero even on a hopelessly wrong trajectory, so it is
deliberately not wired into the live monitor (Section 8.4).
"""

from typing import NamedTuple

import numpy as np

from rigid_body.core.orientation import rotate_body_to_space
from rigid_body.dynamics.state import (
    angular_momentum_body, angular_momentum_space, kinetic_energy)
from rigid_body.dynamics.torque_models import total_torque_body


# --------------------------------------------------------------------
# Floors that keep the reported numbers finite and interpretable
# --------------------------------------------------------------------

# A characteristic energy scale is set once at t = 0 and held (Section
# 8.2). This floor keeps the divisor away from zero for a body that is
# nearly still at the start, so the relative energy drift stays finite.
ENERGY_FLOOR = 1.0e-12

# The same protection for the characteristic angular-momentum magnitude.
MOMENTUM_FLOOR = 1.0e-12

# Drift is reported per unit simulated time (Section 8.2). Dividing by the
# elapsed time needs a floor so the very first sample, taken one step
# after t = 0, does not divide by an almost-zero elapsed time.
TIME_FLOOR = 1.0e-12


# --------------------------------------------------------------------
# The report the monitor returns each step
# --------------------------------------------------------------------

class TrendSummary(NamedTuple):
    """The secular growth of the residual across the run so far.

    The instantaneous drift rates answer the classroom question -- is
    this effect real? -- but a residual can stay small at every instant
    while growing without bound across many periods, the danger in the
    long-integration regime (DESIGN Section 7.4). These two numbers are
    the growth rate of the *raw* relative residual with simulated time,
    fit by least squares over the whole run: near zero for a bounded,
    structure-preserving error and positive for a secular one.
    """

    energy_growth_rate: float
    momentum_growth_rate: float


class ConservationReport(NamedTuple):
    """One step's worth of drift diagnostics, all read-only.

    ``energy_drift_rate`` is the energy residual relative to the fixed
    energy scale and per unit simulated time. The momentum residual is a
    vector and splits into two tells (DESIGN Section 7.3):
    ``magnitude_drift_rate`` asks whether ``|L|`` is holding, and
    ``direction_drift_rate`` asks whether the vector keeps its
    orientation in space -- the sharp one, since under torque-free motion
    that direction is the fixed invariable axis the Poinsot picture is
    built on. ``trend`` carries the secular-growth summary.
    """

    energy_drift_rate: float
    magnitude_drift_rate: float
    direction_drift_rate: float
    trend: TrendSummary


# --------------------------------------------------------------------
# Small numerical helpers
# --------------------------------------------------------------------

def _angle_between(first_vector, second_vector):
    """Return the angle in radians between two vectors, safely.

    Used for the direction drift of the angular momentum. Returns zero if
    either vector is negligibly short (no orientation to compare), and
    clips the cosine into ``[-1, 1]`` so round-off can never push the
    ``arccos`` argument out of its domain.
    """
    first_norm = float(np.linalg.norm(first_vector))
    second_norm = float(np.linalg.norm(second_vector))
    if first_norm < MOMENTUM_FLOOR or second_norm < MOMENTUM_FLOOR:
        return 0.0
    cosine = float(np.dot(first_vector, second_vector)) / (
        first_norm * second_norm)
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


class _RunningSlope:
    """An online least-squares slope of a value against time.

    Accumulates the five running sums a straight-line fit needs, so the
    slope over the entire run is available at O(1) memory and cost -- no
    history is stored. The slope of the raw relative residual against
    simulated time is what separates a bounded error (slope near zero)
    from a secular, growing one (slope clearly positive), which is the
    live counterpart of the offline long-regime growth test (DESIGN
    Section 7.4). Fewer than two distinct times give no slope, reported
    as zero.
    """

    def __init__(self):
        self.sample_count = 0
        self.sum_time = 0.0
        self.sum_value = 0.0
        self.sum_time_squared = 0.0
        self.sum_time_times_value = 0.0

    def record(self, time, value):
        """Fold one ``(time, value)`` sample into the running sums."""
        self.sample_count = self.sample_count + 1
        self.sum_time = self.sum_time + time
        self.sum_value = self.sum_value + value
        self.sum_time_squared = self.sum_time_squared + time * time
        self.sum_time_times_value = (
            self.sum_time_times_value + time * value)

    def slope(self):
        """Return the least-squares slope, or zero if undetermined.

        The denominator ``n * sum(t^2) - sum(t)^2`` vanishes when there is
        only one distinct time; in that case the trend is not yet defined
        and the slope is reported as zero.
        """
        count = self.sample_count
        denominator = (
            count * self.sum_time_squared - self.sum_time**2)
        if denominator <= 0.0:
            return 0.0
        numerator = (
            count * self.sum_time_times_value
            - self.sum_time * self.sum_value)
        return numerator / denominator


class TrendTracker:
    """Follows the secular growth of both residuals across the run.

    A thin presentation helper (its exact form is a code detail, per
    PSEUDOCODE Section 8.2): it feeds the raw *relative* residuals -- each
    already divided by its fixed scale but not by time -- to a running
    least-squares slope, one series for energy and one for momentum. The
    slopes it summarizes grow with a secular error and stay near zero for
    a bounded one.
    """

    def __init__(self):
        self._energy_slope = _RunningSlope()
        self._momentum_slope = _RunningSlope()

    def record(self, time, energy_relative, momentum_relative):
        """Record one step's raw relative residual magnitudes."""
        self._energy_slope.record(time, energy_relative)
        self._momentum_slope.record(time, momentum_relative)

    def summary(self):
        """Return the current growth rates as a ``TrendSummary``."""
        return TrendSummary(
            energy_growth_rate=self._energy_slope.slope(),
            momentum_growth_rate=self._momentum_slope.slope())


# --------------------------------------------------------------------
# The monitor itself
# --------------------------------------------------------------------

class ConservationMonitor:
    """Watches energy and angular momentum for numerical drift, live.

    Constructed from the initial state, the body, and the ordered torque
    list, it captures the initial energy and momentum and the fixed
    reference scales, then seeds the trapezoid quadrature with the rate
    laws evaluated at ``t = 0`` (PSEUDOCODE Section 8.1). Thereafter
    ``update`` is called once per step with the freshly advanced state and
    returns a ``ConservationReport``; the engine holds it read-only, so
    toggling the monitor can never change the trajectory (DESIGN Section
    6.4).
    """

    def __init__(self, initial_state, body, torque_models):
        self.body = body
        self.torque_models = torque_models

        # The two watched invariants at the start of the run (Section
        # 3.2). Everything the monitor reports is measured against these.
        self.energy_initial = kinetic_energy(initial_state, body)
        self.momentum_initial = angular_momentum_space(
            initial_state, body)

        # Fixed reference scales, set once and held (DESIGN Section 7.3).
        # A fixed scale -- not the instantaneous energy -- keeps a body
        # spinning down under damping from sending a fine residual to
        # infinity as its energy approaches zero.
        self.energy_scale = max(self.energy_initial, ENERGY_FLOOR)
        self.momentum_scale = max(
            float(np.linalg.norm(self.momentum_initial)),
            MOMENTUM_FLOOR)

        # Running accumulators for the change the rate laws predict
        # (Section 8.2), advanced one trapezoid step at a time.
        self.energy_predicted_change = 0.0
        self.momentum_predicted_change = np.zeros(3)

        # The previous sample, cached for the trapezoid quadrature. Seed
        # it with the instantaneous rate laws (Section 5.5) at t = 0.
        torque_body_initial = total_torque_body(
            0.0, initial_state, body, torque_models)
        self.last_time = 0.0
        self.last_power = float(np.dot(
            torque_body_initial,
            initial_state.angular_velocity_body))
        self.last_torque_space = rotate_body_to_space(
            initial_state.body_to_space_quaternion,
            torque_body_initial)

        # The secular-trend tracker (DESIGN Section 7.4).
        self.residual_trend = TrendTracker()

    def update(self, state, time):
        """Fold one advanced state into the diagnostics and report drift.

        Recomputes the two invariants from the state (read-only),
        advances the predicted-change accumulators by one trapezoid step
        over the rate laws, forms the residuals as observed-minus-
        predicted, and returns the normalized ``ConservationReport``. With
        ``Gamma = 0`` the accumulators stay zero and the residual reduces
        to plain drift from the initial value, so torque-free motion is
        not a special case (Section 8.2).
        """
        body = self.body

        # (DESIGN Section 7.1) Recompute the watched quantities from the
        # advanced state. Nothing here writes back into the state.
        energy = kinetic_energy(state, body)
        momentum = angular_momentum_space(state, body)

        # The instantaneous rate laws of Section 5.5: power delivered to
        # the body and the torque expressed in space components.
        torque_body = total_torque_body(
            time, state, body, self.torque_models)
        power = float(np.dot(
            torque_body, state.angular_velocity_body))
        torque_space = rotate_body_to_space(
            state.body_to_space_quaternion, torque_body)

        # (DESIGN Section 7.2) Accumulate the PREDICTED change by
        # trapezoidal quadrature over the step just taken. The quadrature
        # runs on the integrator's own steps, so it carries discretization
        # error of the same order (DESIGN Section 6.2): the residual
        # measures the mutual inconsistency of the advanced state and the
        # delivered impulse, not a ground truth the quadrature cannot
        # supply.
        step = time - self.last_time
        energy_increment = 0.5 * (self.last_power + power) * step
        momentum_increment = 0.5 * (
            self.last_torque_space + torque_space) * step
        self.energy_predicted_change = (
            self.energy_predicted_change + energy_increment)
        self.momentum_predicted_change = (
            self.momentum_predicted_change + momentum_increment)

        # (DESIGN Section 7.2) Residual = observed change - predicted
        # change. Zero in exact arithmetic for any torque.
        predicted_momentum = (
            self.momentum_initial + self.momentum_predicted_change)
        energy_residual = (
            (energy - self.energy_initial)
            - self.energy_predicted_change)
        momentum_residual = momentum - predicted_momentum

        # Cache this sample for the next trapezoid step.
        self.last_time = time
        self.last_power = power
        self.last_torque_space = torque_space

        # (DESIGN Section 7.3) Normalize: relative to the FIXED scale and
        # per unit simulated time, so the number is comparable across
        # bodies and run lengths.
        elapsed = max(time, TIME_FLOOR)
        energy_residual_relative = (
            abs(energy_residual) / self.energy_scale)
        momentum_residual_relative = (
            float(np.linalg.norm(momentum_residual))
            / self.momentum_scale)
        energy_drift_rate = energy_residual_relative / elapsed
        magnitude_drift_rate = momentum_residual_relative / elapsed

        # (DESIGN Section 7.3) The direction drift: how far the observed
        # momentum has swung from where the rate law says it should point.
        # Under torque-free motion this is the wandering of the fixed
        # invariable axis, a particularly visible non-physical tell.
        direction_drift_rate = _angle_between(
            momentum, predicted_momentum) / elapsed

        # (DESIGN Section 7.4) Feed the trend tracker the RAW relative
        # residuals (scaled but not divided by time), so a secular error
        # -- small at every instant yet growing across periods -- shows up
        # as a positive growth slope.
        self.residual_trend.record(
            time, energy_residual_relative,
            momentum_residual_relative)

        return ConservationReport(
            energy_drift_rate=energy_drift_rate,
            magnitude_drift_rate=magnitude_drift_rate,
            direction_drift_rate=direction_drift_rate,
            trend=self.residual_trend.summary())


# --------------------------------------------------------------------
# A code-consistency oracle, NOT a live diagnostic (Section 8.4)
# --------------------------------------------------------------------

def poinsot_identity_residual(state, body):
    """Return ``2T - omega . L``: an identity, zero by construction.

    The Poinsot identity ``2T = omega . L`` holds at *every* instant, with
    torque or without, because ``L = I omega`` makes ``omega . L`` equal
    to twice the kinetic energy by definition (Section 3.2). It is not a
    law the motion can violate, so it is useless as a live drift monitor:
    it reads zero even on a hopelessly wrong trajectory, and it is
    deliberately kept out of ``ConservationMonitor`` (DESIGN Section 7.6).

    It is valuable, though, as a *code-consistency* oracle for the test
    suite (ARCHITECTURE Section 8.2): a non-zero value flags a
    ``kinetic_energy`` or an ``angular_momentum_body`` computed
    inconsistently with the state.
    """
    twice_energy = 2.0 * kinetic_energy(state, body)
    return twice_energy - float(np.dot(
        state.angular_velocity_body,
        angular_momentum_body(state, body)))
