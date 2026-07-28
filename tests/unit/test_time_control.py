"""Unit tests for the time-control pacing (dynamics/time_control.py).

The whole promise of the time controls is that they change the *pace* of
the display and nothing about the computed motion. The tests pin that:

* Each pace maps to the right substep count, and slow motion never stalls.
* The pace ordering is FAST > NORMAL > SLOW > 0, so the controls do what
  their names say.
* A single step is a one-shot: the interface re-pauses after it.
* The controls carry no ``dt``, so pacing structurally cannot change the
  step size -- the determinism guarantee at the type level.
"""

from types import SimpleNamespace

from rigid_body.dynamics import time_control as tc


def controls_with(pace, nominal_substeps):
    """A controls snapshot at a given pace and nominal substep count."""
    return tc.Controls(pace=pace, nominal_substeps=nominal_substeps)


# --------------------------------------------------------------------
# The substep map (Section 15.3)
# --------------------------------------------------------------------

def test_each_pace_maps_to_the_expected_substep_count():
    nominal = 8
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.PAUSED, nominal)) == 0
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.SINGLE_STEP, nominal)) == 1
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.SLOW, nominal)) == nominal // tc.SLOW_FACTOR
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.NORMAL, nominal)) == nominal
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.FAST, nominal)) == nominal * tc.FAST_FACTOR


def test_slow_motion_never_stalls_below_one_step():
    # With a nominal count smaller than the slow factor, integer division
    # would give zero; the map floors it at one so slow motion still moves.
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.SLOW, 1)) == 1
    assert tc.substeps_this_frame(
        controls_with(tc.Pace.SLOW, tc.SLOW_FACTOR - 1)) == 1


def test_pace_ordering_is_monotonic():
    nominal = 12
    paused = tc.substeps_this_frame(controls_with(tc.Pace.PAUSED, nominal))
    slow = tc.substeps_this_frame(controls_with(tc.Pace.SLOW, nominal))
    normal = tc.substeps_this_frame(controls_with(tc.Pace.NORMAL, nominal))
    fast = tc.substeps_this_frame(controls_with(tc.Pace.FAST, nominal))
    assert paused < slow < normal < fast


# --------------------------------------------------------------------
# Single-step is a one-shot (Section 15.3)
# --------------------------------------------------------------------

def test_single_step_re_pauses_after_the_frame():
    assert tc.pace_after_frame(tc.Pace.SINGLE_STEP) is tc.Pace.PAUSED


def test_other_paces_persist_across_a_frame():
    for pace in (tc.Pace.PAUSED, tc.Pace.SLOW, tc.Pace.NORMAL,
                 tc.Pace.FAST):
        assert tc.pace_after_frame(pace) is pace


# --------------------------------------------------------------------
# Determinism: pacing cannot reach dt
# --------------------------------------------------------------------

def test_controls_carry_no_step_size():
    # The controls snapshot has no dt/time_step field, so a time control
    # structurally cannot change the step size -- only the substep count
    # (ARCHITECTURE Section 6.3). This is the determinism guarantee made
    # unreachable at the type level.
    controls = tc.Controls()
    assert not hasattr(controls, "dt")
    assert not hasattr(controls, "time_step")


def test_substeps_is_a_pure_function_of_the_controls():
    # Same controls, same answer, with nothing else consulted.
    controls = controls_with(tc.Pace.FAST, 5)
    first = tc.substeps_this_frame(controls)
    second = tc.substeps_this_frame(controls)
    assert first == second == 5 * tc.FAST_FACTOR


# --------------------------------------------------------------------
# default_controls (Section 15.3)
# --------------------------------------------------------------------

def test_default_controls_start_live_at_the_scenario_pace():
    # default_controls only reads the nominal substeps, so a light stub
    # with the right shape is enough to exercise it.
    scenario = SimpleNamespace(
        fidelity=SimpleNamespace(substeps_per_frame=15))
    controls = tc.default_controls(scenario)
    assert controls.mode is tc.ControlMode.LIVE
    assert controls.pace is tc.Pace.NORMAL
    assert controls.nominal_substeps == 15
    assert controls.pending_edit is None
    assert controls.replay_cursor is None
    assert controls.scale_settings == {}
    # And it paces at the nominal rate out of the box.
    assert tc.substeps_this_frame(controls) == 15
