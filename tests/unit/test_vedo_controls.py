"""Unit tests for the live controls state machine (ui/vedo_controls.py).

The keystroke-to-state machine is pure and free of any graphics library, so
the whole control vocabulary is exercised here by feeding it key names -- no
window required.
"""

from rigid_body.dynamics.time_control import ControlMode, Pace
from rigid_body.ui.vedo_controls import (
    KeyboardControlState, AutoControlsSource)


# --------------------------------------------------------------------
# The keyboard state machine
# --------------------------------------------------------------------

def test_starts_at_normal_pace_and_open():
    state = KeyboardControlState(nominal_substeps=10)
    assert state.pace is Pace.NORMAL
    assert not state.closed


def test_space_toggles_pause_and_resume():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("space")
    assert state.pace is Pace.PAUSED
    state.handle_key("space")
    assert state.pace is Pace.NORMAL


def test_single_step_re_pauses_after_the_frame():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("s")
    assert state.pace is Pace.SINGLE_STEP
    # The one-shot: after the loop reads and steps, the pace re-pauses.
    state.advance_after_read()
    assert state.pace is Pace.PAUSED


def test_slow_fast_and_normal_keys():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("minus")
    assert state.pace is Pace.SLOW
    state.handle_key("plus")
    assert state.pace is Pace.FAST
    state.handle_key("n")
    assert state.pace is Pace.NORMAL


def test_quit_keys_close_the_window():
    quit_state = KeyboardControlState(nominal_substeps=10)
    quit_state.handle_key("q")
    assert quit_state.closed

    escape_state = KeyboardControlState(nominal_substeps=10)
    escape_state.handle_key("Escape")
    assert escape_state.closed


def test_keys_are_case_insensitive_and_unknown_keys_are_ignored():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("S")                      # capital still single-steps
    assert state.pace is Pace.SINGLE_STEP
    state.handle_key("z")                      # unknown key: no change
    assert state.pace is Pace.SINGLE_STEP


def test_snapshot_is_a_complete_live_control():
    state = KeyboardControlState(nominal_substeps=7)
    state.handle_key("minus")
    controls = state.snapshot()
    assert controls.mode is ControlMode.LIVE
    assert controls.pace is Pace.SLOW
    assert controls.nominal_substeps == 7
    assert controls.pending_edit is None


# --------------------------------------------------------------------
# The automatic (input-free) controls source
# --------------------------------------------------------------------

def test_auto_source_runs_a_fixed_number_of_frames():
    source = AutoControlsSource(nominal_substeps=5, max_frames=3)
    paces = [source.read().pace for _ in range(3)]
    assert all(pace is Pace.NORMAL for pace in paces)
    assert source.frames_read == 3
    assert source.window_closed()


def test_auto_source_reports_open_until_the_frame_count_is_reached():
    source = AutoControlsSource(nominal_substeps=5, max_frames=2)
    source.read()
    assert not source.window_closed()          # one of two read
    source.read()
    assert source.window_closed()              # both read
