"""Unit tests for the live controls state machine (ui/vedo_controls.py).

The keystroke-to-state machine is pure and free of any graphics library, so
the whole control vocabulary is exercised here by feeding it key names -- no
window required.
"""

from rigid_body.dynamics.time_control import (
    ControlMode, Pace, DISPLAY_LAYERS, ALL_LAYERS_VISIBLE,
    ELLIPSOID_DETAIL_MIN, ELLIPSOID_DETAIL_MAX, ELLIPSOID_DETAIL_DEFAULT)
from rigid_body.ui.vedo_controls import (
    KeyboardControlState, AutoControlsSource, control_legend_lines)


# --------------------------------------------------------------------
# The keyboard state machine
# --------------------------------------------------------------------

def test_starts_at_normal_pace_and_open():
    state = KeyboardControlState(nominal_substeps=10)
    assert state.pace is Pace.NORMAL
    assert not state.closed


def test_space_toggles_pause_and_resume():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("Ctrl+space")
    assert state.pace is Pace.PAUSED
    state.handle_key("Ctrl+space")
    assert state.pace is Pace.NORMAL


def test_single_step_re_pauses_after_the_frame():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("Ctrl+s")
    assert state.pace is Pace.SINGLE_STEP
    # The one-shot: after the loop reads and steps, the pace re-pauses.
    state.advance_after_read()
    assert state.pace is Pace.PAUSED


def test_slow_fast_and_normal_keys():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("Ctrl+minus")
    assert state.pace is Pace.SLOW
    state.handle_key("Ctrl+plus")
    assert state.pace is Pace.FAST
    state.handle_key("Ctrl+n")
    assert state.pace is Pace.NORMAL


def test_quit_chord_and_the_bare_window_keys_all_close():
    # The quit chord closes the window.
    chord_state = KeyboardControlState(nominal_substeps=10)
    chord_state.handle_key("Ctrl+q")
    assert chord_state.closed

    # And the bare window-close keys still close it, so a window the backend
    # shuts on its own (plain q/Escape) keeps our closed flag in step.
    for bare_key in ("q", "Escape"):
        bare_state = KeyboardControlState(nominal_substeps=10)
        bare_state.handle_key(bare_key)
        assert bare_state.closed


def test_chords_are_case_insensitive_and_bare_command_keys_are_ignored():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("Ctrl+S")                 # capital chord still steps
    assert state.pace is Pace.SINGLE_STEP
    # The whole point of chording: an unchorded command key is ours no more,
    # so it neither acts here nor is stolen from the backend's own viewer
    # keys. A bare 's', '+', '-', or 'e' leaves our state untouched.
    for bare_key in ("s", "plus", "minus", "e", "z"):
        state.handle_key(bare_key)
        assert state.pace is Pace.SINGLE_STEP
        assert state.visible_layers == set(DISPLAY_LAYERS)


def test_snapshot_is_a_complete_live_control():
    state = KeyboardControlState(nominal_substeps=7)
    state.handle_key("Ctrl+minus")
    controls = state.snapshot()
    assert controls.mode is ControlMode.LIVE
    assert controls.pace is Pace.SLOW
    assert controls.nominal_substeps == 7
    assert controls.pending_edit is None
    # Untouched, every layer is visible.
    assert controls.visible_layers == ALL_LAYERS_VISIBLE


# --------------------------------------------------------------------
# The display-layer toggles (Section 15.7)
# --------------------------------------------------------------------

def test_every_layer_starts_visible():
    state = KeyboardControlState(nominal_substeps=10)
    assert state.visible_layers == set(DISPLAY_LAYERS)


def test_a_toggle_key_switches_one_layer_off_then_on():
    state = KeyboardControlState(nominal_substeps=10)
    # Ctrl+e toggles the ellipsoid layer; the others are untouched.
    state.handle_key("Ctrl+e")
    assert "ellipsoid" not in state.visible_layers
    assert {"body", "vectors", "triads"} <= state.visible_layers
    # Pressing it again brings the layer back.
    state.handle_key("Ctrl+e")
    assert "ellipsoid" in state.visible_layers


def test_each_layer_key_maps_to_its_own_group():
    for key, layer in (("Ctrl+b", "body"), ("Ctrl+e", "ellipsoid"),
                       ("Ctrl+v", "vectors"), ("Ctrl+t", "triads")):
        state = KeyboardControlState(nominal_substeps=10)
        state.handle_key(key)
        assert layer not in state.visible_layers
        # Only that one layer was removed.
        assert state.visible_layers == set(DISPLAY_LAYERS) - {layer}


def test_toggling_a_layer_is_carried_on_the_snapshot():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("Ctrl+b")                 # hide the body object
    controls = state.snapshot()
    assert "body" not in controls.visible_layers
    assert isinstance(controls.visible_layers, frozenset)


def test_a_layer_key_leaves_the_pace_alone():
    # Toggling visibility is orthogonal to the time controls.
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("Ctrl+minus")             # slow motion
    state.handle_key("Ctrl+v")                 # hide the vectors
    assert state.pace is Pace.SLOW
    assert "vectors" not in state.visible_layers


# --------------------------------------------------------------------
# The ellipsoid mesh-density control (Section 15.7)
# --------------------------------------------------------------------

def test_ellipsoid_detail_starts_at_the_default_level():
    state = KeyboardControlState(nominal_substeps=10)
    assert state.ellipsoid_detail == ELLIPSOID_DETAIL_DEFAULT


def test_finer_and_coarser_chords_step_the_detail_level():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("Ctrl+bracketright")      # finer: one level up
    assert state.ellipsoid_detail == ELLIPSOID_DETAIL_DEFAULT + 1
    state.handle_key("Ctrl+bracketleft")       # coarser: back down
    assert state.ellipsoid_detail == ELLIPSOID_DETAIL_DEFAULT


def test_detail_level_clamps_at_both_bounds():
    state = KeyboardControlState(nominal_substeps=10)
    # Press finer well past the top: it rests at the maximum, not beyond.
    for _ in range(10):
        state.handle_key("Ctrl+bracketright")
    assert state.ellipsoid_detail == ELLIPSOID_DETAIL_MAX
    # And coarser past the bottom rests at the minimum, never an empty cage.
    for _ in range(10):
        state.handle_key("Ctrl+bracketleft")
    assert state.ellipsoid_detail == ELLIPSOID_DETAIL_MIN


def test_detail_level_rides_the_snapshot():
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("Ctrl+bracketright")
    controls = state.snapshot()
    assert controls.ellipsoid_detail == ELLIPSOID_DETAIL_DEFAULT + 1


def test_a_mesh_chord_leaves_the_pace_and_layers_alone():
    # Mesh density is orthogonal to the time controls and the layer toggles.
    state = KeyboardControlState(nominal_substeps=10)
    state.handle_key("Ctrl+minus")             # slow motion
    state.handle_key("Ctrl+bracketright")      # finer cage
    assert state.pace is Pace.SLOW
    assert state.visible_layers == set(DISPLAY_LAYERS)


# --------------------------------------------------------------------
# The on-screen key legend
# --------------------------------------------------------------------

def test_the_legend_names_every_control_key():
    legend = "\n".join(control_legend_lines()).lower()
    # The time controls are all documented.
    for token in ("space", "pause", "step", "quit"):
        assert token in legend
    # The legend tells the viewer to hold Ctrl, since every command is
    # chorded (Section 15.7); without that cue the keys read as bare.
    assert "ctrl" in legend
    # And the toggles: the word "toggle" plus each layer's own name, so a
    # viewer can see what the chords switch without guessing.
    assert "toggle" in legend
    for layer_word in ("body", "ellipsoid", "vectors", "axes"):
        assert layer_word in legend
    # The mesh-density control is documented too.
    assert "mesh" in legend
    for token in ("coarser", "finer"):
        assert token in legend


# --------------------------------------------------------------------
# The automatic (input-free) controls source
# --------------------------------------------------------------------

def test_auto_source_runs_a_fixed_number_of_frames():
    source = AutoControlsSource(nominal_substeps=5, max_frames=3)
    controls = [source.read() for _ in range(3)]
    assert all(one.pace is Pace.NORMAL for one in controls)
    # The input-free source draws every layer at the default mesh density.
    assert all(one.visible_layers == ALL_LAYERS_VISIBLE for one in controls)
    assert all(
        one.ellipsoid_detail == ELLIPSOID_DETAIL_DEFAULT for one in controls)
    assert source.frames_read == 3
    assert source.window_closed()


def test_auto_source_reports_open_until_the_frame_count_is_reached():
    source = AutoControlsSource(nominal_substeps=5, max_frames=2)
    source.read()
    assert not source.window_closed()          # one of two read
    source.read()
    assert source.window_closed()              # both read
