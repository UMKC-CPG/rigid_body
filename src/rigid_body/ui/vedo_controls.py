"""Live controls: turn a vedo window's keystrokes into a Controls snapshot.

This is the concrete :class:`~rigid_body.ui.interactive_session.ControlsSource`
for the interactive tier -- the widget side of PSEUDOCODE Section 15.3. The
frame loop calls ``read`` once per frame for a plain :class:`Controls`
record, ``pump`` to process the window's events, and ``window_closed`` to
learn when the viewer is done. Only the time controls are wired here (pause,
single-step, slow, fast, normal, and quit); editing the scenario through the
window -- which would build a new scenario (Section 15.4) -- is left for
later, so ``pending_edit`` stays absent in this version.

The keystroke-to-state machine, :class:`KeyboardControlState`, is kept pure
and free of any graphics library, so it is unit-tested without a window.
:class:`VedoControlsSource` is the thin binding that registers a key
callback on the renderer's plotter and pumps its event queue; it depends
only on the plotter object it is handed, never importing vedo itself.
:class:`AutoControlsSource` runs a fixed number of frames with no input, for
offscreen previews and headless tests.
"""

from rigid_body.dynamics.time_control import (
    Controls, ControlMode, Pace, pace_after_frame)


# The keys each time control answers to. Several aliases map to one action
# so the bindings feel natural whatever a backend names a key.
_PAUSE_KEYS = {"space"}
_SINGLE_STEP_KEYS = {"s"}
_SLOW_KEYS = {"minus", "underscore", "comma", ","}
_FAST_KEYS = {"plus", "equal", "period", "."}
_NORMAL_KEYS = {"n", "0"}
_CLOSE_KEYS = {"q", "escape"}


class KeyboardControlState:
    """The pure state machine behind the keyboard time controls.

    It holds the current pace, whether the window has been asked to close,
    and the scenario's nominal substep count (so a snapshot is a complete
    :class:`Controls` on its own). It is deliberately free of any window or
    graphics library, so the whole control vocabulary is exercised in tests
    by feeding it key names.
    """

    def __init__(self, nominal_substeps):
        self.nominal_substeps = nominal_substeps
        self.pace = Pace.NORMAL
        self.closed = False

    def handle_key(self, key):
        """Update the state from one pressed key (case-insensitive)."""
        pressed = (key or "").strip().lower()
        if pressed in _PAUSE_KEYS:
            # Space toggles between running and frozen.
            self.pace = (Pace.NORMAL if self.pace is Pace.PAUSED
                         else Pace.PAUSED)
        elif pressed in _SINGLE_STEP_KEYS:
            self.pace = Pace.SINGLE_STEP
        elif pressed in _SLOW_KEYS:
            self.pace = Pace.SLOW
        elif pressed in _FAST_KEYS:
            self.pace = Pace.FAST
        elif pressed in _NORMAL_KEYS:
            self.pace = Pace.NORMAL
        elif pressed in _CLOSE_KEYS:
            self.closed = True

    def snapshot(self):
        """Return the current controls as a plain :class:`Controls`."""
        return Controls(
            mode=ControlMode.LIVE, pace=self.pace,
            nominal_substeps=self.nominal_substeps)

    def advance_after_read(self):
        """Apply the once-per-frame pace transition after a read.

        A single step is a one-shot: once the loop has taken its step the
        controls re-pause, so holding ``s`` does not stream steps (Section
        15.3). Every other pace persists until the next keystroke.
        """
        self.pace = pace_after_frame(self.pace)


class VedoControlsSource:
    """Bind a :class:`KeyboardControlState` to a live vedo window.

    Takes the renderer's plotter (never importing vedo itself), registers a
    key-press callback on it, and exposes the source protocol the frame loop
    drives. ``read`` returns the current controls and then applies the
    single-step re-pause; ``pump`` processes the window's pending events so
    the callback fires without blocking the physics loop; ``window_closed``
    reports when the viewer pressed quit.
    """

    def __init__(self, plotter, nominal_substeps):
        self.plotter = plotter
        self.state = KeyboardControlState(nominal_substeps)
        self._register_key_callback()

    def _register_key_callback(self):
        """Register the key-press handler on the plotter, if it supports it."""
        try:
            self.plotter.add_callback("KeyPress", self._on_key_press)
        except Exception:                          # pragma: no cover
            # Without a callback the window still renders; it simply cannot
            # be driven by the keyboard. Better than refusing to start.
            pass

    def _on_key_press(self, event):
        """Forward a vedo key-press event to the state machine."""
        key = (getattr(event, "keypress", None)
               or getattr(event, "keyPressed", None))
        if key:
            self.state.handle_key(key)

    def read(self):
        """Return the current controls, then apply the per-frame transition."""
        controls = self.state.snapshot()
        self.state.advance_after_read()
        return controls

    def pump(self):
        """Process the window's pending events without blocking."""
        try:
            interactor = self.plotter.interactor
            if interactor is not None:
                interactor.ProcessEvents()
        except Exception:                          # pragma: no cover
            # Offscreen or context-less windows have no interactor to pump.
            pass

    def window_closed(self):
        """Whether the viewer has asked to close the window."""
        return self.state.closed


class AutoControlsSource:
    """A controls source that runs a fixed number of frames, no input.

    Used for an offscreen preview or a headless test: it holds a live
    normal pace for ``max_frames`` frames and then reports the window
    closed, so the session ends on its own with nothing to drive it.
    """

    def __init__(self, nominal_substeps, max_frames=1):
        self.nominal_substeps = nominal_substeps
        self.max_frames = max_frames
        self.frames_read = 0

    def read(self):
        self.frames_read += 1
        return Controls(
            mode=ControlMode.LIVE, pace=Pace.NORMAL,
            nominal_substeps=self.nominal_substeps)

    def pump(self):
        """No window to pump."""

    def window_closed(self):
        return self.frames_read >= self.max_frames
