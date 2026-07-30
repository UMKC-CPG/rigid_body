"""The interactive driver: the frame loop that runs the interactive tier.

This is the code form of PSEUDOCODE Section 1.2 (ARCHITECTURE Section 6.1).
The interactive tier runs its own single-threaded loop: it advances the
physics by the number of substeps the time controls choose, refreshes the
on-screen picture, and pumps the windowing system's events. The batch loop
(``run_batch``) is the same engine step with a plainer driver and no event
pump, which is exactly why the two tiers reproduce each other -- the physics
is one routine, and only what consumes its output differs.

The driver is deliberately **renderer- and controls-agnostic**. It imports
only the headless ``scene_description`` (which names no graphics library),
and takes the renderer and the controls source as objects, so the whole
loop is exercised in tests with fakes and never needs vedo or a display. A
renderer is anything with ``render(scene, state)`` and ``close()``; a
controls source is a :class:`ControlsSource` (``read``, ``pump``,
``window_closed``). The real vedo renderer and the live-window controls are
injected by the entry script.

Everything after the physics -- ``monitor.update``, ``history.append``,
``emit``, ``build_scene``, ``render`` -- only reads the state. That is the
loop-level form of the read-only discipline the monitor (DESIGN Section 7.5)
and retention (DESIGN Section 12.2) require, and it is what keeps the
determinism guarantee: turning the display, the monitor, the recording, or
replay on or off changes nothing the next ``advance_one_substep`` computes
(ARCHITECTURE Section 6.4). The relationship between the once-per-frame
render and the per-substep sink emit is pinned in DESIGN Section 13.7.
"""

from abc import ABC, abstractmethod

from rigid_body.dynamics.simulation_engine import (
    advance_one_substep, emit, finalize)
from rigid_body.dynamics.time_control import (
    ControlMode, substeps_this_frame, replay_state_at)
from rigid_body.dynamics.trajectory import Trajectory
from rigid_body.analysis.conservation_monitor import ConservationMonitor
from rigid_body.render.scene_description import build_scene
from rigid_body.scenario.serialization import (
    resolve_initial_state, build_run_components)
from rigid_body.ui.controls import apply_scenario_edit


class ControlsSource(ABC):
    """Where the frame loop reads the control surface each frame.

    The loop samples the widget state into a plain :class:`Controls` record
    (Section 15.3), pumps the windowing events, and asks whether the window
    has closed. Concrete sources are the live vedo widgets and, in tests, a
    scripted fake; the driver depends only on this small interface, never on
    the widgets themselves (Section 15.2).
    """

    @abstractmethod
    def read(self):
        """Return the current controls as a :class:`Controls` snapshot."""

    def pump(self):
        """Pump the windowing system's event queue (a no-op by default)."""

    def window_closed(self):
        """Whether the viewer has closed the window (``False`` by default)."""
        return False


def run_interactive(scenario, renderer, controls_source, sinks=None,
                    wall_clock=None, max_frames=None):
    """Run one scenario's frame loop until an edit or the window closes.

    Builds the per-run pieces afresh -- a new body, initial state, monitor,
    and retention buffer (PSEUDOCODE Section 1.2) -- then loops: read the
    controls; if a scenario edit is pending, stop and return it for the
    session driver to rebuild from (Section 15.4); otherwise either replay
    stored history or advance the live physics by the chosen substep count,
    then build the scene and draw it. Returns the pending edit, or ``None``
    when the window closes (or ``max_frames`` is reached, for tests).
    """
    sinks = sinks if sinks is not None else []
    state = resolve_initial_state(scenario)
    time = 0.0
    time_step = scenario.fidelity.time_step
    _initial, body, torque_models, integrator = (
        build_run_components(scenario))
    monitor = ConservationMonitor(state, body, torque_models)
    history = Trajectory.from_retention(scenario.retention)
    presentation = scenario.presentation

    # A run is a fresh trajectory, so the renderer's swept trails (Section
    # 10.5) must start empty rather than carry over the previous scenario's
    # trace. The renderer outlives one run; a renderer that keeps no trails
    # simply does not offer this, so the call is guarded.
    reset_trails = getattr(renderer, "reset_trails", None)
    if callable(reset_trails):
        reset_trails()

    start_wall_time = wall_clock() if wall_clock is not None else None
    latest_report = None
    frame_index = 0

    while True:
        controls = controls_source.read()

        # An edit is a NEW run, not a change to the motion in flight
        # (Section 15.4): stop and hand it to the session driver.
        if controls.pending_edit is not None:
            return controls.pending_edit

        if (controls.mode is ControlMode.REPLAY
                and controls.replay_cursor is not None):
            # Replay reads stored history; it never steps the engine, and
            # a read cannot perturb a state (Section 15.5).
            state, time = replay_state_at(
                history, None, controls.replay_cursor)
        else:
            # Live stepping. Pause yields zero substeps, slow motion fewer,
            # fast forward more (ARCHITECTURE Section 6.3).
            for _ in range(substeps_this_frame(controls)):
                state = advance_one_substep(
                    state, time, time_step, body, torque_models,
                    integrator)
                time = time + time_step
                latest_report = monitor.update(state, time)
                history.append(state, time)
                emit(sinks, state, time)

        # Draw once per frame, from the frame's endpoint state (Section
        # 13.7). The wall clock is read for the displayed time ratio only,
        # never for pacing, so it cannot disturb determinism (Section 1.4).
        time_ratio = _simulated_over_elapsed(
            time, start_wall_time, wall_clock)
        scene = build_scene(
            state, body, monitor, presentation=presentation,
            report=latest_report, time_ratio=time_ratio,
            visible_layers=controls.visible_layers,
            ellipsoid_detail=controls.ellipsoid_detail)
        renderer.render(scene, state)

        controls_source.pump()
        frame_index += 1
        if controls_source.window_closed() or (
                max_frames is not None and frame_index >= max_frames):
            return None


def run_interactive_session(scenario, renderer, controls_source,
                            sinks=None, wall_clock=None, max_frames=None):
    """The interactive entry point: run scenarios until the window closes.

    Runs one scenario to completion with :func:`run_interactive`; if that
    returns a pending edit, applies it to build the next scenario and runs
    again, so the rebuild and the setup it re-runs stay out of the frame
    loop (PSEUDOCODE Section 1.2). Any attached sinks are finalized when the
    session ends. The renderer's lifecycle belongs to the caller, which
    created it and will close it.
    """
    sinks = sinks if sinks is not None else []
    try:
        while True:
            edit = run_interactive(
                scenario, renderer, controls_source, sinks=sinks,
                wall_clock=wall_clock, max_frames=max_frames)
            if edit is None:
                return                       # window closed; session ends
            scenario = apply_scenario_edit(scenario, edit)
    finally:
        finalize(sinks)


def _simulated_over_elapsed(simulated_time, start_wall_time, wall_clock):
    """Return the simulated-to-elapsed time ratio, or ``None`` if unclocked.

    ARCHITECTURE Section 6.2 requires this ratio be shown rather than
    silently corrected. It reads the wall clock for *display only*; the
    physics is paced by substep count (Section 1.4), so showing it does not
    disturb determinism. Absent a clock (as in tests), it is simply omitted.
    """
    if wall_clock is None or start_wall_time is None:
        return None
    elapsed = wall_clock() - start_wall_time
    if elapsed <= 0.0:
        return None
    return simulated_time / elapsed
