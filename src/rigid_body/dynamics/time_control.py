"""Time controls: pacing the display without touching the trajectory.

This is the code form of PSEUDOCODE Section 15.3 (DESIGN Section 14.1,
ARCHITECTURE Section 6.3). The controls a student uses to command time --
pause, single-step, slow motion, fast forward, replay -- change only the
pace and direction of *display*. None of them alters ``dt``, because
changing ``dt`` would change the trajectory; instead every time control
maps onto a **substep count**, the number of fixed steps the loop advances
per rendered frame. A student slowing a Dzhanibekov flip therefore watches
the identical flip, just fewer steps per second, not a differently
integrated one.

This module is the pure pacing core: the control record and the map from a
control snapshot to a substep count, with no widgets and no drawing. It
lives in ``dynamics/`` because the run loop consumes it and because the
determinism guarantee (ARCHITECTURE Section 6.4) rests on pacing by count,
not by clock -- a slower machine takes the same steps, just fewer per
second (Section 1.4). The widgets that *produce* a snapshot live in
``ui/controls.py``, which imports this record; nothing here imports
``ui/``, keeping the physics core free of the interface (ARCHITECTURE
Section 4).
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


# How much slow motion slows, and how much fast forward speeds up, relative
# to the scenario's nominal substeps-per-frame. Both divide or multiply the
# substep COUNT, never dt, so the trajectory is identical at any pace.
SLOW_FACTOR = 4
FAST_FACTOR = 4


# The display layers a viewer can switch on and off (Section 15.7). Each
# names a group of drawables the scene can carry: the rigid body itself,
# the momental ellipsoid and its Poinsot construction, the shared vectors
# omega and L, and the coordinate triads. A layer being off simply drops
# its drawables from the scene; nothing about the physics changes, so this
# is presentation state on the read-only control record, never a channel
# into the seven-number state (Section 15.2). The canonical names live here
# because the ``Controls`` record carries the visible set; ``render/`` maps
# each drawable role onto one of these layers.
DISPLAY_LAYERS = ("body", "ellipsoid", "vectors", "triads")

# The default: everything visible. A frozenset so it is safe to share as a
# field default without a factory (it can never be mutated in place).
ALL_LAYERS_VISIBLE = frozenset(DISPLAY_LAYERS)


class ControlMode(Enum):
    """Whether the loop is advancing the physics or replaying history.

    ``LIVE`` runs the engine forward; ``REPLAY`` reads stored states back
    out of the retention buffer (Section 15.5), never stepping the engine.
    """

    LIVE = auto()
    REPLAY = auto()


class Pace(Enum):
    """The time control in force, mapped to a substep count in 15.3.

    ``PAUSED`` freezes the motion while the display still redraws;
    ``SINGLE_STEP`` advances exactly one fixed step and then the interface
    re-pauses; ``SLOW`` and ``FAST`` scale the nominal substeps down and
    up; ``NORMAL`` uses the scenario's nominal rate.
    """

    PAUSED = auto()
    SINGLE_STEP = auto()
    SLOW = auto()
    NORMAL = auto()
    FAST = auto()


@dataclass
class Controls:
    """A plain-data snapshot of the control surface for one frame.

    This is the record ``read_controls`` (Section 15.3, ``ui/controls.py``)
    fills each frame and ``substeps_this_frame`` reads. It carries no
    channel back into the seven-number state: the interaction layer is
    read-only to the physics (Section 15.2), and its only influence on a
    trajectory is to hand the engine a new scenario through
    ``pending_edit`` (Section 15.4).

    ``nominal_substeps`` is carried on the record -- copied from
    ``scenario.fidelity.substeps_per_frame`` -- so that
    ``substeps_this_frame`` is a pure function of the controls alone, with
    nothing to look up elsewhere. ``visible_layers`` is the set of display
    layers currently switched on (Section 15.7); it filters *what is drawn*
    and touches no state, so it too rides on the read-only record.
    """

    mode: ControlMode = ControlMode.LIVE
    pace: Pace = Pace.NORMAL
    replay_cursor: Optional[int] = None
    pending_edit: Optional[dict] = None
    scale_settings: dict = field(default_factory=dict)
    nominal_substeps: int = 1
    visible_layers: frozenset = ALL_LAYERS_VISIBLE


def default_controls(scenario):
    """Return the controls a run starts from, taken from its scenario.

    A live run at the scenario's nominal pace, nothing pending, no
    replay cursor. The scenario is only read for its nominal
    substeps-per-frame, so this stays a pure function of plain data.
    """
    return Controls(
        mode=ControlMode.LIVE,
        pace=Pace.NORMAL,
        replay_cursor=None,
        pending_edit=None,
        scale_settings={},
        nominal_substeps=scenario.fidelity.substeps_per_frame,
        visible_layers=ALL_LAYERS_VISIBLE)


def substeps_this_frame(controls):
    """Map the time control onto a substep count for this frame.

    The one rule of Section 15.3: the pace changes *how many fixed steps*
    advance per rendered frame, never ``dt`` (ARCHITECTURE Section 6.3).
    ``dt`` is fixed, so the trajectory is untouched at any pace.

    * ``PAUSED`` -> 0: the motion is frozen but the display still redraws.
    * ``SINGLE_STEP`` -> 1: one step, after which the interface re-pauses
      (see :func:`pace_after_frame`).
    * ``SLOW`` -> the nominal count divided by ``SLOW_FACTOR``, but never
      below one, so slow motion still makes progress.
    * ``FAST`` -> the nominal count times ``FAST_FACTOR``.
    * ``NORMAL`` -> the scenario's nominal count.
    """
    nominal = controls.nominal_substeps
    if controls.pace is Pace.PAUSED:
        return 0
    if controls.pace is Pace.SINGLE_STEP:
        return 1
    if controls.pace is Pace.SLOW:
        return max(1, nominal // SLOW_FACTOR)
    if controls.pace is Pace.FAST:
        return nominal * FAST_FACTOR
    return nominal


def pace_after_frame(pace):
    """Return the pace the interface holds after this frame is drawn.

    A single step is a one-shot: after the loop takes its one step the
    interface re-pauses, so holding ``SINGLE_STEP`` does not stream steps
    (Section 15.3). Every other pace persists unchanged until the student
    moves the control again.
    """
    if pace is Pace.SINGLE_STEP:
        return Pace.PAUSED
    return pace


# --------------------------------------------------------------------
# The replay scrubber (Section 15.5)
# --------------------------------------------------------------------

def replay_state_at(trajectory, keyframes, target_time):
    """Return the replayed ``(state, time)`` nearest a target time.

    The replay control *reads* history rather than stepping (Section 15.5):
    it draws stored states for the recent past and, beyond the retained
    window, re-integrates from a keyframe. This composes the two retention
    policies of Section 13 into the single lookup the frame loop calls --
    the recent past from the ring buffer (Section 13.4), the deep past from
    the keyframe store (Section 13.5). Either way it is a pure read that
    cannot perturb a single computed state (Section 15.2, 13.2); replay is
    re-reading stored states, never running the engine backward, which no
    dissipative scenario could do anyway.

    ``trajectory`` is the in-memory ring buffer (or ``None`` for a
    keyframe-only session), ``keyframes`` the deep-past store (or ``None``
    when only the window is kept), and ``target_time`` the scrubber's
    position resolved to a simulated time. A time past the computed frontier
    is clamped to the newest retained state, since replay cannot run ahead
    of what the engine has produced. Raises :class:`LookupError` when the
    time falls before the window and no keyframe store can reach it.
    """
    if trajectory is not None:
        span = trajectory.time_span()
        if span is not None:
            oldest_time, newest_time = span
            if target_time >= oldest_time:
                # Within the window, or ahead of it -- clamp a future
                # target to the frontier rather than extrapolate.
                return trajectory.read_at_time(
                    min(target_time, newest_time))

    if keyframes is not None:
        return keyframes.read_at_time(target_time)

    raise LookupError(
        f"replay time {target_time} is before the retained window and "
        "no keyframe store is available to reach it")
