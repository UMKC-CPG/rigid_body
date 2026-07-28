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
    nothing to look up elsewhere.
    """

    mode: ControlMode = ControlMode.LIVE
    pace: Pace = Pace.NORMAL
    replay_cursor: Optional[int] = None
    pending_edit: Optional[dict] = None
    scale_settings: dict = field(default_factory=dict)
    nominal_substeps: int = 1


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
        nominal_substeps=scenario.fidelity.substeps_per_frame)


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
