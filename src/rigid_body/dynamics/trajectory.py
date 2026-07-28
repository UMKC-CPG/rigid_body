"""Trajectory retention: a bounded, read-only cache over the motion.

This is the code form of PSEUDOCODE Section 13 (DESIGN Section 12). Replay
(VISION Goal 8) and video export (Goal 11) need past states to remain
available after they were computed. One fact changes the whole problem: by
the determinism guarantee (ARCHITECTURE Section 6.4) the entire trajectory
is a pure function of the scenario, so a dropped state is never truly lost
-- it can be recomputed. Retention is therefore a **cache** whose only job
is *random access* -- reaching a past instant by table lookup instead of
re-integrating from the start -- and every policy here is free to discard,
because discarding trades memory for recomputation and never risks data.

The recomputation is exact because the state is **Markovian**: the
derivative and every torque model read only the current state and time,
never the history (Section 5.2, 6.1), so any stored state is a perfect
restart point and re-integrating forward from it reproduces the original
continuation bit-for-bit.

Two policies live here. :class:`Trajectory` is the interactive default -- a
bounded ring buffer of exact states, the most recent window at full
resolution (Section 13.4). :class:`KeyframeStore` covers a long run in
bounded memory by storing sparse exact keyframes and re-integrating forward
from the nearest earlier one to reach any instant (Section 13.5). Both are
strictly **read-only** with respect to the state: the history is written to
and read back, but no value in it ever feeds the derivative that advances
the motion (Section 13.2). That is why the buffer copies each state in and
never hands a live reference back to the loop.
"""

from typing import Optional

import numpy as np

from rigid_body.dynamics.state import State
from rigid_body.dynamics.simulation_engine import advance_one_substep


def _copy_state(state):
    """Return an independent copy of a state's seven numbers.

    Retention must isolate the stored sample from the loop's live state
    (Section 13.2): if the buffer held a reference and the loop later
    reused the array, a stored sample would silently change. Copying the
    two arrays in makes the buffer's contents its own.
    """
    return State(
        np.array(state.body_to_space_quaternion, dtype=float),
        np.array(state.angular_velocity_body, dtype=float))


class RetainedSample:
    """One retained instant: the seven-number state and its time.

    A retained sample is the state of Section 3.1 and its simulated time,
    and nothing more -- everything a replayed frame shows (energy, momentum,
    Euler angles, the Poinsot surfaces) is recomputed from that state on
    demand, exactly as in a live run, so a replayed frame is as internally
    consistent as a live one (Section 13.3).

    ``monitor_accumulators`` is the one path-dependent exception (Section
    13.3): the conservation monitor's running integrals cannot be rebuilt
    from a single state, so the loop may stash those few scalars here.
    Everything else recomputes pointwise; only the running balance carries
    history. It is stored as given and defaults to ``None``.
    """

    __slots__ = ("state", "time", "monitor_accumulators")

    def __init__(self, state, time, monitor_accumulators=None):
        self.state = state
        self.time = time
        self.monitor_accumulators = monitor_accumulators


# --------------------------------------------------------------------
# The default: a bounded ring buffer (Section 13.4)
# --------------------------------------------------------------------

class Trajectory:
    """A bounded ring buffer of exact states: the recent window at full
    resolution.

    Sized by the recorded retention limit (Section 12), it stores one
    sample per substep and, once full, overwrites the oldest as each new
    sample arrives. Per-step work is constant -- copy in and advance a
    pointer -- and memory is a fixed block proportional to the window. This
    is the right default because the interactive gestures of Goal 8 are
    overwhelmingly local in time; what falls off the far end is the deep
    past, rarely sought and, being recomputable, never lost.
    """

    def __init__(self, capacity):
        if capacity < 1:
            raise ValueError(
                f"retention capacity must be at least 1, got {capacity}")
        self.capacity = int(capacity)
        self._samples = [None] * self.capacity
        # How many samples have been appended (capped at capacity), and
        # where the next append lands in the ring.
        self.count = 0
        self._next_index = 0

    @classmethod
    def from_retention(cls, retention):
        """Build a trajectory sized by a scenario's retention limit."""
        return cls(retention.limit_samples)

    def __len__(self):
        return self.count

    @property
    def is_full(self):
        """Whether the window has filled and is now overwriting."""
        return self.count == self.capacity

    def append(self, state, time, monitor_accumulators=None):
        """Store one state at the given time, overwriting the oldest if full.

        Constant work. The state is copied in (Section 13.2), so the buffer
        never shares an array with the loop; the loop may also stash the
        monitor accumulators alongside the sample.
        """
        self._samples[self._next_index] = RetainedSample(
            _copy_state(state), float(time), monitor_accumulators)
        self._next_index = (self._next_index + 1) % self.capacity
        self.count = min(self.count + 1, self.capacity)

    def _oldest_index(self):
        """Return the ring index of the oldest retained sample."""
        return (self._next_index - self.count) % self.capacity

    def sample(self, cursor):
        """Return the :class:`RetainedSample` at a window cursor.

        Cursor ``0`` is the oldest retained sample and ``count - 1`` the
        newest. Random access by lookup, with no re-integration.
        """
        if not 0 <= cursor < self.count:
            raise IndexError(
                f"cursor {cursor} outside retained window "
                f"[0, {self.count})")
        index = (self._oldest_index() + cursor) % self.capacity
        return self._samples[index]

    def read(self, cursor):
        """Return ``(state, time)`` at a window cursor (Section 13.4).

        This is the ``history.read`` the frame loop calls for replay. The
        returned state is the buffer's own isolated copy; callers treat it
        as read-only, consistent with retention never writing back.
        """
        retained = self.sample(cursor)
        return retained.state, retained.time

    def newest(self):
        """Return ``(state, time)`` of the most recent sample, or ``None``."""
        if self.count == 0:
            return None
        return self.read(self.count - 1)

    def oldest(self):
        """Return ``(state, time)`` of the oldest retained sample, or
        ``None``."""
        if self.count == 0:
            return None
        return self.read(0)

    def time_span(self):
        """Return ``(oldest_time, newest_time)`` of the window, or ``None``.

        The range of simulated time the window currently covers, which the
        replay scrubber uses to tell an in-window request from one that
        must fall back to a keyframe re-integration (Section 15.5).
        """
        if self.count == 0:
            return None
        return self.sample(0).time, self.sample(self.count - 1).time

    def contains_time(self, time):
        """Whether ``time`` lies within the retained window's span."""
        span = self.time_span()
        if span is None:
            return False
        oldest_time, newest_time = span
        return oldest_time <= time <= newest_time


# --------------------------------------------------------------------
# Covering a long run: keyframe, then re-integrate (Section 13.5)
# --------------------------------------------------------------------

class KeyframeStore:
    """Sparse exact keyframes plus re-integration for a long run.

    A ring buffer cannot give random access to an arbitrary instant of a
    long run in bounded memory; this does, following the Markov property.
    It stores exact states at a regular substep ``stride`` and reaches any
    instant by re-running the *same* engine step forward from the nearest
    earlier keyframe. The result is exact, because re-integrating from an
    exact state reproduces the exact continuation (Section 13.1). Memory is
    bounded by the keyframe count, recomputation by the stride, both fixed.

    This is deliberately preferred over downsampling with interpolation,
    which would invent a plausible path rather than reconstruct the true
    one, and a quaternion interpolation only approximates the orientation
    (Section 13.5, VISION Principle 2).
    """

    def __init__(self, stride, time_step, body, torque_models, integrator):
        if stride < 1:
            raise ValueError(
                f"keyframe stride must be at least 1, got {stride}")
        self.stride = int(stride)
        # The pieces resolved once so any instant can be re-integrated.
        self.time_step = float(time_step)
        self.body = body
        self.torque_models = torque_models
        self.integrator = integrator
        # Keyframes as (step_index, state, time), appended in step order.
        self._keyframes = []

    @property
    def keyframe_count(self):
        return len(self._keyframes)

    def observe(self, step_index, state, time):
        """Record a keyframe when this step lands on the stride.

        The loop calls this every substep; the store keeps only the samples
        whose step index is a multiple of the stride, each copied in so it
        is a stable restart point. Observing step ``0`` (the initial state)
        guarantees a keyframe at or before every later instant.
        """
        if step_index % self.stride == 0:
            self._keyframes.append(
                (int(step_index), _copy_state(state), float(time)))

    def _nearest_earlier_keyframe(self, target_step):
        """Return the keyframe at the largest step index <= ``target_step``.

        The keyframes are appended in increasing step order, so a scan that
        keeps the last one still at or below the target finds it. Raises if
        no keyframe precedes the target (e.g. step 0 was never observed).
        """
        chosen = None
        for keyframe in self._keyframes:
            if keyframe[0] <= target_step:
                chosen = keyframe
            else:
                break
        if chosen is None:
            raise LookupError(
                f"no keyframe at or before step {target_step}; "
                "was the initial state observed?")
        return chosen

    def read_step(self, target_step):
        """Return ``(state, time)`` at an exact substep index.

        Starts from a copy of the nearest earlier keyframe and re-runs the
        same fixed step (Section 1.1) forward the remaining substeps. Exact
        by the Markov property, so this reproduces the original trajectory's
        step ``target_step`` bit-for-bit.
        """
        keyframe_step, keyframe_state, keyframe_time = (
            self._nearest_earlier_keyframe(target_step))
        state = _copy_state(keyframe_state)
        time = keyframe_time
        for _ in range(target_step - keyframe_step):
            state = advance_one_substep(
                state, time, self.time_step, self.body,
                self.torque_models, self.integrator)
            time = time + self.time_step
        return state, time

    def read_at_time(self, target_time):
        """Return ``(state, time)`` nearest a target simulated time.

        The time is resolved to the exact substep grid (rounding to the
        nearest step of the fixed ``dt``) and looked up by re-integration,
        so the deep-past read stays exact rather than interpolated.
        """
        target_step = int(round(target_time / self.time_step))
        return self.read_step(target_step)
