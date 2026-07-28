"""The live sink: the engine's state stream reaches the interactive tier.

This is the sink-boundary adapter of ARCHITECTURE Section 5.2: the engine
does not know what happens to the states it produces -- it emits them to an
object satisfying ``sink_interface``. Attaching ``hdf5_sink`` yields the
batch tier; attaching this one yields the interactive tier, feeding the
renderer (ARCHITECTURE Section 3.7).

Its whole job is transport. On each emitted state it captures the most
recent ``(state, time)`` -- which the frame loop renders once per frame
(PSEUDOCODE Section 1.2), after however many substeps the time controls
chose -- and it optionally notifies a per-state hook, for a renderer or
stream that wants every step. Like every sink it is strictly **read-only**
with respect to the state: it may look but never writes a value back into
the loop, so attaching it cannot change the trajectory (the determinism
guarantee, ARCHITECTURE Section 6.4).

Two deliberate choices keep it honest and testable:

* **Decoupled from vedo.** It knows nothing of vedo, VTK, or even the scene
  description; it takes plain callables, so a fake renderer drives it in a
  test. Turning a state into a picture is the renderer's job (Section 14,
  the renderer boundary); building the scene from a state is the frame
  loop's. This sink only carries the state across the tier boundary.
* **Holds a reference, does not copy.** Unlike ``hdf5_sink``, which
  accumulates a history and must isolate each sample, the live sink keeps
  only the *latest* state and hands it straight on to be drawn. The engine
  produces a fresh ``State`` each substep and never mutates one in place, so
  the retained reference is safe and the copy ``hdf5_sink`` needs would be
  wasted work here.

Note on the design chain: ``live_sink`` is specified only at the
ARCHITECTURE level (Section 3.7, 5.2); the interactive frame loop (Section
1.2) renders inline from the loop's own ``state``. This module realizes the
sink boundary as a transport adapter -- the shape the constraints leave to
the code -- and its exact interplay with the frame-loop driver and
``vedo_renderer`` is settled when those are built.
"""

from rigid_body.sinks.sink_interface import Sink


class LiveSink(Sink):
    """Carry the engine's latest state to the interactive renderer.

    Constructed with two optional hooks and driven by the sink protocol.
    ``on_state(state, time)`` is called for every emitted state, for a
    renderer or stream that wants each step; ``on_close()`` is called once
    when the run ends. Both default to ``None``, in which case the sink is
    a pure latest-state capture the frame loop reads once per frame.
    """

    def __init__(self, on_state=None, on_close=None):
        self.on_state = on_state
        self.on_close = on_close
        # The most recent state and its time, or None before anything is
        # received. Held by reference, not copied (see the module note).
        self.latest_state = None
        self.latest_time = None
        # How many states have arrived, for the frame loop and for tests.
        self.received_count = 0
        # Whether close() has run, so a double close is harmless.
        self._closed = False

    def receive(self, state, time):
        """Capture one emitted state and notify the per-state hook.

        Read-only: it records the state and time and forwards them, never
        writing anything back into the loop (ARCHITECTURE Section 6.4).
        """
        self.latest_state = state
        self.latest_time = time
        self.received_count += 1
        if self.on_state is not None:
            self.on_state(state, time)

    def has_state(self):
        """Whether any state has been received yet."""
        return self.latest_state is not None

    def latest(self):
        """Return the most recent ``(state, time)``, or ``None`` if empty.

        This is what the frame loop draws once per frame: the endpoint of
        the substeps taken since the last render.
        """
        if self.latest_state is None:
            return None
        return self.latest_state, self.latest_time

    def close(self):
        """Notify the close hook once at the end of a run.

        Safe to call more than once; the hook fires only on the first call.
        """
        if self._closed:
            return
        self._closed = True
        if self.on_close is not None:
            self.on_close()
