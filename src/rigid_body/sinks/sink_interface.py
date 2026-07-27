"""The sink interface: the abstract consumer of successive states.

A sink is the consumer end of the engine's state stream (ARCHITECTURE
Sections 3.7 and 5.2). The engine calls ``receive`` once per emitted state
and ``close`` once at the end of a run; a concrete sink decides what that
means -- ``live_sink`` hands the state to the renderer, ``hdf5_sink``
writes it to disk, ``video_sink`` encodes it.

A sink is a strictly read-only consumer: it may look at a state but must
never feed a value back into the loop, because a trajectory that depended
on which sinks were attached would break the determinism guarantee
(ARCHITECTURE Section 6.4).
"""

from abc import ABC, abstractmethod


class Sink(ABC):
    """The consumer end of the engine's stream of states.

    Subclasses implement :meth:`receive`; :meth:`close` defaults to doing
    nothing, since not every sink holds a resource to release.
    """

    @abstractmethod
    def receive(self, state, time):
        """Consume one state, emitted at the given simulated time.

        Called once per substep by the engine. The sink may read the
        state but must not modify it or return anything into the loop.
        """

    def close(self):
        """Flush and release any resources at the end of a run.

        The default is to do nothing; a sink that buffers output or holds
        a file handle overrides this to flush and close it.
        """
