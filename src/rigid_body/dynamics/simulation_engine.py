"""The run loop: advance the physics and emit each state to the sinks.

This is the code form of PSEUDOCODE Sections 1.1 and 1.3 (ARCHITECTURE
Section 6). The engine is the spine of the program: it advances the state
one fixed substep at a time and hands each result to whatever is watching,
arranged so that the computed trajectory depends on the scenario alone --
nothing a sink or a monitor does may feed back into a future state.

Two entry points share the *same* substep, which is exactly why the batch
run reproduces the interactive one: the physics is one routine, and only
what consumes its output differs. This module provides the atomic substep
(Section 1.1) and the batch loop (Section 1.3). The interactive loop
(Section 1.2) additionally needs the renderer, the controls, and the
monitor display, so it is assembled once ``render/`` and ``ui/`` exist.

For now the batch loop takes the already-resolved pieces of a run as
arguments; once the scenario schema exists, a thin wrapper will unpack a
scenario into this call.
"""

from rigid_body.dynamics.equations_of_motion import state_derivative


def advance_one_substep(state, time, dt, body, torque_models, integrator):
    """Advance the state by one fixed step of the integrator.

    The smallest unit of progress. Builds the pure derivative closure of
    DESIGN Section 4.2 -- closing over the fixed body and the ordered
    torque list so the integrator may evaluate it at trial points -- and
    takes one fixed step. Returns the advanced state; mutates nothing.
    """
    def derivative(evaluation_time, trial_state):
        return state_derivative(
            evaluation_time, trial_state, body, torque_models)

    return integrator.advance(state, time, dt, derivative)


def emit(sinks, state, time):
    """Fan one state out to every attached sink.

    A read-only broadcast: each sink receives the state but returns
    nothing into the loop (ARCHITECTURE Section 6.4).
    """
    for sink in sinks:
        sink.receive(state, time)


def finalize(sinks):
    """Close every sink at the end of a run, flushing any buffered output."""
    for sink in sinks:
        sink.close()


def run_batch(initial_state, body, torque_models, integrator,
              time_step, integration_span, sinks, monitor=None):
    """Run the batch tier: the same engine step, a plainer loop.

    Advances for a fixed simulated span with no renderer and no controls,
    emitting every state to the attached sinks (PSEUDOCODE Section 1.3).
    The optional ``monitor`` is updated read-only each step if given; the
    conservation monitor is a later module, so it defaults to absent.

    The step is fixed and the consumers are read-only, so two runs of the
    same inputs produce a bit-for-bit identical trajectory -- the
    determinism guarantee of ARCHITECTURE Section 6.4. Returns the final
    state.
    """
    state = initial_state
    time = 0.0
    while time < integration_span:
        state = advance_one_substep(
            state, time, time_step, body, torque_models, integrator)
        time = time + time_step
        if monitor is not None:
            monitor.update(state, time)
        emit(sinks, state, time)
    finalize(sinks)
    return state
