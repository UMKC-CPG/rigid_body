"""dynamics: the state, the equations of motion, and the run loop.

This subpackage advances the physics (ARCHITECTURE Section 3.3): it holds
the integrated state and its derived quantities, the torque models, the
Euler equations that turn a state into its derivative, the integrators
that step it, and the engine loop that drives the whole thing and emits
each state to a sink.
"""
