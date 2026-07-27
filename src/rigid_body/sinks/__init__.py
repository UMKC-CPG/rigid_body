"""sinks: the consumers of a computed trajectory.

The simulation engine is the *source* of a stream of states; a **sink**
is the consumer end that absorbs that stream (ARCHITECTURE Section 5.2).
The engine emits each state to a sink without knowing which, so attaching
a different sink changes what a run *does* -- feed a renderer, write a
file, encode a video -- while the physics stays identical. That
decoupling is what lets one engine serve both the interactive and the
batch tiers.
"""
