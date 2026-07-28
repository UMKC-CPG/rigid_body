"""ui: the surface a student touches (VISION Goals 7 and 8).

This subpackage holds the interactive controls -- the widgets and their
bindings (ARCHITECTURE Section 3.8). It sits at the top of the dependency
graph and imports downward only; nothing in the physics core imports it
(ARCHITECTURE Section 4), which is what keeps a trajectory dependent on the
scenario alone and never on which widgets were touched (Section 15.2).

One idea governs the whole layer: a control may change what the engine is
*asked* to run -- by producing a new scenario (Section 15.4) -- or what is
*shown* of a run -- through the time-control pacing of
``dynamics/time_control.py`` -- but it never reaches into a trajectory in
flight.
"""
