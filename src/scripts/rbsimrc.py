#!/usr/bin/env python3

"""Resource-control defaults for ``rbsim.py`` (the ``XYZrc.py`` idiom).

This file holds what is machine-dependent and rarely changed for an
interactive run -- the window size and which presentation to use by default
-- and nothing that can affect the computed trajectory. Per ARCHITECTURE
Section 7, any value that influences the physics lives in the scenario file,
never here. ARCHITECTURE Section 9.3 measured the rendering budget: window
area is the scarce resource, so a window near 1280 x 960 sustains a
comfortably interactive frame rate. A user may keep a personal copy of this
file on ``$RIGID_BODY_RC`` to override the shipped defaults.
"""


def parameters_and_defaults():
    """Return the interactive script's machine-dependent default settings."""
    param_dict = {
        # Render window size in pixels. Window area dominates the frame
        # rate (ARCHITECTURE Section 9.3), so this is the first knob to
        # turn if the interactive tier feels sluggish.
        "window_width": 1280,
        "window_height": 960,
        # Which frame view to show, or "scenario" to take it from the
        # scenario's presentation zone. "side_by_side" shows the body and
        # space frames together (VISION Goal 5); "single" shows one.
        "layout": "scenario",
        # Which palette to draw with, or "scenario" to take it from the
        # scenario. One of "light", "dark", "color_blind_safe".
        "palette": "scenario"}
    return param_dict


if __name__ == '__main__':
    print(parameters_and_defaults())
