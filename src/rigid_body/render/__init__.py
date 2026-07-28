"""render: turning physical quantities into a picture.

This subpackage is the renderer boundary (ARCHITECTURE Section 5.3). It has
three layers that flow in one direction: ``scene_description`` lists *what*
to draw as renderer-agnostic plain data, ``palettes`` fixes *how* each
role is encoded, and ``vedo_renderer`` alone turns the result into pixels.
Keeping the first two free of any drawing is what lets a single scene
description drive a live window or a recording sink without either reaching
into the physics (VISION Principle 9).
"""
