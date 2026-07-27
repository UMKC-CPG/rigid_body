"""core: the foundation the whole physics core stands on.

This subpackage holds the two lowest-level concerns of ARCHITECTURE
Section 3.1: unit handling (``units``) and the orientation mathematics
(``orientation``). Everything else in the library rotates a vector or
reads out an angle through the routines defined here, so a name means
the same thing at every level above.

Nothing in this subpackage carries a unit object; per ARCHITECTURE
Section 5.5 units live only at the scenario boundary, and the physics
core operates on bare SI floats (DESIGN Section 2.1).
"""
