"""scenario: the reproducible unit the whole tool computes from.

A scenario is plain data with no behavior (ARCHITECTURE Section 3.6): it
holds no functions and no reference to the machine that wrote it, which is
what lets it cross the tier boundary and travel between users. Everything
that can affect the computed trajectory lives here, recorded as the
*resolved* value that was used, so that handing the file to another user
reproduces the trajectory bit for bit (VISION Goal 11).

``scenario`` defines the schema; ``serialization`` saves and restores it
as TOML. The fields split into a physics zone (body, initial conditions,
torques, fidelity, retention) and a presentation zone (viewpoint, frame,
palette); only the physics zone feeds the engine.
"""
