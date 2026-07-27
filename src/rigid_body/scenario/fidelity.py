"""The fidelity knob set shared by both execution tiers.

These are the settings that fix *how accurately* and *how far* a run is
integrated (ARCHITECTURE Section 2, PSEUDOCODE Section 12.3). They belong
to the physics zone: the bit-for-bit reproducibility guarantee attaches to
them. Unlike the authored physical quantities of the body and the initial
conditions, the fidelity values are stored as bare SI numbers directly --
a time step in seconds, a count, a span in seconds -- because that is how
PSEUDOCODE Section 12.3 records them.
"""

from dataclasses import dataclass


@dataclass
class Fidelity:
    """Integrator choice and the accuracy and extent of a run.

    ``integrator`` names the scheme (``"rk4"`` and, later, the symplectic
    schemes). ``time_step`` is the fixed ``dt`` in seconds.
    ``substeps_per_frame`` is how many fixed steps the interactive tier
    takes per rendered frame (the pacing, ARCHITECTURE Section 6.3).
    ``integration_span`` is the total simulated time a batch run covers, in
    seconds.
    """

    integrator: str
    time_step: float
    substeps_per_frame: int
    integration_span: float
