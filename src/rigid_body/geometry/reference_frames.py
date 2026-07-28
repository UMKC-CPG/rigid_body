"""One motion, two frames: expressing a quantity in body or space.

This is the code form of PSEUDOCODE Section 11 (DESIGN Section 10). VISION
Goal 5 names the body-versus-space confusion the single most common
conceptual error in the subject: reading the two coordinate descriptions
of one motion as two different motions. Section 10 built the object that
makes the distinction vivid (the Poinsot construction); this module
re-expresses one motion in either frame.

There is only ever one physical motion. The body frame and the space frame
are two coordinate systems over it, related at every instant by the single
``body_to_space`` rotation the state already carries (Section 2.2).
Re-expressing a vector is that one mapping applied one way or its inverse
-- nothing new is integrated. Like ``poinsot.py`` this works in bare-SI
physical coordinates and draws nothing, so the batch tier can label a
trajectory by frame with no renderer present.

The honesty point of the section is that a scalar formed from a vector's
components cannot change under a change of frame, so those scalars are
reported once, frame-free (``frame_invariants``), while a vector's
components mean nothing until a frame is named. The one vector whose frame
behavior is itself the lesson -- the angular momentum, fixed in space yet
sweeping through the body -- is drawn in both frames precisely so the two
accounts can be held against each other (``angular_momentum_body_curve``).
"""

import math
from enum import Enum, auto
from typing import NamedTuple

import numpy as np

from rigid_body.core.orientation import (
    rotate_body_to_space, rotate_space_to_body)
from rigid_body.dynamics.state import (
    angular_momentum_body, kinetic_energy)


class Frame(Enum):
    """Which set of axes a view holds still (DESIGN Section 10.2).

    ``SPACE`` nails the laboratory axes ``X, Y, Z`` to the screen -- the
    view from the room, where ``L`` stands still under torque-free motion.
    ``BODY`` nails the principal axes ``1, 2, 3`` -- the view from an
    observer riding the tumbling body, where the space axes and ``L``
    sweep around instead. A frame is *not* a camera angle: it is the
    choice of what is declared motionless (Section 11.6).
    """

    SPACE = auto()
    BODY = auto()


# --------------------------------------------------------------------
# One mapping, applied both ways (Section 11.1)
# --------------------------------------------------------------------

def express_in_space(state, vector_body):
    """Carry a body-frame vector into space components.

    The sandwich rotation of Section 2.2, ``v_space = body_to_space *
    v_body``. This is the *same arrow* -- only its components change.
    """
    return rotate_body_to_space(
        state.body_to_space_quaternion, vector_body)


def express_in_body(state, vector_space):
    """Carry a space-frame vector into body components.

    The inverse mapping (Section 6.3): because the state's quaternion maps
    body to space, its conjugate maps space back to body. Again one arrow,
    re-read on a different set of axes.
    """
    return rotate_space_to_body(
        state.body_to_space_quaternion, vector_space)


def to_view(state, view_frame, vector, vector_frame):
    """Express ``vector`` in the frame currently held still on screen.

    To *watch* the motion in a frame is to nail that frame's axes to the
    screen; every drawable must therefore be expressed in that view frame
    (Section 11.2). A vector already given in the view frame is drawn
    as-is; one given in the other frame is carried across by the single
    mapping of Section 11.1. The tool never stores a second copy of the
    motion -- it re-expresses the one it has.
    """
    if vector_frame is view_frame:
        return np.asarray(vector, dtype=float)
    if view_frame is Frame.SPACE:
        return express_in_space(state, vector)
    return express_in_body(state, vector)


# --------------------------------------------------------------------
# What every frame agrees on (Section 11.3)
# --------------------------------------------------------------------

class FrameInvariants(NamedTuple):
    """The scalars that read the same in every frame.

    A change of frame rotates a vector's components but cannot touch a
    scalar formed from them, so these are presented once, frame-free,
    rather than duplicated under two headings (DESIGN Section 10.3). Note
    that ``frame-free`` is not the same as ``time-constant``: under
    torque-free motion the energies and ``|L|`` are conserved, but for an
    asymmetric top ``omega_magnitude`` and ``omega_to_L_angle`` vary in
    time while still being identical in both frames at each instant.
    """

    kinetic_energy: float
    twice_energy: float
    momentum_squared: float
    momentum_magnitude: float
    omega_magnitude: float
    omega_to_L_angle: float
    principal_moments: np.ndarray


def frame_invariants(state, body):
    """Return the frame-free scalars of the current state.

    Everything here is computed from body-frame components for
    convenience, but each value is independent of that choice: a
    different frame would rotate ``omega`` and ``L`` together and leave
    every scalar below untouched. This is the same set of quantities the
    conservation monitor (Section 8) trades in, for the same reason.
    """
    angular_velocity = np.asarray(
        state.angular_velocity_body, dtype=float)
    momentum = angular_momentum_body(state, body)
    twice_energy = 2.0 * kinetic_energy(state, body)

    momentum_magnitude = float(np.linalg.norm(momentum))
    omega_magnitude = float(np.linalg.norm(angular_velocity))
    return FrameInvariants(
        kinetic_energy=twice_energy / 2.0,
        twice_energy=twice_energy,
        momentum_squared=momentum_magnitude**2,
        momentum_magnitude=momentum_magnitude,
        omega_magnitude=omega_magnitude,
        omega_to_L_angle=_angle_between(angular_velocity, momentum),
        principal_moments=np.asarray(
            body.principal_moments, dtype=float))


def _angle_between(first_vector, second_vector):
    """Return the angle in radians between two vectors, safely.

    Returns zero if either vector is negligibly short, and clips the
    cosine into ``[-1, 1]`` so round-off cannot push ``arccos`` out of its
    domain. The angle is frame-independent, which is exactly why it
    belongs among the invariants.
    """
    first_norm = float(np.linalg.norm(first_vector))
    second_norm = float(np.linalg.norm(second_vector))
    if first_norm < 1.0e-15 or second_norm < 1.0e-15:
        return 0.0
    cosine = float(np.dot(first_vector, second_vector)) / (
        first_norm * second_norm)
    return math.acos(max(-1.0, min(1.0, cosine)))


# --------------------------------------------------------------------
# The angular momentum vector in each frame (Section 11.4)
# --------------------------------------------------------------------

def angular_momentum_body_curve(polhode_points, body):
    """Return the body-frame ``L`` along the polhode: its companion curve.

    Section 3.3 flagged that under torque-free motion the space-frame
    ``L`` is constant while the body-frame ``L`` sweeps around the
    principal axes. Because ``L = I * omega``, as ``omega`` runs along the
    polhode (Section 10.4) the body-frame ``L`` runs along a companion
    curve on the sphere of radius ``|L|`` -- the same magnitude at every
    point, since ``|L|`` is conserved. One phenomenon seen through two
    closely related windows.

    Takes the polhode's body-frame ``omega`` samples, shape
    ``(count, 3)``, and returns the matching body-frame ``L`` samples.
    """
    principal_moments = np.asarray(body.principal_moments, dtype=float)
    return np.asarray(polhode_points, dtype=float) * principal_moments
