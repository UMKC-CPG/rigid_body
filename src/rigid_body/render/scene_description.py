"""What to draw, in renderer-agnostic terms (no pixels, no vedo).

This is the code form of PSEUDOCODE Section 14.2, .5, .6 (DESIGN Section
13). Drawing passes through three hands (the renderer boundary,
ARCHITECTURE Section 5.3): ``geometry/`` and ``analysis/`` produce physical
quantities in physical coordinates; *this* module turns them into a
renderer-agnostic list of drawables -- plain data, each item carrying its
geometry, physical role, panel, coordinate frame, and label, but no color
and no VTK; and ``vedo_renderer.py`` alone realizes the list as pixels.

The middle stage is plain data on purpose. That is what lets one scene
description drive a live window or a recording sink without either reaching
into the physics -- the structural form of VISION Principle 9. The
governing rule is negative (VISION Goal 9, Principle 5): nothing is drawn
that does not stand for a named physical quantity a student can trace to
the equations, so a drawable is not a shape with a color but a quantity
with a geometry, a role, a frame, and a label that travels with it.

This module also discharges two presentation choices earlier sections
deferred here: the momental ellipsoid's scale (Section 10.2) and, through
the drawables' redundant labels, the color-blind-safe redundancy rule
(Section 14.3) that ``palettes.py`` completes.
"""

from enum import Enum, auto
from typing import NamedTuple, Optional

import numpy as np

from rigid_body.core.orientation import euler_angles_from_quaternion
from rigid_body.dynamics.state import (
    angular_momentum_space, kinetic_energy)
from rigid_body.dynamics.time_control import DISPLAY_LAYERS
from rigid_body.geometry.reference_frames import Frame
from rigid_body.geometry.poinsot import (
    momental_ellipsoid, invariable_plane, polhode, herpolhode_point,
    herpolhode_bounds)


# The number of samples the closed polhode curve is drawn with. Detail is
# cheap in the renderer (ARCHITECTURE Section 9.3, fill-rate bound, not
# geometry bound), so a smooth curve costs little.
POLHODE_SAMPLE_COUNT = 240

# The laboratory triad: the space axes X, Y, Z as the identity frame, drawn
# in the space panel (PSEUDOCODE Section 14.2).
LABORATORY_AXES = np.eye(3)


# Which display layer each drawable role belongs to (PSEUDOCODE Section
# 15.7). A viewer toggles whole layers; this map turns a layer switch into
# the set of roles to keep or drop. The telemetry overlay is deliberately
# absent -- it is always-on chrome, never part of a toggleable layer.
LAYER_OF_ROLE = {
    "body_mesh": "body",
    "momental_ellipsoid": "ellipsoid",
    "polhode": "ellipsoid",
    "invariable_plane": "ellipsoid",
    "herpolhode": "ellipsoid",
    "angular_velocity": "vectors",
    "angular_momentum": "vectors",
    "body_triad": "triads",
    "lab_triad": "triads"}

# The layers named here and the canonical list the control record carries
# must agree, or a toggle could never reach a role. Checked at import so a
# rename in one place cannot silently drift from the other.
assert set(LAYER_OF_ROLE.values()) == set(DISPLAY_LAYERS)


class DrawablePanel(Enum):
    """Which panel(s) a drawable belongs to (PSEUDOCODE Section 14.2).

    ``BODY`` and ``SPACE`` anchor a drawable to one frame's panel; ``BOTH``
    marks everything drawn in each panel and re-expressed per panel at draw
    time (Section 11.4) -- the shared arrows and the body-anchored Poinsot
    objects (the object, the momental ellipsoid, the polhode, the body
    axes), which sit still in the body view and roll in the space view;
    ``NEITHER`` is the telemetry overlay, which rides above both panels and
    belongs to no frame.

    This is distinct from :class:`Frame`: ``DrawablePanel`` says *where a
    thing is shown*, while a drawable's ``coordinate_frame`` says *what
    coordinates its geometry is expressed in*, which the renderer needs to
    re-express a shared vector into a panel (Section 11.2).
    """

    BODY = auto()
    SPACE = auto()
    BOTH = auto()
    NEITHER = auto()


class Drawable(NamedTuple):
    """One named physical quantity to draw, with everything but the pixels.

    ``geometry`` is the quantity in physical coordinates (a vector, a
    curve of points, a mesh, an ellipsoid, a plane, or a telemetry
    readout); ``role`` is the palette key naming *what it is* (Section
    14.3); ``panel`` is which panel(s) it appears in; ``coordinate_frame``
    is the frame its geometry is expressed in (``Frame.BODY`` or
    ``Frame.SPACE``, or ``None`` for a frame-free overlay), which the
    renderer uses to carry a shared vector into a panel; ``label`` is the
    text carried *with* the drawable, never added by the renderer; and
    ``scale_note`` is an optional on-screen note for a quantity scaled away
    from its physical magnitude (Section 14.5, VISION Principle 12).
    """

    geometry: object
    role: str
    panel: DrawablePanel
    coordinate_frame: Optional[Frame]
    label: str
    scale_note: Optional[str] = None


class HerpolhodeGeometry(NamedTuple):
    """The herpolhode's drawable geometry at one instant.

    The herpolhode is generally not closed and fills a band over time
    (Section 10.5), so a single frame cannot hand the renderer a finished
    curve. Instead it carries the current contact point in space -- which
    the renderer trails into the herpolhode -- together with the invariable
    plane it lies in and the inner and outer radii of the band it fills, so
    the renderer can draw the bounding annulus around the growing trace.
    """

    current_point: np.ndarray
    plane_normal: np.ndarray
    plane_distance: float
    inner_radius: float
    outer_radius: float


class TelemetryReadout(NamedTuple):
    """The numbers the on-screen conservation overlay shows (Section 14.6).

    ``report`` is the latest conservation report (the instantaneous drift
    rates of Section 8.2) or ``None`` before the first step; ``trend`` is
    the secular-growth summary; ``time_ratio`` is the simulated-to-elapsed
    time ratio ARCHITECTURE Section 6.2 insists be shown rather than
    silently corrected (it reads the wall clock for display only and never
    feeds the physics, so it does not disturb determinism); and
    ``euler_angles`` are the z-x-z angles with their degeneracy already
    flagged near ``sin(theta) = 0`` (Section 2.5).
    """

    report: object
    trend: object
    time_ratio: Optional[float]
    euler_angles: object


# --------------------------------------------------------------------
# Labeled presentation choices (Section 14.5)
# --------------------------------------------------------------------

def ellipsoid_scale_label(presentation=None):
    """Return the on-screen note stating the momental ellipsoid's scale.

    Section 10.2 left the ellipsoid's *size* open between two meaningful
    scalings. The default is the **inertia** ellipsoid
    ``I_1 x1^2 + I_2 x2^2 + I_3 x3^2 = 1``, body-fixed regardless of spin
    rate, so two runs of one body show the same ellipsoid rolling
    differently. A scenario may instead ask for the **energy** ellipsoid
    ``omega . I . omega = 2T``, the same shape resized so the tip of
    ``omega`` is the contact point. Whichever is shown is stated on screen,
    because it is a scale chosen away from a single physical value (VISION
    Principle 12).
    """
    scale = "inertia"
    if presentation is not None:
        scale = getattr(presentation, "ellipsoid_scale", "inertia")
    if scale == "energy":
        return "ellipsoid scaled to energy (omega . I . omega = 2T)"
    return "ellipsoid scaled to inertia (unit form)"


# --------------------------------------------------------------------
# The telemetry overlay (Section 14.6)
# --------------------------------------------------------------------

def telemetry_overlay(monitor, state, body, report=None,
                      time_ratio=None):
    """Assemble the conservation overlay as a frame-free drawable.

    The on-screen face of the monitor (Section 8): the drift rates, the
    simulated-to-elapsed time ratio, and the Euler angles with their
    degeneracy marked. It is a drawable like any other but belongs to no
    single frame, so it rides above both panels (``NEITHER``).
    """
    readout = TelemetryReadout(
        report=report,
        trend=monitor.residual_trend.summary(),
        time_ratio=time_ratio,
        euler_angles=euler_angles_from_quaternion(
            state.body_to_space_quaternion))
    return Drawable(
        geometry=readout,
        role="telemetry",
        panel=DrawablePanel.NEITHER,
        coordinate_frame=None,
        label="conservation monitor",
        scale_note=None)


# --------------------------------------------------------------------
# The scene inventory (Section 14.2)
# --------------------------------------------------------------------

class Scene(NamedTuple):
    """The renderer-agnostic inventory of drawables for one frame.

    ``reference_scale`` is the scene's display size -- the momental
    ellipsoid's largest semi-axis -- carried on the scene so the renderer
    sizes the vectors, triads, and object against a stable value even when
    the ellipsoid layer is toggled off and its drawable is absent (Section
    15.7). It depends only on the body's moments, so it is constant for a
    given body.
    """

    drawables: list
    reference_scale: float = 1.0


def build_scene(state, body, monitor, presentation=None, report=None,
                time_ratio=None, polhode_sample_count=POLHODE_SAMPLE_COUNT,
                visible_layers=None):
    """Assemble the scene inventory for one frame from computed quantities.

    Called by the interactive loop each frame (PSEUDOCODE Section 1.2). It
    is *torque* that decides whether the Poinsot objects appear, since the
    construction exists only for torque-free motion (Section 10): they are
    included exactly when the monitor's torque list is empty. Every
    drawable stands for a named physical quantity a student can trace to
    the equations; nothing decorative is added (VISION Principle 5).

    ``visible_layers`` is the set of display layers currently switched on
    (Section 15.7), or ``None`` to draw everything -- the default the batch
    tier and any non-toggling caller use. Layers switched off drop their
    drawables from the returned scene; the always-on telemetry overlay is
    never filtered. Filtering only chooses *what is drawn* and reads no
    state, so it cannot disturb the physics (VISION Principle 9).
    """
    drawables = []

    # The body itself, or -- for a body given by its moments with no
    # geometry to draw (Section 4, the Earth of VISION Goal 12) -- the
    # momental ellipsoid stands in as its visual proxy. Body-anchored
    # drawables appear in BOTH panels (Section 11.4): held still in the
    # body-frame view and rolling in the space-frame view, which is the
    # very motion the Poinsot ellipsoid makes visible (Section 10). Their
    # geometry stays in the body frame; the renderer re-expresses it per
    # panel with to_view, so the coordinate frame travels with each.
    if getattr(body, "geometry", None) is not None:
        drawables.append(Drawable(
            geometry=body.geometry, role="body_mesh",
            panel=DrawablePanel.BOTH, coordinate_frame=Frame.BODY,
            label="rigid body",
            scale_note="object enlarged for visibility (not to scale)"))

    ellipsoid = momental_ellipsoid(body)
    reference_scale = float(np.max(np.asarray(
        ellipsoid.semi_axes, dtype=float)))
    drawables.append(Drawable(
        geometry=ellipsoid, role="momental_ellipsoid",
        panel=DrawablePanel.BOTH, coordinate_frame=Frame.BODY,
        label="momental ellipsoid",
        scale_note=ellipsoid_scale_label(presentation)))

    # The Poinsot construction: torque-free motion only (Section 10).
    if _is_torque_free(monitor):
        polhode_curve = polhode(body, state, polhode_sample_count)
        drawables.append(Drawable(
            geometry=polhode_curve, role="polhode",
            panel=DrawablePanel.BOTH, coordinate_frame=Frame.BODY,
            label="polhode: omega in body"))
        plane = invariable_plane(state, body)
        drawables.append(Drawable(
            geometry=plane, role="invariable_plane",
            panel=DrawablePanel.SPACE, coordinate_frame=Frame.SPACE,
            label="invariable plane"))
        drawables.append(Drawable(
            geometry=_herpolhode_geometry(
                state, body, polhode_curve.points, plane),
            role="herpolhode", panel=DrawablePanel.SPACE,
            coordinate_frame=Frame.SPACE,
            label="herpolhode: omega in space"))

    # The two shared arrows, drawn in BOTH panels (Section 11.4). Each is
    # stored in its own native frame; the renderer re-expresses it into a
    # panel with to_view (Section 11.2), which is why the coordinate frame
    # travels with the drawable.
    drawables.append(Drawable(
        geometry=np.asarray(state.angular_velocity_body, dtype=float),
        role="angular_velocity", panel=DrawablePanel.BOTH,
        coordinate_frame=Frame.BODY, label="omega"))
    drawables.append(Drawable(
        geometry=angular_momentum_space(state, body),
        role="angular_momentum", panel=DrawablePanel.BOTH,
        coordinate_frame=Frame.SPACE, label="L"))

    # The two labeled triads (Section 1.1 names). Their differing labels
    # (1, 2, 3 vs X, Y, Z) are what keep the two frames distinct even under
    # a color-blind palette (Section 14.3).
    drawables.append(Drawable(
        geometry=np.asarray(body.principal_axes, dtype=float),
        role="body_triad", panel=DrawablePanel.BOTH,
        coordinate_frame=Frame.BODY, label="body axes 1, 2, 3"))
    drawables.append(Drawable(
        geometry=LABORATORY_AXES, role="lab_triad",
        panel=DrawablePanel.SPACE, coordinate_frame=Frame.SPACE,
        label="space axes X, Y, Z"))

    # The telemetry overlay belongs to NEITHER frame (Section 14.6).
    drawables.append(telemetry_overlay(
        monitor, state, body, report=report, time_ratio=time_ratio))

    return Scene(
        drawables=_visible_only(drawables, visible_layers),
        reference_scale=reference_scale)


def active_scale_notes(scene):
    """Return the scale notes of the drawables a scene currently shows.

    Each quantity drawn off its physical magnitude carries a ``scale_note``
    (Section 14.5); this gathers the notes of the drawables the scene
    actually contains, so a note appears exactly when its quantity is on
    screen and vanishes when its layer is toggled off (Section 15.7). The
    result is deduplicated in first-seen order, for the renderer to show as
    the honest on-screen footnote VISION Principle 12 requires: making a
    quantity visible off its true scale is legitimate, doing so silently is
    not.
    """
    notes = []
    for drawable in scene.drawables:
        note = getattr(drawable, "scale_note", None)
        if note and note not in notes:
            notes.append(note)
    return notes


def _visible_only(drawables, visible_layers):
    """Drop drawables whose display layer is switched off (Section 15.7).

    ``visible_layers`` is the set of layers currently on, or ``None`` to
    show everything. A drawable whose role maps to no layer -- the
    telemetry overlay -- is always-on chrome and is never filtered.
    """
    if visible_layers is None:
        return drawables
    kept = []
    for drawable in drawables:
        layer = LAYER_OF_ROLE.get(drawable.role)
        if layer is None or layer in visible_layers:
            kept.append(drawable)
    return kept


def _is_torque_free(monitor):
    """Return whether the run carries no torque model (Section 5.2).

    Reads the monitor's ordered torque list; an empty list is torque-free
    motion, the only regime in which the Poinsot construction exists.
    """
    torque_models = getattr(monitor, "torque_models", None)
    return not torque_models


def _herpolhode_geometry(state, body, polhode_points, plane):
    """Bundle the herpolhode's per-frame geometry (Section 10.5).

    The current space-frame contact point (which the renderer trails into
    the herpolhode), the invariable plane it lies in, and the inner and
    outer radii of the annular band the trace fills over time.
    """
    twice_kinetic_energy = 2.0 * kinetic_energy(state, body)
    bounds = herpolhode_bounds(
        polhode_points, twice_kinetic_energy, plane.distance)
    return HerpolhodeGeometry(
        current_point=herpolhode_point(state, body),
        plane_normal=plane.normal,
        plane_distance=plane.distance,
        inner_radius=bounds.inner_radius,
        outer_radius=bounds.outer_radius)
