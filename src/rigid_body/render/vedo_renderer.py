"""Realize a scene description as pixels with vedo/VTK.

This is the code form of the renderer boundary (ARCHITECTURE Section 5.3):
the *only* module that knows about vedo or VTK. Upstream, ``geometry/`` and
``analysis/`` produce physical quantities and ``scene_description`` turns
them into a renderer-agnostic list of drawables carrying a role, a panel, a
coordinate frame, and a label (Section 14.2); ``palettes`` fixes how each
role is encoded (Section 14.3). This module takes a scene and a palette and
draws them, and nothing above it names a graphics library, so a different
backend would be a new module here and nothing else would change.

Two responsibilities are specific to drawing and live only here:

* **Frame views (Section 11.2).** Each panel holds one frame still -- the
  body-frame view nails the principal axes to the screen, the space-frame
  view nails the laboratory axes -- so every drawable is re-expressed into
  the panel's frame at draw time, reading the drawable's own
  ``coordinate_frame`` to know where its numbers start (the reason that
  field travels with each drawable).
* **Display scaling.** The momental ellipsoid, the polhode, and the contact
  point share one geometric space, but the ``omega`` and ``L`` arrows carry
  different physical units, so they are drawn as directions at a fixed
  display length tied to the ellipsoid's size, with their magnitudes read
  from the telemetry overlay. This is a labeled drawing convention (Section
  14.5, VISION Principle 12), not a change to the physics.

The renderer draws offscreen or on screen; drawing offscreen needs no X
server, which is what lets a render be captured and checked pixel by pixel
(ARCHITECTURE Section 9.3).
"""

import numpy as np
import vedo

from rigid_body.core.orientation import quaternion_to_matrix
from rigid_body.geometry.reference_frames import Frame
from rigid_body.body import shapes
from rigid_body.render.scene_description import (
    DrawablePanel, HerpolhodeGeometry, TelemetryReadout)
from rigid_body.render.palettes import (
    resolve_encoding, LINE_STYLE_DASHED, LINE_STYLE_DASH_DOT,
    MARKER_DOUBLE_ARROW)


# VTK line-stipple bit patterns, so a dashed or dash-dotted trace survives
# the loss of color (the redundancy rule, Section 14.3).
_STIPPLE_PATTERNS = {
    LINE_STYLE_DASHED: 0x00FF,
    LINE_STYLE_DASH_DOT: 0x1C47}

# The arrow display lengths, as multiples of the scene's reference size
# (the ellipsoid's largest semi-axis). omega and L differ slightly in
# length as well as in hue and arrowhead, one more redundant channel.
_OMEGA_ARROW_LENGTH = 1.25
_MOMENTUM_ARROW_LENGTH = 1.5

# The momental ellipsoid is drawn as a cage of latitude and longitude rings
# -- a "globe" wireframe -- rather than a triangulated surface, because the
# triangulation's diagonals clutter the shape and blur the 3D sense (a
# viewer's report). A few clean circles read the surface better and let the
# object nested inside it show through. These set how many rings and how
# smoothly each is sampled.
_ELLIPSOID_PARALLEL_COUNT = 5      # latitude circles between the poles
_ELLIPSOID_MERIDIAN_COUNT = 8      # longitude arcs from pole to pole
_ELLIPSOID_RING_SAMPLES = 64       # points per ring, for a smooth curve

# The body object lives in physical space (metres) while the momental
# ellipsoid lives in the angular-velocity space of 1/sqrt(I); there is no
# physical common scale between them, so the object's on-screen size is a
# presentation choice (Section 14.5). It is drawn so its largest half-extent
# is this fraction of the ellipsoid's reference size, which keeps the object
# visible inside its momental ellipsoid whatever the units. The scale is
# uniform, so the object's shape is preserved exactly -- only its overall
# size is set for display, which the scene states (the body scale_note).
_BODY_DISPLAY_FRACTION = 0.7

# On-screen 3D text: the label size as a fraction of the scene's reference
# size, and how far past an arrow tip its name is set so it clears the head.
_LABEL_TEXT_FRACTION = 0.16
_LABEL_TIP_OFFSET = 1.10

# The per-axis names the two triads carry, so each arm is labeled where it
# points (the body axes 1, 2, 3 against the space axes X, Y, Z, Section
# 14.4). Only these labels keep the frames apart under a color-blind scheme.
_TRIAD_AXIS_NAMES = {
    "body_triad": ("1", "2", "3"),
    "lab_triad": ("X", "Y", "Z")}


class VedoRenderer:
    """Draw a scene, one panel per held-still frame, with a palette.

    ``layout`` is ``"side_by_side"`` for the two-frame comparison (VISION
    Goal 5) or ``"single"`` for one frame. ``offscreen`` renders without a
    window, for capture and testing. The renderer rebuilds its actors from
    the scene each frame, which keeps the drawing a pure function of the
    current state -- the display can be toggled or reconfigured without ever
    touching the physics.
    """

    def __init__(self, palette, layout="side_by_side", size=(1280, 960),
                 offscreen=False, background=None, legend_lines=None):
        self.palette = palette
        self.layout = layout
        # An optional static key reference drawn in a corner so the live
        # controls are discoverable (Section 15.7). Plain strings supplied
        # by the caller; the renderer only draws them, and knows nothing of
        # what the keys mean, keeping the control vocabulary in ui/.
        self.legend_lines = legend_lines
        # The window color: the palette's own field unless overridden, so a
        # scheme's inks always land on the background they were chosen for
        # and never vanish into a mismatched one (a viewer's report).
        self.background = (background if background is not None
                           else getattr(palette, "background", "#101014"))
        # Which frame each panel holds still. Single-panel defaults to the
        # space-frame view -- the view from the room.
        if layout == "side_by_side":
            self.panel_frames = [Frame.BODY, Frame.SPACE]
        else:
            self.panel_frames = [Frame.SPACE]

        vedo.settings.immediate_rendering = False
        self.plotter = vedo.Plotter(
            shape=(1, len(self.panel_frames)), size=size,
            offscreen=offscreen, bg=self.background, axes=0)
        # The actors currently shown in each panel, so they can be cleared
        # before the next frame is built.
        self._panel_actors = [[] for _ in self.panel_frames]
        self._camera_reset_done = False

    # ----------------------------------------------------------------
    # The per-frame entry point
    # ----------------------------------------------------------------

    def render(self, scene, state):
        """Draw one frame of ``scene`` at ``state`` into every panel."""
        rotation = quaternion_to_matrix(state.body_to_space_quaternion)
        for panel_index, view_frame in enumerate(self.panel_frames):
            actors = self._build_panel_actors(
                scene, state, view_frame, rotation)
            actors.append(_panel_title_actor(view_frame))
            # The key reference rides in the first panel's corner only, so
            # it is shown once rather than duplicated across panels.
            if panel_index == 0 and self.legend_lines:
                actors.append(_legend_actor(self.legend_lines))
            panel = self.plotter.at(panel_index)
            panel.remove(self._panel_actors[panel_index])
            panel.add(actors)
            self._panel_actors[panel_index] = actors
        # Frame each panel to its own contents once, on the first render,
        # then leave the camera to the viewer (moving the camera is never a
        # change of frame, Section 11.6). A single global reset would frame
        # only the active panel and leave the other camera inside its
        # geometry (a viewer's report), so each panel is reset in turn.
        if not self._camera_reset_done:
            for panel_index in range(len(self.panel_frames)):
                self.plotter.at(panel_index).reset_camera()
            self._camera_reset_done = True
        self.plotter.render()

    def screenshot(self, path=None, as_array=False):
        """Capture the current frame to a PNG path or a NumPy array."""
        return self.plotter.screenshot(path, asarray=as_array)

    def close(self):
        """Release the render window."""
        self.plotter.close()

    # ----------------------------------------------------------------
    # Building the actors of one panel
    # ----------------------------------------------------------------

    def _build_panel_actors(self, scene, state, view_frame, rotation):
        """Return the vedo actors for one panel's held-still frame."""
        # The scene carries the reference size (from the ellipsoid's
        # semi-axes) so it stays fixed even when the ellipsoid layer is
        # toggled off; older scenes without it fall back to a scan.
        reference_scale = getattr(scene, "reference_scale", None)
        if not reference_scale:
            reference_scale = _reference_scale(scene)
        actors = []
        for drawable in scene.drawables:
            if not _drawable_in_panel(drawable, view_frame):
                continue
            actors.extend(self._actors_for(
                drawable, state, view_frame, rotation, reference_scale))
        return actors

    def _actors_for(self, drawable, state, view_frame, rotation,
                    reference_scale):
        """Dispatch a drawable to the actor builder for its role."""
        encoding = resolve_encoding(self.palette, drawable.role)
        view = _view_rotation(view_frame, drawable.coordinate_frame,
                              rotation)
        role = drawable.role

        if role == "body_mesh":
            return _mesh_actors(
                drawable.geometry, encoding, view, reference_scale)
        if role == "momental_ellipsoid":
            return _ellipsoid_actors(drawable.geometry, encoding, view)
        if role == "polhode":
            return _curve_actors(drawable.geometry.points, encoding, view)
        if role == "invariable_plane":
            return _plane_actors(
                drawable.geometry, encoding, view, reference_scale)
        if role == "herpolhode":
            return _herpolhode_actors(
                drawable.geometry, encoding, view, reference_scale)
        if role in ("angular_velocity", "angular_momentum"):
            length = (_OMEGA_ARROW_LENGTH if role == "angular_velocity"
                      else _MOMENTUM_ARROW_LENGTH)
            return _vector_actors(
                drawable.geometry, encoding, view,
                reference_scale * length, drawable.label, reference_scale)
        if role in ("body_triad", "lab_triad"):
            return _triad_actors(
                drawable.geometry, encoding, view, reference_scale,
                _TRIAD_AXIS_NAMES[role])
        if role == "telemetry":
            return _telemetry_actors(drawable.geometry)
        return []


# --------------------------------------------------------------------
# Panel membership and frame transforms
# --------------------------------------------------------------------

def _drawable_in_panel(drawable, view_frame):
    """Whether a drawable belongs in the panel holding ``view_frame`` still.

    Body-anchored drawables show only in the body view, space-anchored only
    in the space view, ``BOTH`` in either, and the frame-free overlay in
    every panel (it rides above them, Section 14.6).
    """
    panel = drawable.panel
    if panel is DrawablePanel.NEITHER or panel is DrawablePanel.BOTH:
        return True
    if panel is DrawablePanel.BODY:
        return view_frame is Frame.BODY
    return view_frame is Frame.SPACE


def _view_rotation(view_frame, coordinate_frame, rotation):
    """Return the 3x3 rotation that expresses a drawable in the view frame.

    A drawable's geometry is in its ``coordinate_frame``; to draw it in a
    panel that holds ``view_frame`` still, its numbers are carried across by
    the state's body-to-space rotation or its inverse (Section 11.2). When
    the two frames agree there is nothing to do.
    """
    if coordinate_frame is None or coordinate_frame is view_frame:
        return np.eye(3)
    if view_frame is Frame.SPACE:
        # Body-anchored geometry, seen in the space view: body -> space.
        return rotation
    # Space-anchored geometry, seen in the body view: space -> body.
    return rotation.T


# --------------------------------------------------------------------
# Encoding helpers
# --------------------------------------------------------------------

def _apply_line_style(actor, encoding):
    """Apply a dashed or dash-dotted stipple to a line actor, if any."""
    pattern = _STIPPLE_PATTERNS.get(encoding.line_style)
    if pattern is None:
        return actor
    try:
        vtk_property = actor.properties
        vtk_property.SetLineStipplePattern(pattern)
        vtk_property.SetLineStippleRepeatFactor(2)
    except Exception:                              # pragma: no cover
        # Stipple is a redundancy nicety; hue and label still distinguish
        # the trace if a backend cannot honor it.
        pass
    return actor


def _reference_scale(scene):
    """Return a display size for the scene: the ellipsoid's max semi-axis.

    The momental ellipsoid sets the geometric scale everything else is
    drawn against, so the vectors and triads are sized relative to it.
    """
    for drawable in scene.drawables:
        if drawable.role == "momental_ellipsoid":
            return float(np.max(drawable.geometry.semi_axes))
    return 1.0


# --------------------------------------------------------------------
# Per-role actor builders
# --------------------------------------------------------------------

def _mesh_actors(shape, encoding, view, reference_scale):
    """Build the rigid body's mesh from its shape primitive.

    The mesh is scaled to a display size beside the ellipsoid (its own
    per-layer scale, Section 14.5) so the object is visible inside its
    momental ellipsoid rather than a speck at the centre -- the two live in
    different spaces with no common physical scale (see
    ``_BODY_DISPLAY_FRACTION``). The scale is uniform, so the object's shape
    is untouched; only its overall size is chosen for display.
    """
    mesh = _mesh_for_shape(shape)
    if mesh is None:
        return []
    _scale_to_display_size(mesh, reference_scale)
    mesh.c(encoding.color).alpha(encoding.opacity).lighting("default")
    _apply_rotation(mesh, view)
    return [mesh]


def _scale_to_display_size(mesh, reference_scale):
    """Scale a mesh so its largest half-extent is a fixed display size.

    Uniformly scales the mesh (about the origin, where the body is centred)
    so its largest half-extent becomes ``_BODY_DISPLAY_FRACTION`` of the
    ellipsoid's reference size. This is the object layer's display scale
    (Section 14.5): it preserves the object's shape exactly and only sets
    how large it is drawn relative to the momental ellipsoid.
    """
    bounds = mesh.bounds()
    half_extent = 0.5 * max(
        bounds[1] - bounds[0], bounds[3] - bounds[2],
        bounds[5] - bounds[4])
    if half_extent <= 0.0:
        return
    target = _BODY_DISPLAY_FRACTION * reference_scale
    mesh.scale(target / half_extent)


def _mesh_for_shape(shape):
    """Return a vedo mesh for a shape primitive, centered at the origin.

    The parametric primitives map to vedo builders directly; a Platonic
    solid falls back to a sphere of its circumscribed size, since for a
    Platonic solid the physics is spherical and the momental ellipsoid --
    itself a sphere -- already carries it.
    """
    if isinstance(shape, shapes.Sphere):
        return vedo.Sphere(r=shape.radius, res=24)
    if isinstance(shape, shapes.Ellipsoid):
        return vedo.Sphere(r=1.0, res=24).scale(
            [shape.semi_axis_a, shape.semi_axis_b, shape.semi_axis_c])
    if isinstance(shape, shapes.Parallelepiped):
        return vedo.Box(
            length=shape.edge_a, width=shape.edge_b, height=shape.edge_c)
    if isinstance(shape, shapes.Cube):
        return vedo.Box(
            length=shape.edge, width=shape.edge, height=shape.edge)
    if isinstance(shape, shapes.Cylinder):
        return vedo.Cylinder(r=shape.radius, height=shape.height)
    if isinstance(shape, shapes.Cone):
        return vedo.Cone(r=shape.base_radius, height=shape.height)
    if isinstance(shape, (shapes.Tetrahedron, shapes.Octahedron,
                          shapes.Dodecahedron, shapes.Icosahedron)):
        return vedo.Sphere(r=0.5 * shape.edge, res=16)
    return None


def _ellipsoid_actors(ellipsoid, encoding, view):
    """Build the momental ellipsoid as a cage of latitude/longitude rings.

    Only the circular grid lines are drawn -- parallels and meridians --
    and not the diagonal edges a triangulated wireframe carries, so the
    surface reads cleanly and the object nested inside it stays visible (a
    viewer's report). Each ring is a unit-sphere circle scaled by the
    semi-axes, oriented by the body's principal axes, then carried into the
    panel's frame, so the ellipsoid rolls with the body in the space view.
    """
    semi_axes = np.asarray(ellipsoid.semi_axes, dtype=float)
    orientation = view @ np.asarray(ellipsoid.axes, dtype=float)
    actors = []
    for ring, closed in _ellipsoid_ring_points(semi_axes):
        placed = ring @ orientation.T
        line = vedo.Line(placed, closed=closed, lw=encoding.line_weight)
        line.c(encoding.color).alpha(encoding.opacity)
        actors.append(line)
    return actors


def _ellipsoid_ring_points(semi_axes):
    """Yield ``(points, closed)`` for each latitude and longitude ring.

    Points lie on the unit sphere scaled by the semi-axes, in the body's
    principal frame. Parallels are closed circles at a fixed polar angle
    (excluding the poles themselves); meridians are open arcs running from
    pole to pole at a fixed azimuth. Together they form the globe cage.
    """
    azimuth = np.linspace(0.0, 2.0 * np.pi, _ELLIPSOID_RING_SAMPLES)
    polar = np.linspace(0.0, np.pi, _ELLIPSOID_RING_SAMPLES)
    # Latitude circles (parallels), evenly spaced between the poles.
    for index in range(1, _ELLIPSOID_PARALLEL_COUNT + 1):
        angle = np.pi * index / (_ELLIPSOID_PARALLEL_COUNT + 1)
        ring = np.column_stack([
            np.sin(angle) * np.cos(azimuth),
            np.sin(angle) * np.sin(azimuth),
            np.full_like(azimuth, np.cos(angle))])
        yield ring * semi_axes, True
    # Longitude arcs (meridians), pole to pole at even azimuths.
    for index in range(_ELLIPSOID_MERIDIAN_COUNT):
        angle = 2.0 * np.pi * index / _ELLIPSOID_MERIDIAN_COUNT
        arc = np.column_stack([
            np.sin(polar) * np.cos(angle),
            np.sin(polar) * np.sin(angle),
            np.cos(polar)])
        yield arc * semi_axes, False


def _curve_actors(points, encoding, view):
    """Build a polyline (or a single marker) for a curve of body points."""
    rotated = np.asarray(points, dtype=float) @ view.T
    if rotated.shape[0] == 1:
        marker = vedo.Point(rotated[0]).c(encoding.color)
        return [marker]
    line = vedo.Line(rotated, lw=encoding.line_weight).c(encoding.color)
    line.alpha(encoding.opacity)
    return [_apply_line_style(line, encoding)]


def _plane_actors(plane, encoding, view, reference_scale):
    """Build the invariable plane as a translucent square sheet."""
    normal = view @ np.asarray(plane.normal, dtype=float)
    center = plane.distance * normal
    sheet = vedo.Plane(
        pos=center, normal=normal,
        s=(4.0 * reference_scale, 4.0 * reference_scale))
    sheet.c(encoding.color).alpha(encoding.opacity)
    return [sheet]


def _herpolhode_actors(geometry, encoding, view, reference_scale):
    """Build the herpolhode's current contact point in the invariable plane.

    The herpolhode is a trace the contact point sweeps out over time; a
    single frame carries only its current position, so this draws that
    point as a marker in the plane. Accumulating the swept trail (and
    drawing the bounding band of Section 10.5 around it) is a stateful
    renderer refinement left for later, when the frame-loop driver feeds
    successive frames.
    """
    current = view @ np.asarray(geometry.current_point, dtype=float)
    marker = vedo.Point(current).c(encoding.color)
    return [marker]


def _vector_actors(vector, encoding, view, display_length, label,
                   reference_scale):
    """Build a shared arrow (omega or L) as a direction at a fixed length.

    The magnitude carries physical units that differ between the two
    arrows, so the arrow shows the *direction* at a display length tied to
    the scene, with the magnitude read from the telemetry overlay (Section
    14.5). ``L`` gets a second arrowhead, so it differs from ``omega`` in
    shape as well as in hue and label (Section 14.3). The drawable's own
    label is drawn at the tip, so a viewer reads which arrow is which
    directly off the picture rather than from a legend.
    """
    direction = np.asarray(vector, dtype=float)
    magnitude = float(np.linalg.norm(direction))
    if magnitude < 1.0e-15:
        return []
    tip = (view @ (direction / magnitude)) * display_length

    actors = [vedo.Arrow((0.0, 0.0, 0.0), tip, c=encoding.color)]
    if encoding.marker == MARKER_DOUBLE_ARROW:
        # A second, shorter head near the base gives the double-arrow look.
        actors.append(vedo.Arrow(
            0.15 * tip, 0.55 * tip, c=encoding.color))
    actors.append(_label_actor(
        label, tip * _LABEL_TIP_OFFSET, encoding.color, reference_scale))
    return actors


def _triad_actors(axes, encoding, view, reference_scale, axis_names):
    """Build a coordinate triad as three labeled arrows along its columns.

    Each arm is drawn in the frame's hue and tagged with its axis name at
    the tip (1, 2, 3 for the body, X, Y, Z for space). Those names are what
    tell the two frames apart when color cannot (Section 14.4), so they are
    drawn on the arms, not just carried as data.
    """
    axis_matrix = np.asarray(axes, dtype=float)
    length = 1.2 * reference_scale
    actors = []
    for column in range(3):
        tip = (view @ axis_matrix[:, column]) * length
        actors.append(vedo.Arrow(
            (0.0, 0.0, 0.0), tip, c=encoding.color))
        actors.append(_label_actor(
            axis_names[column], tip * _LABEL_TIP_OFFSET,
            encoding.color, reference_scale))
    return actors


def _label_actor(text, position, color, reference_scale):
    """Build a small 3D text label at a point, in the drawable's hue.

    The label rides in the scene as a 3D object (rather than flat overlay
    text) so it sits beside the quantity it names in whichever panel the
    quantity appears, sized relative to the scene so it stays legible
    without dominating (Section 14.2, the label travels with the drawable).
    """
    size = _LABEL_TEXT_FRACTION * float(reference_scale)
    return vedo.Text3D(
        text, pos=position, s=size, c=color, justify="center")


def _telemetry_actors(readout):
    """Build the on-screen conservation readout as 2D text (Section 14.6)."""
    return [vedo.Text2D(
        _format_telemetry(readout), pos="top-left", s=0.8)]


def _panel_title_actor(view_frame):
    """Build the corner label naming which frame a panel holds still.

    The two-panel layout is a body-frame view beside a space-frame view
    (Section 11.2); naming each panel keeps the split from reading as an
    unexplained division of the window (a viewer's report). It sits in the
    top-right corner, clear of the telemetry overlay at the top-left.
    """
    name = "Body frame" if view_frame is Frame.BODY else "Space frame"
    return vedo.Text2D(name, pos="top-right", s=1.0)


def _legend_actor(lines):
    """Build the static key-reference overlay in the bottom-left corner.

    The window's own answer to "where are the keys shown?" (Section 15.7):
    a few lines of fixed text naming each control, drawn low in the first
    panel so it is always in view without crowding the telemetry readout at
    the top-left or the frame title at the top-right.
    """
    return vedo.Text2D("\n".join(lines), pos="bottom-left", s=0.75)


def _format_telemetry(readout):
    """Format the telemetry readout into a few honest lines of text."""
    lines = []
    if readout.time_ratio is not None:
        lines.append(f"sim/real time = {readout.time_ratio:.2f}")
    report = readout.report
    if report is not None:
        lines.append(f"drift(E)   = {report.energy_drift_rate:.2e}")
        lines.append(f"drift(|L|) = {report.magnitude_drift_rate:.2e}")
        lines.append(
            f"drift(dir) = {report.direction_drift_rate:.2e}")
    angles = readout.euler_angles
    if angles is not None:
        marker = " (degenerate)" if angles.is_degenerate else ""
        lines.append(
            f"theta = {angles.nutation_angle:.3f} rad{marker}")
    return "\n".join(lines) if lines else "conservation monitor"


# --------------------------------------------------------------------
# Applying a rotation to a mesh
# --------------------------------------------------------------------

def _apply_rotation(mesh, rotation):
    """Apply a 3x3 rotation to a vedo mesh via a homogeneous transform."""
    homogeneous = np.eye(4)
    homogeneous[:3, :3] = np.asarray(rotation, dtype=float)
    mesh.apply_transform(vedo.LinearTransform(homogeneous))
    return mesh
