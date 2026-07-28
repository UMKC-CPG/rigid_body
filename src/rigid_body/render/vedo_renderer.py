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
                 offscreen=False, background="black"):
        self.palette = palette
        self.layout = layout
        # Which frame each panel holds still. Single-panel defaults to the
        # space-frame view -- the view from the room.
        if layout == "side_by_side":
            self.panel_frames = [Frame.BODY, Frame.SPACE]
        else:
            self.panel_frames = [Frame.SPACE]

        vedo.settings.immediate_rendering = False
        self.plotter = vedo.Plotter(
            shape=(1, len(self.panel_frames)), size=size,
            offscreen=offscreen, bg=background, axes=0)
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
            panel = self.plotter.at(panel_index)
            panel.remove(self._panel_actors[panel_index])
            panel.add(actors)
            self._panel_actors[panel_index] = actors
        # Frame the scene once, on the first render, then leave the camera
        # to the viewer (moving the camera is never a change of frame,
        # Section 11.6).
        self.plotter.render()
        if not self._camera_reset_done:
            self.plotter.reset_camera()
            self.plotter.render()
            self._camera_reset_done = True

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
            return _mesh_actors(drawable.geometry, encoding, view)
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
                reference_scale * length)
        if role in ("body_triad", "lab_triad"):
            return _triad_actors(
                drawable.geometry, encoding, view, reference_scale)
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

def _mesh_actors(shape, encoding, view):
    """Build the rigid body's mesh from its shape primitive."""
    mesh = _mesh_for_shape(shape)
    if mesh is None:
        return []
    mesh.c(encoding.color).alpha(encoding.opacity).lighting("default")
    _apply_rotation(mesh, view)
    return [mesh]


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
    """Build the momental ellipsoid as a translucent wireframe."""
    mesh = vedo.Sphere(r=1.0, res=32).scale(
        [float(axis) for axis in ellipsoid.semi_axes])
    mesh.wireframe(True).c(encoding.color).alpha(encoding.opacity)
    # Orient by the body's principal axes, then into the view frame.
    _apply_rotation(mesh, view @ np.asarray(ellipsoid.axes, dtype=float))
    return [mesh]


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


def _vector_actors(vector, encoding, view, display_length):
    """Build a shared arrow (omega or L) as a direction at a fixed length.

    The magnitude carries physical units that differ between the two
    arrows, so the arrow shows the *direction* at a display length tied to
    the scene, with the magnitude read from the telemetry overlay (Section
    14.5). ``L`` gets a second arrowhead, so it differs from ``omega`` in
    shape as well as in hue and label (Section 14.3).
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
    return actors


def _triad_actors(axes, encoding, view, reference_scale):
    """Build a coordinate triad as three arrows along its axis columns."""
    axis_matrix = np.asarray(axes, dtype=float)
    length = 1.2 * reference_scale
    actors = []
    for column in range(3):
        tip = (view @ axis_matrix[:, column]) * length
        actors.append(vedo.Arrow(
            (0.0, 0.0, 0.0), tip, c=encoding.color))
    return actors


def _telemetry_actors(readout):
    """Build the on-screen conservation readout as 2D text (Section 14.6)."""
    return [vedo.Text2D(
        _format_telemetry(readout), pos="top-left", s=0.8)]


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
