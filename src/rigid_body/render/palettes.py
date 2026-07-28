"""How each drawable is encoded: role -> visual encoding (still no pixels).

This is the code form of PSEUDOCODE Section 14.3 and 14.4 (DESIGN Section
13.3, 13.4). A palette maps the *role* an item plays (the palette key the
scene description names) to the *visual encoding* it is drawn with -- color,
line style, line weight, opacity, marker. The scene names only the role;
the palette resolves it; ``vedo_renderer`` turns the encoding into pixels.
That indirection is VISION Principle 6 made concrete: because the palette is
a selectable table, switching from a light to a dark to a color-blind-safe
scheme touches neither the scene description nor the physics.

Principle 6 also imposes the rule that shaped Section 14.2:

    No distinction that carries meaning may rest on color alone where a
    label or line style can also carry it.

So every meaningful distinction is encoded **redundantly** -- in color and
in a second channel that survives the loss of color. The polhode and
herpolhode differ in hue *and* dash pattern *and* label; the two frames
differ in hue *and* their axis labels; ``omega`` and ``L`` differ in hue
*and* arrowhead *and* label. This module realizes that by keeping the
non-color channels **palette-independent**: only color changes between
schemes, while line style, marker, weight, and opacity are fixed by role,
so a color-blind palette cannot collapse a distinction the styles preserve.

Frame coding (Section 14.4) follows the same rule: everything anchored to
the body shares one hue, everything anchored to space shares another, and
the frames are also told apart by their axis labels, so the coding survives
a color-blind palette. The label itself lives on the drawable (Section
14.2), so it is always available as the final redundant channel.
"""

from typing import NamedTuple


# --------------------------------------------------------------------
# The vocabulary of non-color channels (palette-independent)
# --------------------------------------------------------------------

# Line weights, in the renderer's abstract units (thin to thick). The
# renderer maps these to pixels; here they only need to be distinct.
LINE_WEIGHT_THIN = 1.0
LINE_WEIGHT_MEDIUM = 2.5
LINE_WEIGHT_THICK = 4.0

# Line styles. A trace is told from a surface, and one trace from another,
# by its dash pattern -- a channel that survives the loss of color.
LINE_STYLE_SOLID = "solid"
LINE_STYLE_DASHED = "dashed"
LINE_STYLE_DASH_DOT = "dash_dot"
LINE_STYLE_NONE = "none"

# Markers. Arrowhead style distinguishes the two shared vectors, and the
# triads and overlay carry their own marks.
MARKER_NONE = "none"
MARKER_ARROW = "arrow"
MARKER_DOUBLE_ARROW = "double_arrow"
MARKER_AXIS = "axis"
MARKER_TEXT = "text"


# --------------------------------------------------------------------
# Which frame each role is anchored to (Section 14.4)
# --------------------------------------------------------------------

# Every role belongs to the body frame, the space frame, or neither
# (the overlay). The frame fixes the hue family; the styles below fix the
# redundant channels. These roles are exactly the ones build_scene emits
# (Section 14.2).
FRAME_FAMILY = {
    "body_mesh": "body",
    "momental_ellipsoid": "body",
    "polhode": "body",
    "angular_velocity": "body",
    "body_triad": "body",
    "invariable_plane": "space",
    "herpolhode": "space",
    "angular_momentum": "space",
    "lab_triad": "space",
    "telemetry": "neutral"}


# The palette-independent style of each role: (line_style, line_weight,
# opacity, marker). Color is supplied per palette; everything here is the
# same in every scheme, which is what guarantees the redundancy rule holds
# regardless of the color choices.
_ROLE_STYLE = {
    # The body is a translucent surface so its interior stays visible.
    "body_mesh": (LINE_STYLE_SOLID, LINE_WEIGHT_MEDIUM, 0.5, MARKER_NONE),
    # The ellipsoid is a faint wireframe-weight surface behind it.
    "momental_ellipsoid": (
        LINE_STYLE_SOLID, LINE_WEIGHT_THIN, 0.3, MARKER_NONE),
    # The two traces differ by dash pattern (and by hue, and by label).
    "polhode": (LINE_STYLE_DASHED, LINE_WEIGHT_MEDIUM, 1.0, MARKER_NONE),
    "herpolhode": (
        LINE_STYLE_DASH_DOT, LINE_WEIGHT_MEDIUM, 1.0, MARKER_NONE),
    # The invariable plane is a translucent sheet.
    "invariable_plane": (
        LINE_STYLE_SOLID, LINE_WEIGHT_THIN, 0.25, MARKER_NONE),
    # The two shared arrows differ by arrowhead (and by hue, and by label).
    "angular_velocity": (
        LINE_STYLE_SOLID, LINE_WEIGHT_THICK, 1.0, MARKER_ARROW),
    "angular_momentum": (
        LINE_STYLE_SOLID, LINE_WEIGHT_THICK, 1.0, MARKER_DOUBLE_ARROW),
    # The two triads share a mark and are told apart by hue and by their
    # axis labels (1, 2, 3 vs X, Y, Z), per Section 14.4.
    "body_triad": (
        LINE_STYLE_SOLID, LINE_WEIGHT_MEDIUM, 1.0, MARKER_AXIS),
    "lab_triad": (
        LINE_STYLE_SOLID, LINE_WEIGHT_MEDIUM, 1.0, MARKER_AXIS),
    # The overlay is text, belonging to no frame.
    "telemetry": (LINE_STYLE_NONE, LINE_WEIGHT_THIN, 1.0, MARKER_TEXT)}

# The complete set of roles a palette must encode, taken straight from the
# style table so the two cannot drift apart.
ALL_ROLES = tuple(_ROLE_STYLE)


# --------------------------------------------------------------------
# The encoding and palette records
# --------------------------------------------------------------------

class Encoding(NamedTuple):
    """The full visual encoding of one role (DESIGN Section 13.3).

    ``color`` is a hex string; ``line_style`` and ``marker`` are the shape
    channels that survive the loss of color; ``line_weight`` and
    ``opacity`` are the finer channels. The renderer turns all five into
    pixels; nothing here names vedo or VTK.
    """

    color: str
    line_style: str
    line_weight: float
    opacity: float
    marker: str


class Palette(NamedTuple):
    """A named, selectable role-to-encoding table (Principle 6)."""

    name: str
    encodings: dict


def _build_palette(name, body_color, space_color, neutral_color):
    """Assemble a palette from its three frame hues and the shared styles.

    Only the three colors vary between palettes; every role's line style,
    weight, opacity, and marker come from the palette-independent
    ``_ROLE_STYLE`` table. This is the mechanism that keeps the redundancy
    rule true in every scheme -- a color-blind palette changes the hues and
    nothing else, so no distinction it might blur in color is lost.
    """
    family_color = {
        "body": body_color, "space": space_color,
        "neutral": neutral_color}
    encodings = {}
    for role, (line_style, line_weight, opacity, marker) in (
            _ROLE_STYLE.items()):
        encodings[role] = Encoding(
            color=family_color[FRAME_FAMILY[role]],
            line_style=line_style, line_weight=line_weight,
            opacity=opacity, marker=marker)
    return Palette(name=name, encodings=encodings)


# --------------------------------------------------------------------
# The built-in palettes
# --------------------------------------------------------------------

# Light scheme: a warm body hue against a cool space hue, dark neutral.
LIGHT_PALETTE = _build_palette(
    "light", body_color="#c1440e", space_color="#1f6feb",
    neutral_color="#333333")

# Dark scheme: the same two hue families lightened for a dark background.
DARK_PALETTE = _build_palette(
    "dark", body_color="#ff9f6b", space_color="#79b8ff",
    neutral_color="#dddddd")

# Color-blind-safe scheme: the Okabe-Ito orange and blue, chosen because
# they stay distinct across the common forms of color vision deficiency;
# the redundant line styles and labels carry the rest (Section 14.3).
COLOR_BLIND_SAFE_PALETTE = _build_palette(
    "color_blind_safe", body_color="#e69f00", space_color="#0072b2",
    neutral_color="#000000")

# The registry select_palette dispatches on, by name.
PALETTES = {
    palette.name: palette for palette in (
        LIGHT_PALETTE, DARK_PALETTE, COLOR_BLIND_SAFE_PALETTE)}


# --------------------------------------------------------------------
# Lookups
# --------------------------------------------------------------------

def select_palette(palette_name):
    """Return the named palette, or raise if the name is unknown.

    The scenario's presentation zone (Section 12.2) names a palette by
    string; this resolves it. An unknown name is an error naming the
    available schemes, rather than a silent fallback that would hide a
    typo.
    """
    try:
        return PALETTES[palette_name]
    except KeyError:
        available = ", ".join(sorted(PALETTES))
        raise ValueError(
            f"unknown palette {palette_name!r}; "
            f"available palettes are: {available}")


def resolve_encoding(palette, role):
    """Return the encoding a palette gives a role (DESIGN Section 13.3).

    The scene description names only the role; this is where it becomes an
    encoding. An unknown role is an error, since every drawable a scene can
    emit must be encodable.
    """
    try:
        return palette.encodings[role]
    except KeyError:
        raise ValueError(
            f"palette {palette.name!r} has no encoding for role {role!r}")


def frame_family_of(role):
    """Return the frame family a role is anchored to (Section 14.4).

    ``"body"``, ``"space"``, or ``"neutral"`` -- the family that fixes the
    role's hue, so a viewer who learns the coding once reads it everywhere.
    """
    return FRAME_FAMILY[role]
