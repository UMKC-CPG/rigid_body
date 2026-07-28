"""The Poinsot construction: seeing torque-free motion.

This is the code form of PSEUDOCODE Section 10 (DESIGN Section 9). Poinsot
is a way of *seeing* torque-free rotation: a fixed ellipsoid attached to
the body rolls without slipping on a plane fixed in space, and the contact
point traces one curve on the ellipsoid (the polhode) and another on the
plane (the herpolhode). Every piece is built from the two invariants the
state derives -- twice the kinetic energy ``2T`` and the squared angular
momentum ``|L|^2`` (Section 3.3) -- so the whole construction exists only
for torque-free motion, where those two are conserved.

This module computes *what* the surfaces and curves are, in bare-SI
physical coordinates; it draws nothing. The rendering choices -- which
scale to draw the ellipsoid at, what colors the two frames get -- are
deferred to ``render/`` (Section 14, ARCHITECTURE Section 3.5). Keeping the
two apart is exactly what lets the batch tier write a polhode to HDF5 with
no renderer present.

The polhode is taken from the certified analytic solution of Section 9
wherever one exists (a point for a spherical top, a circle for a symmetric
top, the verified Jacobi-elliptic curve for an asymmetric top), and
``intersect_quadrics`` supplies a purely numerical fallback that intersects
the two invariant quadrics directly -- an independent second route to the
same curve, used where no analytic branch fits.
"""

import math
from typing import NamedTuple

import numpy as np

from rigid_body.core.orientation import rotate_body_to_space
from rigid_body.dynamics.state import (
    angular_momentum_space, kinetic_energy)
from rigid_body.body.rigid_body_model import (
    TopClass, MOMENT_EQUALITY_RELATIVE_TOLERANCE)
from rigid_body.analysis.analytic_solutions import (
    free_asymmetric_top_parameters, evaluate_free_asymmetric_top)


# --------------------------------------------------------------------
# The geometric objects this module returns
# --------------------------------------------------------------------

class MomentalEllipsoid(NamedTuple):
    """The body-fixed inertia ellipsoid the contact point rides on.

    ``semi_axes`` are ``1 / sqrt(I_k)`` along each principal axis -- note
    the inversion that surprises students, that the *longest* semi-axis
    lies along the axis of *smallest* moment, the direction the body most
    easily spins about (DESIGN Section 9.2). ``axes`` are the principal
    axes those semi-axes point along, so a drawing can orient the surface.
    """

    semi_axes: np.ndarray
    axes: np.ndarray


class InvariablePlane(NamedTuple):
    """The space-fixed plane the ellipsoid rolls on.

    ``normal`` is the unit outward normal, parallel to the conserved
    angular momentum -- the fixed invariable axis the conservation monitor
    watches (Section 8.2). ``distance`` is the plane's fixed distance
    ``sqrt(2T) / |L|`` from the origin. Both are constant under torque-free
    motion, so any wandering of them on screen is numerical drift, not
    physics (DESIGN Section 9.3).
    """

    normal: np.ndarray
    distance: float


class Polhode(NamedTuple):
    """The track the contact point traces on the body-fixed ellipsoid.

    ``points`` are body-frame angular-velocity samples along the closed
    curve, shape ``(sample_count, 3)`` -- or a single point for a
    spherical top, whose angular velocity never moves. ``kind`` names how
    the curve was produced (``"point"``, ``"circle"``, ``"elliptic"``, or
    ``"numeric"``), a label the tests and the renderer can read.
    """

    points: np.ndarray
    kind: str


class HerpolhodeBounds(NamedTuple):
    """The inner and outer radii of the annular band the herpolhode fills.

    The herpolhode is generally not closed -- the body circulation and the
    space precession run at an irrational rate ratio, so the contact point
    fills a band rather than retracing a loop (DESIGN Section 9.5). The
    band lies between two concentric circles in the invariable plane; these
    are their radii.
    """

    inner_radius: float
    outer_radius: float


# --------------------------------------------------------------------
# The momental ellipsoid and the contact point (Section 10.2)
# --------------------------------------------------------------------

def momental_ellipsoid(body):
    """Return the body-fixed momental ellipsoid of a body.

    The surface ``I_1 x_1^2 + I_2 x_2^2 + I_3 x_3^2 = 1``, whose semi-axis
    along principal axis ``k`` is ``1 / sqrt(I_k)``. Its shape is a
    property of the body alone, so it can be computed once at
    construction. It is also the visual proxy for a body given by its
    moments with no geometry to draw -- the Earth of VISION Goal 12.
    """
    principal_moments = np.asarray(body.principal_moments, dtype=float)
    semi_axes = 1.0 / np.sqrt(principal_moments)
    return MomentalEllipsoid(
        semi_axes=semi_axes,
        axes=np.asarray(body.principal_axes, dtype=float))


def poinsot_contact_point(state, body):
    """Return the contact point ``rho = omega / sqrt(2T)`` in body axes.

    This is the point where the ellipsoid touches the invariable plane. It
    lies on the ellipsoid, because ``rho . I . rho = (omega . I . omega)
    / (2T) = 1`` using ``omega . I . omega = 2T`` (Section 3.2). As the
    body tumbles, this point moves over the fixed ellipsoid.
    """
    twice_kinetic_energy = 2.0 * kinetic_energy(state, body)
    return state.angular_velocity_body / math.sqrt(twice_kinetic_energy)


# --------------------------------------------------------------------
# The invariable plane (Section 10.3)
# --------------------------------------------------------------------

def invariable_plane(state, body):
    """Return the space-fixed plane the ellipsoid rolls on.

    The ellipsoid's outward normal at the contact point is parallel to
    ``L`` (its gradient there is ``2 I rho ~ I omega = L``), so the plane's
    orientation is the direction of the conserved angular momentum. Its
    distance from the origin is ``sqrt(2T) / |L|``. Both are fixed under
    torque-free motion (DESIGN Section 9.3).
    """
    momentum_space = angular_momentum_space(state, body)
    momentum_magnitude = float(np.linalg.norm(momentum_space))
    twice_kinetic_energy = 2.0 * kinetic_energy(state, body)
    return InvariablePlane(
        normal=momentum_space / momentum_magnitude,
        distance=math.sqrt(twice_kinetic_energy) / momentum_magnitude)


# --------------------------------------------------------------------
# The polhode: the track on the body (Section 10.4)
# --------------------------------------------------------------------

def polhode(body, state, sample_count=240):
    """Return the polhode, the body-frame path of ``omega``.

    The polhode is the intersection of the two invariant quadrics in
    angular-velocity space, a closed curve fixed in the body. Its exact
    form is the Section 9 solution, chosen by the body's class: a single
    point for a spherical top (``omega`` is constant), a circle about the
    symmetry axis for a symmetric top, and the certified Jacobi-elliptic
    curve for an asymmetric top. The numerical ``intersect_quadrics`` is
    the fallback where no analytic branch fits.
    """
    if body.top_class is TopClass.SPHERICAL:
        # A spherical top spins with constant angular velocity, so the
        # polhode collapses to the single point where omega sits (§9.2).
        single_point = np.asarray(
            state.angular_velocity_body, dtype=float).reshape(1, 3)
        return Polhode(points=single_point, kind="point")

    if body.top_class is TopClass.SYMMETRIC:
        return Polhode(
            points=_symmetric_polhode_circle(body, state, sample_count),
            kind="circle")

    return Polhode(
        points=_asymmetric_polhode_elliptic(body, state, sample_count),
        kind="elliptic")


def _symmetric_polhode_circle(body, state, sample_count):
    """Sample the symmetric top's polhode: a circle about the figure axis.

    For a symmetric top two moments are equal; the odd one is the figure
    (symmetry) axis. The angular velocity keeps a constant component along
    that axis while its transverse pair circles at a fixed radius (Section
    9.3), so the polhode is a circle. The samples run once around it,
    starting from where ``omega`` currently sits.
    """
    principal_moments = np.asarray(body.principal_moments, dtype=float)
    figure_axis = _symmetry_axis_index(principal_moments)
    transverse_axes = [axis for axis in range(3) if axis != figure_axis]
    first_axis, second_axis = transverse_axes

    angular_velocity = np.asarray(
        state.angular_velocity_body, dtype=float)
    axial_component = angular_velocity[figure_axis]
    transverse_radius = math.hypot(
        angular_velocity[first_axis], angular_velocity[second_axis])
    starting_phase = math.atan2(
        angular_velocity[second_axis], angular_velocity[first_axis])

    phases = np.linspace(
        0.0, 2.0 * math.pi, sample_count) + starting_phase
    points = np.zeros((sample_count, 3))
    points[:, first_axis] = transverse_radius * np.cos(phases)
    points[:, second_axis] = transverse_radius * np.sin(phases)
    points[:, figure_axis] = axial_component
    return points


def _symmetry_axis_index(principal_moments):
    """Return the index of a symmetric top's distinct (figure) axis.

    Two moments match to within the moment-equality tolerance; the third
    is the figure axis. If moment 1 equals moment 2, axis 3 is distinct,
    and so on.
    """
    moment_1, moment_2, moment_3 = principal_moments
    tolerance = MOMENT_EQUALITY_RELATIVE_TOLERANCE
    if np.isclose(moment_1, moment_2, rtol=tolerance, atol=0.0):
        return 2
    if np.isclose(moment_2, moment_3, rtol=tolerance, atol=0.0):
        return 0
    return 1


def _asymmetric_polhode_elliptic(body, state, sample_count):
    """Sample the asymmetric top's polhode from the certified solution.

    The polhode is fixed by the two invariants alone, independent of where
    the state currently sits on it, so this reconstructs the canonical
    ``omega_2 = 0`` anchor that carries the state's own ``2T`` and
    ``|L|^2`` and samples ``evaluate_free_asymmetric_top`` (Section 9.4)
    over one full body period. The moments are sorted ascending for the
    solver's ``I_1 < I_2 < I_3`` convention and the sampled components are
    permuted back into the body's own axis order.
    """
    principal_moments = np.asarray(body.principal_moments, dtype=float)
    ascending_order = np.argsort(principal_moments)
    sorted_moments = principal_moments[ascending_order]
    smallest, _middle, largest = sorted_moments

    twice_kinetic_energy = 2.0 * kinetic_energy(state, body)
    momentum_body = principal_moments * np.asarray(
        state.angular_velocity_body, dtype=float)
    momentum_squared = float(np.sum(momentum_body**2))

    # The anchor where the polhode crosses omega_2 = 0. Solving the two
    # invariant equations there gives these squared outer components; they
    # reproduce the state's invariants exactly, so the reconstructed curve
    # is the very polhode the state rides (Section 10.4).
    anchor_smallest_squared = (
        (largest * twice_kinetic_energy - momentum_squared)
        / (smallest * (largest - smallest)))
    anchor_largest_squared = (
        (momentum_squared - smallest * twice_kinetic_energy)
        / (largest * (largest - smallest)))
    anchor_spin_sorted = np.array([
        math.sqrt(max(anchor_smallest_squared, 0.0)),
        0.0,
        math.sqrt(max(anchor_largest_squared, 0.0))])

    parameters = free_asymmetric_top_parameters(
        sorted_moments, anchor_spin_sorted)
    sample_times = np.linspace(
        0.0, parameters.period, sample_count)
    omega_sorted = evaluate_free_asymmetric_top(parameters, sample_times)

    # Undo the ascending sort so each column returns to its body axis.
    omega_body = np.zeros_like(omega_sorted)
    omega_body[:, ascending_order] = omega_sorted
    return omega_body


def intersect_quadrics(body, twice_kinetic_energy, momentum_squared,
                       sample_count=240):
    """Trace the polhode by intersecting the two invariant quadrics.

    A purely numerical route to the polhode, independent of the analytic
    solution, used where no analytic branch fits and valuable as a second
    oracle where one does. The polhode is where these two surfaces meet in
    angular-velocity space::

        energy   quadric:  I_1 w1^2   + I_2 w2^2   + I_3 w3^2   = 2T
        momentum quadric:  I_1^2 w1^2 + I_2^2 w2^2 + I_3^2 w3^2 = |L|^2

    Sweeping the middle-axis squared component and solving the resulting
    2x2 linear system for the outer two squared components gives points
    that satisfy *both* quadrics exactly; four sign octants tile the full
    closed loop. The middle axis and one outer axis cross zero (they carry
    the loop), while the remaining outer axis keeps a fixed sign.
    """
    principal_moments = np.asarray(body.principal_moments, dtype=float)
    ascending_order = np.argsort(principal_moments)
    smallest, middle, largest = principal_moments[ascending_order]

    # The middle-axis squared component ``u_middle`` is the sweep
    # parameter. Solving the two quadrics for the outer squared components
    # gives them as linear functions of ``u_middle`` (Cramer's rule on the
    # 2x2 system in the smallest/largest moments).
    def outer_components(u_middle):
        energy_remainder = twice_kinetic_energy - middle * u_middle
        momentum_remainder = (
            momentum_squared - middle * middle * u_middle)
        u_smallest = (
            (largest * energy_remainder - momentum_remainder)
            / (smallest * (largest - smallest)))
        u_largest = (
            (momentum_remainder - smallest * energy_remainder)
            / (largest * (largest - smallest)))
        return u_smallest, u_largest

    # The sweep runs from omega_middle = 0 up to where the first outer
    # component reaches zero; that outer axis is the loop's other
    # zero-crossing partner, the remaining one keeps a fixed sign.
    u_middle_at_smallest_zero = (
        (largest * twice_kinetic_energy - momentum_squared)
        / (middle * (largest - middle)))
    u_middle_at_largest_zero = (
        (momentum_squared - smallest * twice_kinetic_energy)
        / (middle * (middle - smallest)))
    if u_middle_at_smallest_zero <= u_middle_at_largest_zero:
        u_middle_max = u_middle_at_smallest_zero
        crossing_outer, fixed_outer = 0, 2   # smallest crosses zero
    else:
        u_middle_max = u_middle_at_largest_zero
        crossing_outer, fixed_outer = 2, 0   # largest crosses zero

    # One quarter arc, from omega_middle = 0 to the crossing extreme. The
    # sweep runs over the middle-axis *amplitude* rather than its square,
    # so the points stay evenly spaced through the zero-crossing instead
    # of bunching away from it as sqrt(u_middle) would.
    quarter_count = max(sample_count // 4, 2)
    w_middle_grid = np.linspace(
        0.0, math.sqrt(max(u_middle_max, 0.0)), quarter_count)
    quarter = np.zeros((quarter_count, 3))
    for row, w_middle in enumerate(w_middle_grid):
        u_smallest, u_largest = outer_components(w_middle**2)
        quarter[row, 0] = math.sqrt(max(u_smallest, 0.0))
        quarter[row, 1] = w_middle
        quarter[row, 2] = math.sqrt(max(u_largest, 0.0))

    loop_sorted = _tile_polhode_quarter(
        quarter, middle_axis=1,
        crossing_outer=crossing_outer, fixed_outer=fixed_outer)

    # Map the sorted-axis loop back into the body's own axis order.
    loop_body = np.zeros_like(loop_sorted)
    loop_body[:, ascending_order] = loop_sorted
    return loop_body


def _tile_polhode_quarter(quarter, middle_axis, crossing_outer,
                          fixed_outer):
    """Tile one quarter arc into the full closed polhode by sign symmetry.

    ``quarter`` holds non-negative component magnitudes along one arc, from
    the middle-axis extreme to the crossing-outer extreme. The middle axis
    and the crossing-outer axis both flip sign around the loop; the fixed
    outer axis keeps its (positive) sign throughout. Four signed, ordered
    segments join into one continuous closed curve.
    """
    middle = quarter[:, middle_axis]
    crossing = quarter[:, crossing_outer]
    fixed = quarter[:, fixed_outer]
    reversed_slice = slice(None, None, -1)

    def segment(middle_sign, crossing_sign, forward):
        order = slice(None) if forward else reversed_slice
        piece = np.zeros((quarter.shape[0], 3))
        piece[:, middle_axis] = middle_sign * middle[order]
        piece[:, crossing_outer] = crossing_sign * crossing[order]
        piece[:, fixed_outer] = fixed[order]
        return piece

    # Walk the four sign octants so consecutive segments join end to end.
    segments = [
        segment(+1.0, +1.0, forward=True),
        segment(+1.0, -1.0, forward=False),
        segment(-1.0, -1.0, forward=True),
        segment(-1.0, +1.0, forward=False)]
    return np.concatenate(segments, axis=0)


# --------------------------------------------------------------------
# The herpolhode: the track in space (Section 10.5)
# --------------------------------------------------------------------

def herpolhode_point(state, body):
    """Return the contact point in space components, one herpolhode sample.

    The contact point ``rho`` mapped from body axes into space; it lands
    in the fixed invariable plane. Accumulated along a trajectory, these
    points trace the herpolhode -- the space-frame path of ``omega``.
    """
    contact_point_body = poinsot_contact_point(state, body)
    return rotate_body_to_space(
        state.body_to_space_quaternion, contact_point_body)


def herpolhode_bounds(polhode_points, twice_kinetic_energy,
                      plane_distance):
    """Return the inner and outer radii of the herpolhode's band.

    Each contact point sits at the fixed height ``plane_distance`` above
    the invariable plane, so its radius within the plane is
    ``sqrt(|rho|^2 - plane_distance^2)``. The nearest and farthest
    approaches of the polhode to ``L`` give the two bounding radii between
    which the (generally open) herpolhode is confined (DESIGN Section 9.5).
    """
    contact_points = np.asarray(polhode_points, dtype=float) / math.sqrt(
        twice_kinetic_energy)
    contact_radii_squared = (
        np.sum(contact_points**2, axis=1) - plane_distance**2)
    # Clip away tiny negative round-off before the square root.
    radii = np.sqrt(np.clip(contact_radii_squared, 0.0, None))
    return HerpolhodeBounds(
        inner_radius=float(np.min(radii)),
        outer_radius=float(np.max(radii)))
