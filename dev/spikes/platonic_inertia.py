#!/usr/bin/env python3
"""Verify the inertia tensors of the five Platonic solids.

Each solid is decomposed into tetrahedra (centroid joined to every
triangular face of the convex hull) and the second-moment integral is
evaluated exactly on each tetrahedron. A Monte Carlo estimate is run
alongside as an independent check, because an exact formula applied
wrongly is still wrong and the two methods fail differently.
"""

import numpy as np
from scipy.spatial import ConvexHull, Delaunay

GOLDEN_RATIO = (1.0 + np.sqrt(5.0)) / 2.0


def platonic_vertices():
    """Vertex sets for the five regular solids, unnormalized."""

    phi = GOLDEN_RATIO
    inv = 1.0 / phi

    tetrahedron = np.array([[1, 1, 1], [1, -1, -1],
                            [-1, 1, -1], [-1, -1, 1]], dtype=float)

    cube = np.array([[x, y, z] for x in (-1, 1)
                     for y in (-1, 1) for z in (-1, 1)], dtype=float)

    octahedron = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0],
                           [0, -1, 0], [0, 0, 1], [0, 0, -1]],
                          dtype=float)

    dodecahedron = np.array(
        [[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
        + [[0, s * inv, t * phi] for s in (-1, 1) for t in (-1, 1)]
        + [[s * inv, t * phi, 0] for s in (-1, 1) for t in (-1, 1)]
        + [[s * phi, 0, t * inv] for s in (-1, 1) for t in (-1, 1)],
        dtype=float)

    icosahedron = np.array(
        [[0, s, t * phi] for s in (-1, 1) for t in (-1, 1)]
        + [[s, t * phi, 0] for s in (-1, 1) for t in (-1, 1)]
        + [[s * phi, 0, t] for s in (-1, 1) for t in (-1, 1)],
        dtype=float)

    return {'tetrahedron': tetrahedron, 'cube': cube,
            'octahedron': octahedron, 'dodecahedron': dodecahedron,
            'icosahedron': icosahedron}


def shortest_edge_length(vertices):
    """The edge length of a regular solid: its shortest vertex gap."""

    differences = vertices[:, None, :] - vertices[None, :, :]
    distances = np.linalg.norm(differences, axis=-1)
    np.fill_diagonal(distances, np.inf)
    return distances.min()


def tetrahedron_second_moment(vertex_a, vertex_b, vertex_c):
    """Exact integral of x_i x_j over a tetrahedron at the origin.

    The tetrahedron has one vertex at the origin and the other three at
    the given points. For such a tetrahedron the second-moment integral
    has the closed form used below, with the signed volume taken from
    the scalar triple product.
    """

    # ConvexHull does not orient its simplices consistently, so the
    # triple product may come out negative. The moment expression below
    # is symmetric under permuting the three corners, so taking the
    # magnitude of the volume is sufficient to orient every tetrahedron
    # outward from the interior point they all share.
    signed_volume = abs(
        np.dot(vertex_a, np.cross(vertex_b, vertex_c))) / 6.0

    corners = np.array([vertex_a, vertex_b, vertex_c])
    corner_sum = corners.sum(axis=0)

    # Integral of x_i x_j over the tetrahedron, before scaling by
    # volume: (sum of outer products + outer product of the sum) / 20.
    outer_products = sum(np.outer(corner, corner) for corner in corners)
    moment = (outer_products + np.outer(corner_sum, corner_sum)) / 20.0

    return signed_volume, signed_volume * moment / 1.0


def exact_inertia_factor(vertices):
    """Compute I / (M * edge^2) exactly for a convex polyhedron."""

    hull = ConvexHull(vertices)
    centroid_guess = vertices.mean(axis=0)
    shifted = vertices - centroid_guess

    total_volume = 0.0
    total_second_moment = np.zeros((3, 3))
    total_first_moment = np.zeros(3)

    for simplex in hull.simplices:
        corner_a, corner_b, corner_c = shifted[simplex]
        volume, moment = tetrahedron_second_moment(
            corner_a, corner_b, corner_c)
        total_volume += volume
        total_second_moment += moment
        total_first_moment += volume * (
            corner_a + corner_b + corner_c) / 4.0

    # Shift the second moment to the true center of mass.
    center_of_mass = total_first_moment / total_volume
    total_second_moment -= total_volume * np.outer(
        center_of_mass, center_of_mass)

    # Convert the second-moment matrix into the inertia tensor, then
    # normalize to unit mass and unit edge length.
    trace = np.trace(total_second_moment)
    inertia = trace * np.eye(3) - total_second_moment
    inertia /= total_volume                      # unit mass
    inertia /= shortest_edge_length(vertices) ** 2

    return inertia


def monte_carlo_inertia_factor(vertices, sample_count=4_000_000,
                               seed=20260724):
    """Independent estimate of I / (M * edge^2) by random sampling."""

    triangulation = Delaunay(vertices)
    generator = np.random.default_rng(seed)

    lower = vertices.min(axis=0)
    upper = vertices.max(axis=0)

    accepted = []
    while sum(len(block) for block in accepted) < sample_count:
        candidates = generator.uniform(
            lower, upper, size=(sample_count, 3))
        inside = triangulation.find_simplex(candidates) >= 0
        accepted.append(candidates[inside])

    points = np.concatenate(accepted)[:sample_count]
    points -= points.mean(axis=0)

    second_moment = points.T @ points / len(points)
    trace = np.trace(second_moment)
    inertia = trace * np.eye(3) - second_moment
    inertia /= shortest_edge_length(vertices) ** 2

    return inertia


def main():
    # Candidate closed forms, as I / (M * edge^2).
    sqrt5 = np.sqrt(5.0)
    candidates = {
        'tetrahedron': ('1/20', 1.0 / 20.0),
        'cube': ('1/6', 1.0 / 6.0),
        'octahedron': ('1/10', 1.0 / 10.0),
        'dodecahedron': ('(95+39*sqrt5)/300',
                         (95.0 + 39.0 * sqrt5) / 300.0),
        'icosahedron': ('(3+sqrt5)/20', (3.0 + sqrt5) / 20.0),
    }

    print(f"{'solid':<14} {'exact':>10} {'monte carlo':>13} "
          f"{'candidate':>10}  formula")
    print("-" * 72)

    for name, vertices in platonic_vertices().items():
        exact = exact_inertia_factor(vertices)
        estimate = monte_carlo_inertia_factor(vertices)

        # Confirm isotropy: all three principal moments equal.
        exact_moments = np.linalg.eigvalsh(exact)
        spread = exact_moments.max() - exact_moments.min()

        label, value = candidates[name]
        print(f"{name:<14} {exact_moments.mean():>10.6f} "
              f"{np.linalg.eigvalsh(estimate).mean():>13.6f} "
              f"{value:>10.6f}  {label}")
        print(f"{'':14} isotropy spread = {spread:.3e}")

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
