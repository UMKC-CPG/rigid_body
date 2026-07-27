"""body: shapes, density, and inertia computation.

This subpackage turns a body's *shape* into the three principal moments
and their axes that the dynamics consume, and after that the shape is
never consulted again (DESIGN Section 3, ARCHITECTURE Section 3.2). The
inertia-provider seam (ARCHITECTURE Section 5.1) is what lets a closed
form today and a numerically integrated density field later be
interchangeable from the outside.
"""
