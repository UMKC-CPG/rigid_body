"""geometry: derived display geometry for torque-free motion.

This subpackage turns the physics the dynamics computes into the geometric
objects a student sees (ARCHITECTURE Section 3.5): the Poinsot
construction -- momental ellipsoid, invariable plane, polhode and
herpolhode -- and the reference-frame mappings that show one motion in
both the body and the space frame. It computes *what* those surfaces and
curves are in physical coordinates; it draws nothing, which is what lets
the batch tier write a polhode to disk with no renderer present.
"""
