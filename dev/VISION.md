# Vision

## Purpose

This project is an interactive, real-time teaching tool that builds
physical intuition for **rigid-body rotation** in a graduate
theoretical-mechanics course. A student manipulates a rigid body and
its initial conditions, then watches the resulting motion evolve live
alongside the conserved quantities that govern it. The goal is to make
the abstract content of Euler's equations — torque-free tumbling of an
asymmetric top, the intermediate-axis (tennis-racket / Dzhanibekov)
instability, and gyroscopic precession and nutation — visible, tangible,
and connected back to the mathematics.

It is modeled after the 3D rigid-body simulation at
https://www.ialms.net/sim/ , adapted to the pedagogical needs of a
graduate mechanics classroom.

## Scope and Non-Goals

This tool simulates **a single rigid body**. Multi-body systems,
contact, and collision response are deliberately outside its scope.
They belong to a general-purpose physics engine, and pursuing them
would dilute the pedagogical focus without advancing any goal below.

A single fixed pivot — as required by the heavy symmetric top of Goal 3
— is in scope, since it is a property of how the one body is mounted
rather than an interaction between bodies.

Extensions that are deferred but deliberately left possible are
recorded under Future Directions at the end of this document.

## Goals

1. **Visualize torque-free motion via the Poinsot construction.**
   Numerically integrate Euler's equations for a free asymmetric top
   and display the angular-velocity and angular-momentum vectors
   together with the full Poinsot construction: the **momental (inertia)
   ellipsoid** rolling without slipping on the fixed **invariable
   plane**, tracing the **polhode** on the ellipsoid and the
   **herpolhode** on the plane.

2. **Demonstrate intermediate-axis instability.** Reproduce the
   tennis-racket / Dzhanibekov effect on demand: near-axis rotation
   about the intermediate principal axis that periodically flips,
   shown live and repeatably from chosen initial conditions.

3. **Show gyroscopic precession and nutation.** Model the heavy
   symmetric top under gravity so students can watch steady precession,
   nutation, and the transition to a "sleeping" top.

4. **Ground the dynamics in the inertia tensor.** Make explicit how a
   body's shape sets its principal axes and moments of inertia, and how
   those moments in turn drive every feature of the motion.

5. **Distinguish the body frame from the space frame.** Conflating the
   two is the single most common conceptual error in this subject. The
   tool presents the same motion in both frames — simultaneously or by
   switching — with each frame's axes drawn and labeled. The angular
   velocity traces the polhode as seen in the body and the herpolhode
   as seen in space; being able to watch both descriptions of one
   motion at once is what makes the Poinsot construction intelligible
   rather than merely decorative.

6. **Offer a library of recognizable bodies.** The initial set is
   uniform-density, highly symmetric solids whose inertia tensors are
   known in closed form — sphere and ellipsoid, cube and orthorhombic
   parallelepiped, cylinder, and possibly a cone. These give students
   shapes they can reason about analytically before trusting the
   simulation. The body model must not, however, assume uniform density
   or symmetry as a permanent restriction (see Future Directions).

7. **Support real-time manipulation.** Let a student set the body and
   its initial conditions, watch the motion evolve immediately, and
   read the conserved quantities (kinetic energy and angular momentum)
   as a running check on the physics.

8. **Give the student control over time.** Pause, single-step,
   slow-motion, adjustable time scaling, and replay of a completed run.
   The Dzhanibekov flip happens too fast at natural speed for a viewer
   to see anything but the result; the ability to slow down or step
   through the flip is what turns "it flipped" into an understanding of
   how it flipped. Long-run behavior likewise needs acceleration.

9. **Teach, not just animate.** Every on-screen element maps to a named
   physical quantity so a student can trace what they see back to the
   governing equations, and read the source code to see how it is
   computed. Where a closed-form solution exists, the tool can overlay
   the analytic result on the numerical one so students see theory and
   simulation agree.

10. **Admit applied torques, not only free motion.** Although
    torque-free tumbling is the first target, the design anticipates
    external torques (a force acting through a lever arm — for example,
    gravity on the heavy top) and optional frictional damping, so the
    same engine can drive precession, nutation, and energy-loss
    demonstrations.

11. **Make demonstrations reproducible and shareable.** A complete
    scenario — body, initial conditions, applied torques, integrator
    settings, and viewpoint — can be saved and restored exactly, so an
    instructor can hand students the precise setup shown in lecture and
    recover that same setup in later years. A selected span of a run
    can additionally be exported as video, for slides and for
    asynchronous use.

## Design Principles

<!-- Non-negotiable constraints. All architectural and implementation
decisions must be consistent with these principles. -->

1. **Physical fidelity.** The motion is produced by numerically
   integrating the true equations of motion, never by scripted or faked
   animation.

2. **Numerical error is disclosed, never disguised.** The integrator is
   *not* required to be free of drift; demanding that would
   over-constrain the implementation for little pedagogical return.
   What is required is that drift be measured and surfaced. The tool
   continuously monitors the quantities that ought to be conserved and
   reports the magnitude of their departure, so a viewer can always
   tell whether an effect on screen is physics or numerics. This
   matters most in precisely the cases that are most interesting: when
   a real physical effect — precession under a very weak torque,
   accumulated over a very long time — is the same order of magnitude
   as the integration error that tracks it. In that regime an
   undisclosed error is indistinguishable from a discovery.

3. **Validated against closed-form solutions.** Wherever an analytic
   result exists it serves as the standard of correctness: the inertia
   tensors of the symmetric uniform solids, torque-free motion of the
   symmetric top, and steady precession of the heavy top. These cases
   form the regression suite (see `tests/`) and are the yardstick for
   judging integrator quality, not merely examples.

4. **Real-time interactivity.** Manipulating the body or its initial
   conditions produces an immediate, smoothly animated response. The
   tool is something a student drives, not a movie they watch.

5. **Pedagogical transparency.** Nothing physical is hidden. Every
   vector, trace, and ellipsoid is labeled with the quantity it
   represents, and the underlying mathematics is exposed rather than
   buried.

6. **Configurable visual encoding.** The mapping from physical quantity
   to color and line style is a selectable palette, not a hard-coded
   constant. Light, dark, and color-blind-safe palettes are provided at
   minimum. No distinction that carries meaning may rest on color alone
   where a label or line style can also carry it.

7. **Student-readable source.** The code itself is a teaching artifact:
   richly documented, with self-explanatory names, so a student can
   read it cold and follow the physics (see `CLAUDE.md`).

8. **Start simple, stay extensible.** The first target is live
   manipulation of torque-free motion. Guided lecture scenarios and a
   quantitative "numerical lab" mode are anticipated later, and the
   design must admit them without a rewrite.

9. **Physics decoupled from presentation.** The physics and integration
   engine is independent of the rendering and user-interface layer, so
   the display medium (a native desktop window versus a browser) is an
   architecture-level choice that can be made — and later changed —
   without touching the physics core.

10. **The computation method never dictates the simulation.** How a
    quantity is obtained is an implementation detail hidden behind a
    stable interface. A body's inertia tensor may come from a
    closed-form expression for a symmetric uniform solid, or from
    numerical integration over an arbitrary density distribution;
    either way, every other part of the system sees only the resulting
    tensor. No consumer of a quantity may branch on, or be restricted
    by, the technique that produced it. This principle is what keeps
    the simple cases we build first from foreclosing the harder cases
    we may build later.

11. **Explicit physical units.** Quantities carry real dimensions and
    are reported in stated SI units, rather than being dimensionless
    numbers chosen for visual convenience. Real units let a student
    reason about physical scale — and are a prerequisite for any later
    regime, such as a relativistic one, where the scale itself carries
    meaning.

## Future Directions

<!-- These are explicitly NOT goals for the initial versions. They are
recorded so that present-day decisions do not preclude them. Any design
choice that would make one of these impossible should be reconsidered. -->

The following extensions are not being built now, and the early versions
must not be compromised in order to accommodate them. They are recorded
because the architecture should leave the door open rather than nail it
shut.

1. **Arbitrary geometry and non-uniform density.** Bodies defined by a
   mesh, a voxel grid, or a density field rather than a closed-form
   solid, with the inertia tensor and center of mass obtained by
   numerical integration. Principle 10 exists chiefly to protect this
   path: the switch from analytic to numerical inertia computation must
   be invisible to the dynamics, the visualization, and the interface.

2. **Special relativity and meaningful scales.** Extending to
   relativistic rotation, where the very notion of a "rigid body"
   becomes problematic — a rigid body cannot be strictly relativistic,
   since rigidity implies instantaneous signal propagation across the
   body. Making that tension visible, at physically meaningful speeds
   and length scales, is itself a valuable teaching outcome. Principle
   11 (explicit units) is the groundwork for it.

3. **Charge, currents, and external fields.** A charged or
   current-carrying body responding to external electric and magnetic
   fields, adding magnetic-moment precession and electromagnetic
   torques to the mechanical picture. This is the furthest-out
   extension; it is noted only so that the treatment of external
   torques (Goal 10) is not hard-wired to gravity alone.
