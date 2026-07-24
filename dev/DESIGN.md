# Design

> **Document hierarchy:** VISION → ARCHITECTURE → **DESIGN** → PSEUDOCODE
> → Code. For goals and principles, see `VISION.md`. For repository layout
> and module map, see `ARCHITECTURE.md`.

---

## Contents

| Section | Topic | Status |
| --- | --- | --- |
| 1 | Conventions and notation | written |
| 2 | State representation | written |
| 3 | The inertia tensor | written |
| 4 | Equations of motion | written |
| 5 | Torque models | written |
| 6 | Integrators | written |
| 7 | Conservation monitoring | written |
| 8 | Analytic solutions | written |
| 9 | Poinsot geometry | not yet written |
| 10 | Frame presentation | not yet written |
| 11 | Scenario schema | not yet written |
| 12 | Trajectory retention | not yet written |
| 13 | Scene description and palettes | not yet written |

---

## 1. Conventions and Notation

Almost every serious bug in rigid-body code is a convention error, and
convention errors are unusually cruel: they do not produce nonsense.
Confusing two conventions typically yields the *inverse* of the rotation
you wanted, and an inverse rotation still conserves energy, still
conserves angular momentum, and still looks like a plausibly tumbling
body. The simulation runs, the diagnostics stay green, and the
precession quietly proceeds in the wrong direction.

This section therefore fixes every convention once. Later sections may
assume them silently; nothing may contradict them.

The course text is Fetter and Walecka, *Theoretical Mechanics of
Particles and Continua*, and the notation here follows it so that a
student can read this source against the book.

### 1.1 The two frames

| Frame | Axes | Meaning |
| --- | --- | --- |
| Space | `X, Y, Z` | Inertial (laboratory) frame, fixed in space |
| Body | `1, 2, 3` | Fixed in the body, along its principal axes |

The body frame's axes are always the **principal axes** of the inertia
tensor, which is what makes the tensor diagonal and Euler's equations
simple (§3, §4).

The body-frame origin is the body's **center of mass** for free motion,
and the **fixed pivot** for the heavy top. This is the only case in
which the origin moves relative to the center of mass, and §3 gives the
parallel-axis shift that accounts for it.

### 1.2 Active rotations

**Rotations in this project are active.** A rotation operator moves the
physical body; the coordinate axes stay where they are.

The body's orientation is the rotation that carries it from a reference
orientation — body axes aligned with space axes — to its current one.
Written as a matrix, that rotation maps body-frame components of a
vector to space-frame components:

```
v_space = R v_body           R  =  body_to_space
v_body  = R^T v_space        R^T = space_to_body
```

Because a rotation matrix is orthogonal, `R^T = R^-1`. That identity is
precisely why the passive convention (in which the vector holds still
and the *axes* rotate) is dangerous rather than merely different: adopt
it accidentally in one place and you obtain the exact inverse rotation,
which is a perfectly well-formed rotation and will not announce itself.

**Naming rule, binding on all code.** No matrix, quaternion, or
transformation is ever named for what it *is*. It is named for the
direction it maps:

```
GOOD:  body_to_space, space_to_body, body_to_space_quaternion
BAD:   rotation_matrix, R, orientation, transform
```

A reader must be able to check a transformation's direction at the call
site without consulting a definition elsewhere.

### 1.3 Euler angles: the z-x-z sequence

Fetter and Walecka use the **z-x-z** convention (also called the
*x*-convention), and so do we. The body is brought from the reference
orientation to its current one by three successive rotations, **each
about the axis produced by the previous rotation**:

| Step | Angle | Axis rotated about | Name |
| --- | --- | --- | --- |
| 1 | `phi` | space `Z` | precession |
| 2 | `theta` | the new `x` (line of nodes) | nutation |
| 3 | `psi` | the new `z` (= body `3`) | spin |

Composed into the single body-to-space matrix:

```
R = R_z(phi) R_x(theta) R_z(psi)
```

with the conventional ranges

```
phi   in [0, 2*pi)      theta in [0, pi]      psi in [0, 2*pi)
```

The angular velocity, resolved along the **body** principal axes, is
related to the Euler-angle rates by

```
omega_1 = phi_dot * sin(theta) * sin(psi) + theta_dot * cos(psi)
omega_2 = phi_dot * sin(theta) * cos(psi) - theta_dot * sin(psi)
omega_3 = phi_dot * cos(theta) + psi_dot
```

**These relations are not invertible when `sin(theta) = 0`.** Solving
them for the Euler-angle rates introduces a factor of `1 / sin(theta)`,
so at `theta = 0` and `theta = pi` the rates are undefined. This is the
coordinate singularity usually called gimbal lock, and §2.3 explains why
it dictates the choice of state variables.

### 1.4 Symbols and their code names

VISION Principle 7 asks that the source be readable by a student cold.
Mathematical notation is therefore mirrored by expressive code names
rather than by transliterated single letters.

| Symbol | Code name | Meaning | SI unit |
| --- | --- | --- | --- |
| I₁, I₂, I₃ | `principal_moments` | Principal moments of inertia | kg m² |
| ω | `angular_velocity_body` | Angular velocity in body axes | rad/s |
| **L** | `angular_momentum_body` | Angular momentum in body axes | kg m²/s |
| **L** | `angular_momentum_space` | Same vector in space axes | kg m²/s |
| T | `kinetic_energy` | Rotational kinetic energy | J |
| **Γ** | `external_torque_body` | Applied torque in body axes | N m |
| φ | `precession_angle` | First Euler angle | rad |
| θ | `nutation_angle` | Second Euler angle | rad |
| ψ | `spin_angle` | Third Euler angle | rad |
| q | `body_to_space_quaternion` | Orientation (§2) | dimensionless |
| R | `body_to_space_matrix` | Orientation as a matrix | dimensionless |

The torque symbol **Γ** follows Fetter and Walecka.

Ordering of the principal moments is **not** assumed sorted. Several
results (notably the intermediate-axis instability of VISION Goal 2)
depend on which axis is intermediate, so the code identifies the
intermediate axis explicitly rather than relying on an index convention.

**Greek in prose, ASCII in code.** Greek letters appear in this document
and in the symbol column above, matching the textbook. They never appear
as identifiers in the source, which uses the expressive ASCII names in
the second column. Displayed formula blocks likewise use ASCII
transliterations (`omega`, `theta`), since those blocks are meant to
read as the code does. Source must stay typeable on any keyboard.

---

## 2. State Representation

### 2.1 What is integrated

The state advanced by the integrator is seven numbers:

| Quantity | Size | Meaning |
| --- | --- | --- |
| `body_to_space_quaternion` | 4 | Orientation of the body |
| `angular_velocity_body` | 3 | Angular velocity, body components |

Both are stored as bare SI floats. Per ARCHITECTURE §5.5, units exist
only at the scenario boundary; nothing at this level carries a unit
object. The seven numbers are laid out as a flat array so that the
integrator's inner loop presents the array-shaped interface that
ARCHITECTURE §5.4 reserves for a future compiled kernel.

**Translation is not part of the state.** For a freely tumbling body the
center of mass moves uniformly and its motion decouples entirely from
the rotation, so simulating it would add a variable that changes nothing
on screen. For the heavy top the pivot is fixed and there is no
translation to simulate. Rotation is the whole subject (ARCHITECTURE
"Scope and Non-Goals"), and the state reflects that.

### 2.2 Quaternion conventions

Quaternions invite the same class of silent error as rotations, so their
conventions are fixed here with equal care.

**Scalar first.** A quaternion is stored as

```
q = (q_w, q_x, q_y, q_z)
```

with the real part first. The alternative (scalar-last) ordering is
common in other software, and mixing the two produces a well-formed
quaternion that represents the wrong rotation — the same failure mode as
§1.2. Any quaternion crossing a library boundary is converted
explicitly.

**Direction.** `q` represents the *body-to-space* rotation, matching
`R` of §1.2. A vector is carried from body to space components by

```
v_space = q * (0, v_body) * q_conjugate
```

**Unit norm.** Only unit quaternions represent rotations. The constraint
`|q| = 1` is preserved by the integrator; the renormalization policy
belongs to §6 because it interacts with the choice of integrator.

**Double cover.** `q` and `-q` represent the *same* physical rotation.
Nothing may treat them as distinct: comparisons, tests, and any
interpolation must account for the sign ambiguity rather than assume a
canonical representative. This matters for the determinism test of
ARCHITECTURE §8.6, which compares trajectories.

### 2.3 Why quaternions rather than Euler angles

| Representation | Parameters | Constraints | Singularity |
| --- | --- | --- | --- |
| Euler angles | 3 | none | yes, at `sin(theta) = 0` |
| Rotation matrix | 9 | 6 (orthogonality) | none |
| Unit quaternion | 4 | 1 (unit norm) | none |

Euler angles are the minimal parameterization and the one the course
text teaches, which makes them tempting as state variables. They are
nonetheless unusable for integration here, for a reason that is
pedagogical before it is numerical.

As §1.3 showed, recovering the Euler-angle rates requires dividing by
`sin(theta)`. As `theta` approaches `0` or `pi` the individual rates
`phi_dot` and `psi_dot` diverge, even though the body is doing nothing
unusual — only their *sum* remains well behaved. An integrator working
in these coordinates takes wilder and wilder steps as the singularity is
approached, and the body on screen jitters, stalls, or flies apart.

None of that is physics. It is an artifact of the coordinates, and
VISION Principle 2 draws a hard line at exactly this: a viewer must
never mistake a numerical artifact for a physical effect.

What makes it intolerable rather than merely unfortunate is *where* the
singularity sits. `theta = 0` is not an obscure corner of the parameter
space:

- The **sleeping top** (VISION Goal 3) is the configuration `theta -> 0`.
  It is one of the demonstrations the tool exists to show.
- The **Chandler wobble** (VISION Goal 12) has small `theta` throughout,
  since the Earth's figure axis stays within a few tenths of an
  arcsecond of its rotation axis.

Integrating in Euler angles would therefore place a violent coordinate
artifact precisely on top of the two cases where the physics is most
delicate and the student is least equipped to tell the difference.

The rotation matrix has no singularity but carries nine numbers bound by
six constraints, and numerical drift pushes it off the rotation group,
requiring periodic re-orthogonalization. The unit quaternion has no
singularity, carries four numbers bound by one constraint, and is
restored to the constraint surface by a single division. It is the
natural choice, and it is what the tool integrates.

### 2.4 Quaternion kinematics

The orientation evolves according to

```
q_dot = 0.5 * q * (0, angular_velocity_body)
```

where `*` is the quaternion product and the second factor is the pure
quaternion built from the body-frame angular velocity.

The ordering matters and is a direct consequence of §2.2. Because `q`
maps body to space and `omega` is expressed in **body** components, the
angular-velocity factor multiplies from the **right**. Had `omega` been
given in space components the correct form would instead be
`q_dot = 0.5 * (0, omega_space) * q`. Using one form with the other
frame's angular velocity is another member of the silent-inverse family
of §1.2.

This equation, together with Euler's equations for `omega_dot` (§4),
closes the system.

### 2.5 Derived quantities

These are computed from the state rather than stored in it, so they can
never disagree with it:

```
angular_momentum_body  = (I_1*omega_1, I_2*omega_2, I_3*omega_3)
angular_momentum_space = body_to_space_matrix * angular_momentum_body
kinetic_energy         = 0.5 * (I_1*omega_1^2 + I_2*omega_2^2
                                + I_3*omega_3^2)
```

`angular_momentum_body` is componentwise because the body frame is the
principal-axis frame (§1.1), where the inertia tensor is diagonal.

Under torque-free motion, `angular_momentum_space` is **constant** and
`kinetic_energy` is **constant**. These are the two quantities the
conservation monitor watches (§7), the two invariants the Poinsot
construction is built from (§9), and among the sharpest oracles
available to the test suite (ARCHITECTURE §8.2). Note that it is the
*space-frame* angular momentum that is conserved; the body-frame
components change continuously as the body tumbles beneath the fixed
vector, and that difference is itself worth showing a student (§10).

### 2.6 Euler angles as an output

Although `phi`, `theta`, and `psi` are not integrated, they are what the
course text uses and what a student expects to see. They are therefore
computed from the quaternion whenever needed for display, and never fed
back into the state.

The extraction inherits the degeneracy of §1.3: when `sin(theta)` is
near zero, `phi` and `psi` are individually undetermined and only their
sum (or difference, according to the sign of `cos(theta)`) is
meaningful. The display must not print two rapidly changing numbers as
though they were measurements. In that regime the tool reports the
well-defined combination and marks `phi` and `psi` as degenerate.

This is VISION Principle 2 applied to the user interface rather than to
the integrator: the quantity is genuinely undefined there, and saying so
is more honest, and more instructive, than displaying noise.

---

## 3. The Inertia Tensor

The inertia tensor is where a body's *shape* enters the dynamics, and
after it has been computed the shape is never consulted again. Euler's
equations (§4) see three numbers; the Poinsot construction (§9) sees the
same three numbers; nothing downstream knows whether they came from a
sphere or from a numerically integrated density field. That is
ARCHITECTURE §5.1 stated physically rather than structurally.

### 3.1 Definition and general properties

For a body of density `rho(r)`, the tensor about the origin of the body
frame is

```
I_jk = integral rho(r) * (r^2 * delta_jk - r_j * r_k) dV
```

Three properties follow from that definition and hold for every physical
body. They are asserted wherever a tensor is produced (ARCHITECTURE
§8.3), because a violation means the body is not a body:

1. **Symmetry.** `I_jk = I_kj`, immediately from the definition.
2. **Positive definiteness.** All three eigenvalues are strictly
   positive for any body with volume.
3. **The triangle inequalities.** `I_1 + I_2 >= I_3` and its two
   permutations. This one is easy to violate by accident and impossible
   to violate in nature: no distribution of mass produces moments that
   fail it.

The eigenvectors of the tensor are the **principal axes**, and its
eigenvalues are the **principal moments** `I_1, I_2, I_3`. In the
principal-axis frame — which §1.1 adopts as the body frame — the tensor
is diagonal, and that is the entire reason for choosing it.

### 3.2 Closed forms for the uniform primitives

VISION Goal 6 asks for solids a student can reason about analytically
before trusting the simulation. Each is given here about its center of
mass, in its own principal-axis frame, for total mass `M`.

```
Sphere, radius a
    I_1 = I_2 = I_3 = (2/5) M a^2

Ellipsoid, semi-axes a, b, c along axes 1, 2, 3
    I_1 = (M/5) (b^2 + c^2)
    I_2 = (M/5) (c^2 + a^2)
    I_3 = (M/5) (a^2 + b^2)

Parallelepiped, full edge lengths a, b, c along axes 1, 2, 3
    I_1 = (M/12) (b^2 + c^2)
    I_2 = (M/12) (c^2 + a^2)
    I_3 = (M/12) (a^2 + b^2)

Cube, edge a
    I_1 = I_2 = I_3 = (1/6) M a^2

Cylinder, radius a and height h, symmetry axis along 3
    I_1 = I_2 = (M/12) (3 a^2 + h^2)
    I_3 = (1/2) M a^2

Cone, base radius a and height h, symmetry axis along 3
    I_1 = I_2 = (3M/80) (4 a^2 + h^2)
    I_3 = (3/10) M a^2

Platonic solids, edge a  --  every one is isotropic, I_1 = I_2 = I_3
    Tetrahedron     I = (1/20) M a^2
    Cube            I = (1/6) M a^2
    Octahedron      I = (1/10) M a^2
    Dodecahedron    I = ((95 + 39*sqrt(5)) / 300) M a^2
    Icosahedron     I = ((3 + sqrt(5)) / 20) M a^2
```

The five Platonic constants were verified numerically before being
written here, by two independent methods — exact decomposition of each
polyhedron into tetrahedra, and Monte Carlo integration — which agreed
to the precision of the sampling. The check earned its keep: a value for
the icosahedron taken from memory was wrong by exactly a factor of two,
and would otherwise have reached the code by way of this document. The
script is kept at `dev/spikes/platonic_inertia.py`.

Two notes on the cone, which is the only entry with a trap in it. Its
center of mass lies at `h/4` above the base, not at `h/2`, and the
transverse moment above is taken about that point — the more commonly
quoted `(3/20)M(a^2 + 4h^2)` is about the *apex* and is not what this
table wants. The sphere and cube rows are written out despite being
special cases of the rows above them, because both are degenerate in a
way §3.3 makes use of.

### 3.3 Classification, and why the cube is interesting

The pattern of degeneracy among the principal moments determines what
motion is even possible, so it is worth naming:

| Class | Moments | Examples |
| --- | --- | --- |
| Spherical top | all three equal | sphere, **all Platonic solids** |
| Symmetric top | exactly two equal | cylinder, cone, spheroid |
| Asymmetric top | all three distinct | box, general ellipsoid |

A spherical top has constant ω and shows no free precession at all. A
symmetric top precesses steadily about its symmetry axis. Only the
asymmetric top produces the full Poinsot picture, and only it can
exhibit the Dzhanibekov flip.

The spherical-top row deserves emphasis, because it is where student
intuition fails most reliably. **A cube tumbles exactly as a sphere of
the same mass does.** Every axis is a principal axis; a freely rotating
cube spins steadily about whatever axis it was given, with no wobble
whatsoever. So does a tetrahedron, and so does an icosahedron.

This is not a coincidence of the arithmetic in §3.2. A body whose
symmetry group acts irreducibly on three-dimensional space *must* have
an isotropic inertia tensor, because the tensor commutes with every
symmetry operation and Schur's lemma then forces it to be a multiple of
the identity. All five Platonic solids have such symmetry groups, which
is why all five are spherical tops despite looking nothing alike. The
numerical check reported in §3.2 found the isotropy exact to machine
precision, as the argument requires.

The consequence for the tool is a demonstration available in one click:
switch the body from a cube to a slightly unequal parallelepiped, and
motion that was rigidly steady becomes the full tumbling picture. What
changed was not the amount of symmetry visible to the eye but whether
the moments are exactly equal.

It also fixes the scope of VISION Goal 2: **the Dzhanibekov flip
requires three distinct moments.** There is no intermediate axis to be
unstable about unless there is a strict ordering, so the demonstration
must use an asymmetric body and the interface should not offer the
instability for a body that cannot exhibit it.

### 3.4 Obtaining the principal axes

For the primitives of §3.2 stated in their natural orientation, the
tensor is already diagonal and the principal axes are the coordinate
axes. No eigenvalue problem is solved, and no numerical error is
introduced, in the common case.

The general case — a body given in some other orientation, and every
body once Future Direction 1 arrives — requires diagonalizing a real
symmetric matrix. This uses a symmetric eigensolver (`numpy.linalg.eigh`
rather than the general `eig`), which guarantees real eigenvalues and an
orthonormal eigenvector set. Two corrections are then applied:

1. **Handedness.** An eigensolver may return a left-handed set. If the
   determinant of the eigenvector matrix is negative, one axis is
   negated to restore a right-handed frame. Skipping this yields a
   reflection rather than a rotation, which is §1.2's silent-inverse
   failure wearing a different hat.
2. **Degenerate subspaces.** When two or three moments coincide, the
   corresponding eigenvectors are **not unique** — any orthonormal basis
   of the degenerate subspace is equally valid, and a solver will return
   an arbitrary one that can change discontinuously from call to call.
   Left alone, the drawn body axes of a cylinder would jitter or spin
   for no physical reason. The tensor is therefore diagonalized **once**,
   when the body is built, and the resulting axes are stored and reused;
   they are never recomputed per frame. Where a canonical choice is
   still needed, the degenerate subspace is aligned to the body's
   geometric symmetry axis.

Point 2 is a direct application of VISION Principle 2: an arbitrary
choice made by a library is not physics, and must not appear on screen
as motion.

### 3.5 Shifting to a pivot

The heavy top of VISION Goal 3 rotates about a fixed pivot rather than
its center of mass. With `d` the vector from the center of mass to the
pivot, the parallel-axis theorem gives

```
I_pivot_jk = I_com_jk + M * (d^2 * delta_jk - d_j * d_k)
```

The subtlety worth stating: **the shifted tensor is in general no longer
diagonal in the original principal axes.** Displacing along a principal
axis preserves diagonality, and that is the usual textbook case — a top
pivoted on its symmetry axis stays a symmetric top. Any other
displacement mixes the axes and requires re-diagonalizing by §3.4 to
recover a principal-axis body frame.

Because §1.1 defines the body frame as the principal-axis frame, this
re-diagonalization is not optional bookkeeping. If it is skipped, the
inertia tensor is not diagonal, and Euler's equations in the form §4
uses are simply wrong.

### 3.6 Direct entry of the moments

A body may also be specified by giving `I_1, I_2, I_3` directly, without
any shape at all. This is not a convenience feature but a requirement of
VISION Goal 12: the Chandler wobble needs the Earth's *ratio* of
principal moments, and modeling the Earth's actual shape to obtain it
would be absurd when the ratio is the only thing that matters.

Directly entered moments are validated against §3.1 before use —
positive, and satisfying the triangle inequalities. A set that fails is
rejected with an explanation of which inequality failed and by how much,
rather than being quietly accepted and integrated into nonsense. A user
is free to explore unusual bodies; they are not free to explore
impossible ones without being told.

When a body is given this way it has no geometry to draw. The
visualization falls back to displaying the momental ellipsoid alone
(§9), which is in any case the object the dynamics actually cares about.

### 3.7 The provider boundary in practice

ARCHITECTURE §5.1 requires that consumers never learn how a tensor was
computed. Concretely, the provider returns the same three items whatever
its method: the total mass, the center of mass, and the principal
moments with their axes.

When `numerical_inertia.py` is eventually written, its first obligation
is to reproduce §3.2 on every primitive to a stated tolerance. That test
is what certifies the boundary, because it demonstrates that the two
methods are interchangeable from the outside — which is exactly the
claim VISION Principle 10 makes.

### 3.8 Rigidity as a computational invariant

Everything in this section is computed **once**, when the body is
built, and never recomputed while the simulation runs: the inertia
tensor, its diagonalization, the principal axes, the principal moments,
and the momental ellipsoid's shape that follows from them (§9).

This is worth stating as a rule rather than leaving as an optimization,
because it is not an optimization. In the body frame the inertia tensor
is *constant by definition* — that constancy is precisely what the word
"rigid" means. A body whose body-frame inertia tensor changed with time
would not be a rigid body, and Euler's equations as §4 writes them
would not apply to it. So the rule is:

> Any quantity that is fixed in the body frame is computed once at
> construction and reused thereafter. Recomputing it per frame is not
> merely wasteful; it invites a value that should be constant to drift,
> which would put non-physical motion on screen.

The §3.4 concern about jittering degenerate axes is one instance of
this general rule rather than a special case.

Changing a body — editing an edge length, moving the pivot, entering
different moments — **constructs a new body** rather than mutating an
existing one. That keeps the invariant true by construction and makes
the scenario record (§11) unambiguous about which body produced which
trajectory.

**Where the rule breaks, and why that is interesting.** The assumption
fails exactly where rigidity itself fails, and the tool is eventually
meant to say so:

- A relativistic body (VISION Future Direction 2) cannot be strictly
  rigid, since rigidity implies instantaneous signal propagation.
- The **real Earth** is not rigid. This is not a technicality: it is the
  whole content of VISION Goal 12. A rigid Earth would show a free
  precession of about 305 days, and the observed Chandler wobble is
  about 433 days. That discrepancy *is* the failure of this section's
  central assumption, measured in days. The Earth's inertia tensor is
  not constant, because the Earth yields elastically as it wobbles.

So the tool computes the rigid answer, displays it honestly, and the
gap to the observed value becomes the lesson rather than an error.

---

## 4. Equations of Motion

### 4.1 Euler's equations

Angular momentum in the space frame obeys `dL/dt = Gamma`. Rewriting
that in the body frame, which rotates with angular velocity `omega`,
introduces the usual transport term:

```
(dL/dt)_body + omega x L = Gamma
```

Substituting `L = I omega` gives the equation the simulation actually
integrates:

```
I omega_dot + omega x (I omega) = Gamma
```

**The step that removes `dI/dt` is exactly §3.8.** Differentiating
`L = I omega` in the body frame should leave a term `(dI/dt) omega`,
and it is dropped because the body-frame inertia tensor is constant —
which is what rigidity means. Euler's equations are therefore not merely
convenient in the body frame; they are *available* only there. In the
space frame `I` changes as the body turns, that term survives, and the
equations lose their simple form.

Because §1.1 puts the body frame on the principal axes, `I` is diagonal
and the vector equation separates into three scalar ones:

```
I_1 omega_dot_1 = (I_2 - I_3) omega_2 omega_3 + Gamma_1
I_2 omega_dot_2 = (I_3 - I_1) omega_3 omega_1 + Gamma_2
I_3 omega_dot_3 = (I_1 - I_2) omega_1 omega_2 + Gamma_3
```

Inverting `I` is division by a scalar per component, never a matrix
solve. This is one of the reasons the principal-axis frame is worth the
diagonalization cost of §3.4.

### 4.2 The complete system

Euler's equations give `omega_dot`; §2.4 gives `q_dot`. Together they
close the seven-component system that §2.1 defines as the state:

```
q_dot     = 0.5 * q * (0, omega)
omega_dot = ( Gamma - omega x (I omega) ) / I        [componentwise]
```

The derivative function presented to the integrator therefore has the
shape

```
state_derivative(time, state, body, torque_models) -> derivative
```

and is a pure function: it reads the state and returns a derivative,
mutating nothing. That purity is what lets a multi-stage integrator
(§6) evaluate it at trial points without the evaluations interfering,
and it is what makes the determinism guarantee of ARCHITECTURE §6.4
straightforward to keep.

Note the asymmetry between the two halves. The `omega_dot` equation is
where the physics lives; the `q_dot` equation is pure bookkeeping,
recording how the orientation follows from an angular velocity that has
already been determined. A torque never appears in `q_dot`.

### 4.3 The gyroscopic term and what makes rotation interesting

The term `omega x (I omega)` carries the whole of free-rotation
behavior. It is worth asking when it vanishes, because the answer
reproduces the classification of §3.3 exactly.

It vanishes precisely when `I omega` is parallel to `omega`, which
happens in two circumstances:

1. **`omega` lies along a principal axis.** Then `I omega = I_k omega`
   for that axis, and the cross product is zero.
2. **The body is a spherical top.** Then `I omega = I omega` for *every*
   `omega`, so the term vanishes identically and permanently.

The second case is why a cube, a tetrahedron, and a sphere all rotate
with constant `omega` and show no free precession whatever (§3.3). It
is not that their wobble is small; the term that would produce it is
identically zero. The first case is why rotation about a principal axis
is an equilibrium — and §4.4 shows that not all such equilibria are
stable.

For torque-free motion the scalar equations reduce to

```
omega_dot_1 = ((I_2 - I_3) / I_1) omega_2 omega_3
omega_dot_2 = ((I_3 - I_1) / I_2) omega_3 omega_1
omega_dot_3 = ((I_1 - I_2) / I_3) omega_1 omega_2
```

which makes plain that if any two components vanish the third is
constant: rotation about a principal axis persists.

### 4.4 The intermediate-axis instability, derived

VISION Goal 2 asks for the Dzhanibekov flip. It is worth deriving here
rather than treating it as an empirical curiosity, because the
derivation yields a *number* the tool can use and the test suite can
check.

Take torque-free motion with the body spinning about axis 1 at rate
`Omega`, and perturb it:

```
omega = (Omega, eps_2, eps_3),   with eps small
```

Discarding products of small quantities, `omega_1` stays constant and

```
eps_dot_2 = ((I_3 - I_1) Omega / I_2) eps_3
eps_dot_3 = ((I_1 - I_2) Omega / I_3) eps_2
```

Differentiating the first and substituting the second:

```
eps_ddot_2 = Omega^2 * ((I_3 - I_1)(I_1 - I_2) / (I_2 I_3)) * eps_2
```

Everything follows from the sign of that coefficient:

| Sign | Behavior | When |
| --- | --- | --- |
| Negative | Oscillation; stable | `I_1` is the largest or smallest moment |
| Positive | Exponential growth | `I_1` is the **intermediate** moment |

The product `(I_3 - I_1)(I_1 - I_2)` is positive exactly when `I_1` lies
between the other two, which is the tennis-racket theorem. Rotation
about the axis of largest or smallest moment is stable; rotation about
the intermediate axis is not.

When unstable, the perturbation grows as `exp(sigma t)` with

```
sigma = Omega * sqrt( (I_3 - I_1)(I_1 - I_2) / (I_2 I_3) )
```

This growth rate is useful in three separate places, which is why it is
worth having in closed form:

- **As an oracle** (ARCHITECTURE §8.2). A simulation started near the
  intermediate axis must show a perturbation growing at this rate. That
  is a far sharper test than "the body eventually flips".
- **As a time-scale estimate.** The interval before a flip is roughly
  `(1/sigma) * ln(1/eps_0)` for an initial perturbation `eps_0`, so the
  tool can choose sensible substeps-per-frame (ARCHITECTURE §6.3) and
  tell a student roughly when to watch rather than making them wait
  blindly.
- **As an interface guard.** `sigma` is zero unless the three moments
  are distinct, which is the §3.3 restriction stated quantitatively:
  offering the demonstration for a symmetric or spherical top is
  offering something that provably cannot happen.

A caution for the implementation: started *exactly* on the intermediate
axis, the equations say the body stays there forever, and to the extent
floating-point arithmetic is exact about zero, the simulation will show
that. The flip is triggered by perturbation, so the scenario must supply
a deliberate, recorded initial tilt rather than relying on rounding
error to provide one. Depending on rounding noise would make the
demonstration irreproducible, in violation of VISION Goal 11.

### 4.5 How torque enters

`Gamma` is not computed here. The equations above take it as given, and
§5 supplies it through an interface that this section constrains in
exactly two ways:

1. **Body components.** `Gamma` arrives resolved along the body
   principal axes, because that is the frame Euler's equations are
   written in. A torque naturally expressed in space — gravity is the
   obvious case — is rotated by `space_to_body` (§1.2) *inside* the
   torque model, not afterward.
2. **Additive.** Several torque models may act at once, and the total is
   their sum. Nothing in the equations distinguishes gravity from
   friction from a future electromagnetic term (VISION Future
   Direction 3), which is what lets §5 add models without touching §4.

### 4.6 Conserved quantities, and their rates when not conserved

Two exact statements follow from the equations and are the foundation of
the conservation monitor (§7):

```
d(kinetic_energy)/dt      = Gamma . omega
d(angular_momentum_space)/dt = Gamma_space
```

With `Gamma = 0` both right-hand sides vanish, giving the familiar
constancy of energy and of space-frame angular momentum under
torque-free motion.

The more useful observation is that these hold **whether or not** a
torque is acting. A monitor that only checked for constancy would be
blind the moment gravity was switched on, which is precisely when the
heavy top (Goal 3) makes it most worth watching. Checking the *rate*
instead keeps the diagnostic alive in every scenario: the monitor
compares the observed change in energy against `Gamma . omega`
accumulated over the same interval, and reports the discrepancy.

That discrepancy — not the raw drift — is the honest measure of
integration error under applied torque, and it is what VISION
Principle 2 puts on screen.

---

## 5. Torque Models

### 5.1 The interface

A torque model answers one question: given the current instant, what
torque acts on this body? It presents a single operation,

```
torque_body(time, state, body) -> 3-vector in body components
```

and obeys the two constraints §4.5 imposed — body components, and
additive. It is also **pure**, for the same reason §4.2 requires the
derivative function to be: a multi-stage integrator evaluates the
derivative at trial states that are not on the trajectory, and a model
that recorded or accumulated anything during those evaluations would
corrupt itself.

The full state is passed even though most models use only part of it.
Gravity needs the orientation, viscous damping needs the angular
velocity, and a driven torque would need the time. Passing everything
keeps one signature for all of them.

### 5.2 Torque-free motion

Torque-free motion is not a special case in the code. It is an **empty
list of models**, whose sum is the zero vector. There is no null model,
no `if torque is None`, and no separate free-rotation code path — which
matters because VISION Goal 1 makes torque-free motion the most-used
configuration, and a branch that only the common case takes is a branch
whose failure would be found late.

### 5.3 Uniform gravity through a pivot

This is the heavy top of VISION Goal 3. The body turns about a fixed
pivot, and gravity acts at the center of mass, which is displaced from
that pivot.

Let `pivot_to_com_body` be the vector from the pivot to the center of
mass. It is expressed in **body** components, where §3.8 guarantees it
is constant. The torque about the pivot is then

```
Gamma_body = M * cross(pivot_to_com_body,
                       space_to_body * gravity_space)
```

The order of operations here is deliberate. Gravity is naturally a
space-frame vector — it points down, whatever the body is doing — so it
is rotated into the body frame and crossed with a lever arm that never
changes. The alternative, rotating the lever arm into space and crossing
there, is algebraically equivalent but computes a quantity that is
constant in one frame by transforming it every step, and then has to
rotate the result back. One rotation per step instead of two, and the
constant stays visibly constant.

Two consequences worth stating:

- Because the pivot is the origin of rotation, the inertia tensor must
  be the one **about the pivot**, not about the center of mass. §3.5
  gives the shift, including the warning that it generally destroys
  diagonality unless the displacement is along a principal axis.
- The classic top has `pivot_to_com_body` along the symmetry axis, which
  is the case that preserves diagonality. That is not an accident of
  pedagogy; it is why the textbook case is tractable.

For a rapidly spinning symmetric top, the steady precession rate is

```
precession_rate = M * g * l / (I_3 * omega_3)
```

with `l` the pivot-to-center-of-mass distance. §8 develops this as an
analytic oracle and as the overlay that VISION Goal 9 draws beside the
numerical solution.

### 5.4 Viscous damping

A simple, honest dissipation model: a torque opposing the angular
velocity,

```
Gamma_body = -damping_coefficient * angular_velocity_body
```

Its behavior is exactly what §4.6 predicts. The power is

```
Gamma . omega = -damping_coefficient * |omega|^2
```

which is negative whenever the body is rotating, so energy decreases
monotonically and the body spins down. Angular momentum also decreases,
because this is a genuine external torque.

This is the right model for a body immersed in a fluid or dragging on
its mount. It is *not* a model of internal friction, and conflating the
two is a mistake worth its own subsection.

### 5.5 Internal dissipation is not a torque

The reference simulation this project is modeled on offers an "internal
friction" alongside external friction, and the distinction matters
enough to be worked out rather than copied.

Internal dissipation means a body losing energy to its own deformation
while nothing outside it exerts any torque. The defining feature is that
**angular momentum is conserved and energy is not**. That combination
cannot be produced by any external torque, since `Gamma = 0` is exactly
what conserving `L` requires, and `Gamma . omega = 0` then forces the
energy to be constant too.

It cannot be produced by a *rigid* body either, and this is the
important part. For a rigid body, `L_space` fixed and `I` constant in
the body frame together determine the motion completely — that is just
the torque-free equation of §4.3 — and its energy is automatically
constant. There is no freedom left in which to dissipate anything.

**So internal dissipation is a statement that the body is not rigid.**
Any model of it in this tool is therefore a phenomenological addition
outside the rigid-body framework, and VISION Principle 12 requires it
be labeled as such on screen rather than presented as ordinary physics.

The physics it produces is genuinely worth showing. At fixed `|L|`,
kinetic energy is smallest when rotation is about the axis of **largest**
moment of inertia. A dissipating body therefore drifts toward that
state, whatever it started in. Three consequences a student should meet:

- A tumbling satellite settles into a flat spin about its maximum-moment
  axis. Explorer 1 did precisely this in 1958, to the surprise of its
  designers, and it is the reason spacecraft are not spin-stabilized
  about a minor axis.
- The intermediate-axis instability of §4.4 and this slow drift are
  different phenomena with different timescales; the first is fast and
  conservative, the second slow and dissipative.
- The Chandler wobble of VISION Goal 12 should damp out in a few decades
  under exactly this mechanism, yet it persists — which means something
  continues to excite it. The same non-rigidity that stretches the free
  precession period from 305 to 433 days also damps the motion, so one
  model connects both halves of that discussion.

Because it is not a torque, it does not implement the §5.1 interface. It
belongs in a separate category of *state modifiers* applied after the
torque sum, constrained to hold `angular_momentum_space` fixed while
reducing `kinetic_energy`. It is **not scheduled for the first version**;
it is described here so that §5.1 is not mistakenly widened to
accommodate something that does not belong in it.

### 5.6 Composition, and an ordering subtlety

The total torque is the sum over all active models. Addition commutes,
so the physics does not depend on their order.

**Floating-point addition, however, is not associative.** Summing three
torques in a different sequence can give a bitwise-different result, and
ARCHITECTURE §6.4 promises trajectories identical bit for bit. The order
of the model list is therefore fixed, recorded in the scenario (§11),
and iterated in that recorded order. This costs nothing and removes an
irreproducibility that would otherwise be nearly impossible to diagnose,
since it would appear only as a last-digit difference that slowly grows.

### 5.7 Room left for later models

Nothing in §4 or in this section names a specific torque, which is what
lets new ones arrive as additions. Two are anticipated:

- **Electromagnetic torque** (VISION Future Direction 3), of the form
  `magnetic_moment x magnetic_field`, which is already the right shape
  for the §5.1 interface — it depends on the orientation, exactly as
  gravity does.
- **Driven or time-dependent torque**, for forced-precession
  demonstrations, which is why `time` appears in the signature despite
  no current model using it.

A model that cannot be expressed through this interface is a signal that
it is not a torque at all, and §5.5 is the worked example of what that
situation looks like.

---

## 6. Integrators

The integrator is the one component whose *errors* are meant to be seen.
Everywhere else in this document a wrong number is a bug; here a nonzero
error is expected, and VISION Principle 2 asks only that it be measured
and disclosed rather than hidden. That framing decides almost every
choice below, because it means the goal is never "make the error zero"
but "make the error understood."

Two facts settled earlier converge in this section. §2.2 fixed the
quaternion conventions but deferred the renormalization policy to here,
because it depends on which integrator is running. And ARCHITECTURE §8.4
split the accuracy requirement into two regimes — a classroom regime and
a long-integration regime — that call for *different kinds* of
integrator, not merely different step sizes. Both threads are picked up
below.

### 6.1 The integrator is a selectable strategy

Which integrator runs is a scenario setting, not a hard-wired choice.
This follows directly from ARCHITECTURE §2, where the integrator type is
one of the fidelity knobs that distinguish the interactive tier from the
batch tier, and from the kernel boundary of ARCHITECTURE §5.4, which
isolates the time-stepping loop behind a narrow interface so a compiled
version can replace it later.

Every integrator therefore presents the same shape:

```
advance(state, dt, derivative_function) -> new_state
```

It receives the flat seven-component array of §2.1, a time step, and the
pure derivative function of §4.2, and returns the advanced array. It
knows nothing about bodies, torques, or rendering; those are already
sealed inside `derivative_function` by the time the integrator sees it.
That narrowness is what lets the strategy be swapped per scenario and,
eventually, per language (ARCHITECTURE §5.4).

**The step is fixed, never adaptive.** ARCHITECTURE §6.2 advances the
physics by a fixed `dt` and a fixed number of substeps per rendered
frame, and §6.3 there maps every time control onto the substep count.
That model requires a fixed-step integrator: one call, one `dt`, one
step. It is worth naming the road not taken, because it is the obvious
one — `scipy.integrate.solve_ivp` with an adaptive method such as
`RK45`. Adaptive solvers choose their own internal step sizes to hit a
tolerance, which is exactly the coupling ARCHITECTURE §6.2 forbids: the
sequence of steps would depend on the trajectory in a way that
complicates the bit-for-bit reproducibility of ARCHITECTURE §6.4, and
the natural "advance by one substep" model of the time controls would
have no clean meaning. The integrators here take a step of a size the
scenario chose.

### 6.2 The baseline: fixed-step RK4

The default integrator is the classical fourth-order Runge-Kutta method.
It evaluates the derivative four times per step — once at the start, twice
at trial midpoints, once at a trial endpoint — and combines them in the
standard weighting. Its purity requirement is why §4.2 insisted the
derivative function mutate nothing: the two midpoint evaluations are at
trial states off the trajectory, and a derivative that accumulated state
across them would corrupt the step.

RK4 is chosen for the classroom regime for three reasons. It is accurate
enough that its error sits well below the conserved-quantity drift the
monitor already discloses (§6.4). It is explicit, so each step is a fixed
sequence of arithmetic with no inner solve — which keeps it fast enough
for real-time interaction (Principle 4) and trivially deterministic
(ARCHITECTURE §6.4). And it is simple enough to read, which Principle 7
values in a component a student may well want to understand.

Its global error is order four: `error ~ C * dt^4`. That is not just a
selling point but a *testable* claim, and it supplies one of the derived
tolerances ARCHITECTURE §8.4 demands instead of tuned ones. Halving `dt`
must cut the error by about a factor of sixteen, and a convergence study
that measures the observed order against a case with a known solution
(§8) both confirms the implementation and calibrates what drift to expect
at a given step size. A tolerance chosen this way is justified by the
method's order; a tolerance chosen by loosening it until the test passes
is the failure ARCHITECTURE §8.4 exists to prevent.

### 6.3 Keeping the quaternion a rotation

RK4 does not know that `q` must stay on the unit sphere. The kinematic
equation of §2.4 keeps `|q|` constant in exact arithmetic, but a
finite-step method takes short straight-line moves along the tangent and
lands slightly off the sphere each step. Left alone, `|q|` drifts away
from one, and a non-unit quaternion applied as a rotation (§2.2) quietly
introduces a scaling — a distortion of exactly the kind Principle 2
forbids putting on screen unlabeled.

The remedy is a projection: after each completed step, divide the
quaternion by its norm, returning it to the sphere. This is the
renormalization §2.2 deferred to here. It costs one square root and one
division per step, which is negligible beside four derivative
evaluations.

Two details matter.

- **Renormalize after the whole step, not inside the stages.** The four
  RK4 stages are combined linearly, and their trial quaternions are
  *meant* to be slightly off the sphere — that is how the linear
  combination reconstructs a fourth-order-accurate result. Projecting a
  trial stage back onto the sphere mid-step would corrupt that
  combination and degrade the order. The derivative function tolerates a
  slightly non-unit `q` over the span of one step; the projection
  restores the constraint once, before the small error can accumulate.
- **The projection never changes the sign.** Dividing by a positive norm
  cannot flip `q` to `-q`, so the double-cover care of §2.2 is
  undisturbed by renormalization.

Projection is the correct tool for RK4 precisely because RK4 has no
structure worth protecting. §6.5 shows that for a structure-preserving
integrator the same projection would be a mistake — one more place where
the integrator choice and the norm policy are entangled, exactly as §2.2
warned.

### 6.4 Two regimes of error

RK4 is excellent and still not enough for every case the project
anticipates. The reason is the distinction ARCHITECTURE §8.4 drew, now
stated at the integrator level.

RK4 is not symplectic. For a conservative system its error in the
conserved quantities is **secular** — it accumulates with time rather
than oscillating around the true value. Over a classroom demonstration
this is invisible: the accumulated energy drift stays far below the
`1e-6` relative bound of ARCHITECTURE §8.4, the monitor of §7 reports it,
and Principle 2 is satisfied by disclosure. In this regime RK4 is not a
compromise; it is the right tool.

The long-integration regime is different in kind. VISION Future
Direction 4 — the precession of the equinoxes, some ten million rotations
— runs long enough that a secular error, however small per step, grows
until it overwhelms the very effect being measured. The decisive point is
that **no reduction of `dt` cures a secular error.** A smaller step lowers
the error at any fixed time, but the growth is unbounded, so for a long
enough run it always wins. This is why ARCHITECTURE §8.4 insists the two
regimes are different requirements rather than two tolerances: the fix is
not a smaller step but a different structural property of the integrator.

The convergence test of §6.2 measures error over a short run and cannot
see this; a test for the long regime must measure the *growth* of the
error across many periods, per ARCHITECTURE §8.4. The two tests check two
different things, and passing the first says nothing about the second.

### 6.5 The structure-preserving path

What the long regime needs is a **symplectic** (structure-preserving)
integrator, whose conserved-quantity error stays bounded — oscillating
within a fixed envelope forever instead of drifting. This is designed for
now and built when Future Direction 4 is approached, mirroring the
"design now, build later" stance ARCHITECTURE §2 takes toward the batch
tier. The selectable-strategy interface of §6.1 is what lets it drop in
without disturbing anything upstream.

Two candidate schemes are recorded, both standard in geometric
integration:

- **The implicit midpoint rule.** A second-order symplectic method that
  preserves every *quadratic* first integral of the motion exactly, to
  round-off. This property is unusually well matched to this problem.
  `|q|^2 = 1` is quadratic, so the orientation stays on the unit sphere
  with **no renormalization at all** — and applying §6.3's projection
  here would actively damage the bounded-error behavior, which is the
  entanglement §6.3 flagged. For torque-free motion the energy and the
  squared angular momentum `|L|^2` are also quadratic in the state, so
  both are preserved to round-off rather than merely bounded. The cost is
  that the method is implicit: each step solves a nonlinear equation by
  iteration, far more expensive than RK4's four explicit evaluations,
  which is acceptable in the batch tier and is why it is not the
  interactive default.
- **A splitting integrator for the free rigid body.** The rotational
  motion is split into rotations about the three principal axes, each of
  which is exactly solvable on its own, and these are composed in a
  symmetric sequence. The result is explicit and symplectic and preserves
  the angular-momentum magnitude by construction. It is the natural
  choice where the implicit solve is too costly.

A determinism caution attaches to any implicit scheme (ARCHITECTURE
§6.4). Its per-step iteration must stop on a fixed, deterministic
criterion — a fixed iteration count, or a convergence threshold checked
in a fixed order — so that the same scenario yields the identical
trajectory every time. This is the §5.6 concern about floating-point
reproducibility reappearing inside the integrator rather than in the
torque sum.

Finally, symplecticity is a property of *conservative* systems, which
couples the integrator choice back to the torque models of §5. Torque-free
motion (§5.2) and the heavy top under gravity (§5.3) are conservative and
are what the symplectic path is for. Viscous damping (§5.4) is
dissipative: it removes energy deliberately, so there is no symplectic
structure left to preserve and a symplectic integrator confers no benefit.
For dissipative scenarios the drift that matters is not departure from
constancy but departure from the *rate* §4.6 predicts, `d(energy)/dt =
Gamma . omega`, and RK4 measured against that rate is the appropriate
tool. The scenario selects an integrator suited to its physics, which is
the whole reason §6.1 makes the integrator a selectable strategy.

---

## 7. Conservation Monitoring

VISION Principle 2 is a promise made to the student: at any moment they
can tell whether what they see on screen is physics or numerical
artifact. The conservation monitor is the component that keeps that
promise. It is the on-screen face of the principle, and almost everything
it needs was already derived — §2.5 identified the two quantities to
watch, and §4.6 gave the exact laws they obey. This section is therefore
mostly about *how the monitor presents* what those sections established,
and about one design decision that makes it work under applied torque.

### 7.1 A live diagnostic, not a test fixture

The monitor is a first-class runtime component (ARCHITECTURE §3.4), not a
helper that lives only in `tests/`. Its output is displayed live,
alongside the motion, every frame. The oracles of ARCHITECTURE §8.2 use
the same conserved quantities to *judge* the integrator offline; the
monitor's job is different — to let a viewer judge it in real time, while
they watch.

It watches the two quantities §2.5 computes from the state: the
`kinetic_energy` and the space-frame `angular_momentum_space`. Both are
derived from the state rather than stored in it, so the monitor never
compares the state against a private copy that could itself be stale — it
recomputes from the seven numbers the integrator just advanced.

### 7.2 The residual: checking the balance, not constancy

The obvious monitor watches for constancy: hold the initial energy, and
report how far the current energy has moved from it. That works for
torque-free motion and fails exactly when it is needed most. The instant
gravity is switched on for the heavy top (§5.3), energy and angular
momentum are *supposed* to change, and a constancy check would light up
with physics it has mistaken for error — blind precisely during the
demonstration (VISION Goal 3) it should be supervising.

§4.6 gives the way out. Energy and angular momentum are not constant under
torque, but they obey exact rate laws whether or not a torque acts:

```
d(kinetic_energy)/dt          = Gamma . omega
d(angular_momentum_space)/dt  = Gamma_space
```

So the monitor checks the *balance*, not the value. It maintains a running
accumulator of the change these laws predict, and compares it against the
change actually observed in the state. The difference is the residual:

```
energy_residual   = ( kinetic_energy(t) - kinetic_energy(0) )
                    - integral over [0, t] of ( Gamma . omega ) dt

momentum_residual = ( angular_momentum_space(t)
                      - angular_momentum_space(0) )
                    - integral over [0, t] of Gamma_space dt
```

In exact arithmetic both residuals are identically zero, for any torque.
Their departure from zero is integration error and nothing else — which is
what §4.6 meant by calling this discrepancy, not the raw drift, the honest
measure of error under applied torque. With `Gamma = 0` the accumulators
vanish and the residual reduces to plain drift from the initial value, so
torque-free motion is not a special case here any more than it was in §5.2.

One honesty note about the accumulator. The predicted-change integrals are
themselves computed by quadrature over the same steps the integrator
takes, so they carry their own discretization error, of the same order as
the integrator (§6.2). The residual therefore measures the *mutual
inconsistency* between the advanced state and the delivered power and
torque impulse, which is exactly the thing that ought to be zero and is
the quantity Principle 2 wants surfaced. It is not represented as an
exact ground truth it cannot be.

### 7.3 Reporting: relative, and per unit time

Raw residuals are not directly interpretable — a large machine spinning
fast has a large energy, so a large absolute energy residual may be
excellent and a small one on a slow body may be poor. Two normalizations
make the number meaningful, both matching ARCHITECTURE §8.4.

- **Relative to a fixed scale.** The energy residual is divided by a
  characteristic energy, and the momentum residual by a characteristic
  `|L|`, so the reported drift is dimensionless. The scale is fixed once,
  at the start of the run, and held. It is tempting to divide by the
  *instantaneous* energy instead, but a body spinning down under damping
  (§5.4) has energy approaching zero, which would send a well-behaved
  residual to infinity. A fixed reference scale keeps the reported number
  interpretable for the whole run.
- **Per unit simulated time.** ARCHITECTURE §8.4 requires drift bounds to
  be stated as a rate, so that they stay meaningful as run length changes.
  The monitor divides accordingly and reports drift per unit of simulated
  time, not a raw total that would grow simply because the run was long.

Because `angular_momentum_space` is a vector, its residual carries more
information than a scalar drift, and the monitor separates the two parts.
The **magnitude** drift asks whether `|L|` is holding; the **direction**
drift asks whether the vector is holding its orientation in space. The
second is a particularly sharp tell: under torque-free motion the
space-frame angular momentum defines the fixed invariable axis that the
Poinsot construction (§9) is built on, so any wandering of its direction
is visibly non-physical, and it is worth showing on its own.

### 7.4 Watching the trend, not only the value

The instantaneous residual answers the classroom question — is this
effect real? The long-integration regime of §6.4 and ARCHITECTURE §8.4
poses a different question that a single current value cannot answer.

There, the danger is a *secular* residual: one small enough to look
acceptable at every instant while growing without bound over the run,
until it overwhelms the weak physical effect being measured (VISION Future
Direction 4). A monitor that reported only the current value could show a
reassuring small number the whole way and still be tracking a drift that
has already swamped the signal. The monitor therefore also reports the
*trend* of the residual — its growth across many periods — which is the
signature that distinguishes a bounded, symplectic error (§6.5) from a
growing, non-symplectic one. This is the live counterpart of the
long-regime test ARCHITECTURE §8.4 specifies: the test measures growth
offline, the monitor makes the same growth visible while the run proceeds.

### 7.5 It reports; it never corrects

The monitor is strictly read-only with respect to the state. It computes
residuals from the seven numbers and displays them, and it never writes
back. In particular it does **not** rescale the angular velocity to
restore the initial energy, nor project the state onto a constant-`|L|`
surface, though both are known techniques for suppressing drift.

They are refused here for two reasons, and the first is decisive. A tool
that quietly corrected the conserved quantities would be manufacturing the
very constancy it claims to verify — disguising the numerical error rather
than disclosing it, which is the exact inversion of VISION Principle 2. A
student would see energy pinned flat and conclude the integration was
perfect, when the flatness was imposed by hand. The second reason is
mechanical: writing back into the state would make the trajectory depend
on the monitor, so turning the display on or off would change the physics,
breaking the determinism guarantee of ARCHITECTURE §6.4.

The one legitimate correction in the whole loop is the quaternion
renormalization of §6.3, and it is legitimate precisely because it
restores a constraint the representation requires — a unit quaternion *is*
a rotation — rather than papering over an error in the physics. The line
is between enforcing a definition and faking a result.

### 7.6 An identity is not a diagnostic

One relation looks like a conservation check and is not. The Poinsot
identity `2T = omega . L` holds at **every** instant, torque or no torque,
because `L = I omega` makes `omega . L = omega . (I omega)` equal to twice
the kinetic energy by definition (§2.5). It is not a law the motion can
violate; it is an algebraic identity among quantities the code computes
three different ways.

That makes it useless as a live drift monitor — it would read zero even on
a hopelessly wrong trajectory — but valuable as a *code-consistency*
oracle in the test suite, where it catches a `kinetic_energy` or an
`angular_momentum_body` that was computed inconsistently with the state.
ARCHITECTURE §8.2 lists it among the Poinsot invariants for exactly that
reason. Keeping the distinction clear prevents an identity from being
mistaken for a measurement and wired into the monitor of §7.1, where it
would report a reassuring zero that means nothing.

---

## 8. Analytic Solutions

A handful of rigid-body motions can be solved in closed form. VISION
Principle 3 makes them the standard against which the integrator is
judged, and VISION Goal 9 asks that they be drawn on screen beside the
numerical motion so a student sees theory and simulation agree. This
section writes those solutions down concretely, because a formula that is
merely gestured at cannot become either an overlay or a test.

Several were already quoted where they were needed — the symmetric-top
precession rate is implied by §4.3, the instability growth rate was
derived in §4.4, and the heavy-top precession rate was stated in §5.3.
Here they are collected, completed, and given the exact forms the code
will use.

### 8.1 One module, two duties

`analytic_solutions.py` (ARCHITECTURE §3.4) serves two purposes from a
single set of formulas, which is why it is a runtime component and not a
test-only helper.

- **As an overlay** (VISION Goal 9). Where a closed form exists, the tool
  draws it alongside the numerically integrated motion. Agreement is
  reassurance; divergence is a lesson (§8.6).
- **As an oracle** (VISION Principle 3, ARCHITECTURE §8.2). The same
  formulas are the yardstick the regression suite measures the integrator
  against, and the known-solution cases the convergence study of §6.2
  needs.

To serve both, each analytic solution produces a trajectory in exactly
the form the simulation engine emits — the state of §2.1 as a function of
time — so the renderer and the test harness treat analytic and numerical
motion identically, with no special path for either.

The closed forms cover only special cases. A general asymmetric heavy top
has no analytic solution at all, and §8.6 explains why the tool says so
plainly rather than papering over the gap.

### 8.2 Steady rotation about a principal axis

The simplest solution, and the first oracle of ARCHITECTURE §8.2: a body
set spinning about a single principal axis stays there. §4.3 showed the
gyroscopic term vanishes, so `omega` is constant and the body turns
uniformly about that fixed axis. Trivial as it is, it is a sharp test —
any leak of `omega` onto the other two axes is pure numerical error,
since the physics forbids it exactly.

### 8.3 The torque-free symmetric top

For a symmetric top, `I_1 = I_2 != I_3`, under no torque, the motion is a
steady precession with two distinct rates — one seen in the body, one
seen in space — and telling them apart is exactly the frame lesson of
VISION Goal 5 (§10):

```
Body frame:  omega_3 = const, and the pair (omega_1, omega_2)
             rotates about body axis 3 at the body precession rate
                 Omega_body = omega_3 * (I_3 - I_1) / I_1

Space frame: the symmetry axis holds a fixed angle theta to the
             constant angular momentum L and precesses about it at
                 phi_dot = |L| / I_1
```

Both rates are exact and independent of amplitude, which makes them among
the cleanest oracles available: a simulated symmetric top must show
`omega` circling body axis 3 at `Omega_body` and its figure axis circling
`L` at `phi_dot`, and any error in either rate is measurable directly
against these expressions. `Omega_body` is the rate ARCHITECTURE §8.2
names; `phi_dot` is what the overlay draws in the space frame.

The sign of `Omega_body` carries physics worth noting: it is positive for
a prolate body (`I_3 < I_1`, a rod-like top) and negative for an oblate
one (`I_3 > I_1`, a disk-like top such as the Earth of Goal 12), so the
body-frame precession runs the opposite way for the two shapes. The
Earth's oblateness is what makes its free precession retrograde in the
body frame.

### 8.4 The torque-free asymmetric top

When all three moments differ there is still an exact solution, in Jacobi
elliptic functions rather than sines and cosines. It is the sharpest
possible oracle for the fully general free motion — the Dzhanibekov case
of VISION Goal 2 — and it completes the linear picture of §4.4.

Order the moments `I_1 < I_2 < I_3`. The motion is fixed by the two
conserved quantities `2T` and `|L|^2` (§2.5). For the case
`|L|^2 > 2*T*I_2`, where the body spins nearer axis 3, the body-frame
angular velocity is

```
omega_1 = sqrt( (2*T*I_3 - |L|^2) / (I_1*(I_3 - I_1)) ) * cn(tau, k)
omega_2 = sqrt( (2*T*I_3 - |L|^2) / (I_2*(I_3 - I_2)) ) * sn(tau, k)
omega_3 = sqrt( (|L|^2 - 2*T*I_1) / (I_3*(I_3 - I_1)) ) * dn(tau, k)

tau = t * sqrt( (I_3 - I_2)*(|L|^2 - 2*T*I_1) / (I_1*I_2*I_3) )

k^2 = ( (I_2 - I_1) * (2*T*I_3 - |L|^2) )
      / ( (I_3 - I_2) * (|L|^2 - 2*T*I_1) )
```

The complementary case `|L|^2 < 2*T*I_2` is the same solution with axes 1
and 3 exchanged: the body then spins nearer axis 1, and `omega_1` takes
the `dn` role while `omega_3` takes the `cn` role. The body-frame period
is `4*K(k)` in `tau`, with `K` the complete elliptic integral of the
first kind, so the tumbling slows as `k` approaches one.

The dividing case `|L|^2 = 2*T*I_2` is the whole point. There `k = 1`, the
elliptic functions degenerate to hyperbolic ones (`cn -> sech`,
`sn -> tanh`, `dn -> sech`), and `K(1)` diverges, so the period becomes
infinite. This is the separatrix: the motion started near the
intermediate axis creeps toward the unstable equilibrium, lingers, then
flips — the Dzhanibekov flip is a near-separatrix orbit, and its
"hang, then flip suddenly" character is `K(k)` growing without bound as
`k -> 1`. The exponential growth rate `sigma` of §4.4 is the same
separatrix's departure exponent, so the flip time `~ (1/sigma) *
ln(1/eps_0)` estimated there is the near-separatrix limit of this exact
period. The linear result of §4.4 and this nonlinear one are two views of
one orbit.

**These formulas were recalled, not derived here, and were validated
numerically before being trusted** — the same discipline §3.2 records for
the Platonic constants, where a remembered value was wrong by exactly a
factor of two. The spike `dev/spikes/free_top_elliptic.py` evaluates both
branches and compares them against a high-accuracy integration of the same
initial condition: the two agree to about `1e-13`, the integrator's own
noise floor, and the analytic solution conserves `2T` and `|L|^2` to
`1e-14` on its own. The connections above (the `k -> 1` separatrix, the
period `4*K(k)`, the reduction to §4.4) were cross-checks as much as
claims — a transcription error would have broken at least one — and the
same spike confirms the §4.4 growth rate `sigma` as the separatrix
exponent to `1e-4`. Only the verified forms are recorded here.

### 8.5 Steady precession of the heavy symmetric top

The heavy symmetric top of VISION Goal 3 has no closed form for a general
initial condition — the nutation is elliptic, like §8.4 — but its
*steady-precession* solutions, in which the figure axis sweeps a cone at a
fixed nutation angle `theta_0`, are exact and are what the overlay draws.

At a fixed `theta_0`, a steady precession rate `phi_dot` must satisfy

```
I_1*cos(theta_0)*phi_dot^2 - I_3*omega_3*phi_dot + M*g*l = 0
```

with `l` the pivot-to-center-of-mass distance of §5.3. This quadratic has
two roots — the top can precess slow or fast at the same tilt:

```
Slow root (fast-spinning top):  phi_dot ~ M*g*l / (I_3*omega_3)
Fast root:                      phi_dot ~ I_3*omega_3 / (I_1*cos(theta_0))
```

The slow root is the familiar one, and it is exactly the
`precession_rate` §5.3 quoted. The quadratic also states the condition for
steady precession to exist at all: the discriminant is non-negative only
when

```
(I_3*omega_3)^2 >= 4*I_1*M*g*l*cos(theta_0)
```

that is, only when the top spins fast enough. Below that threshold there
is no steady precession at the given tilt, and the motion necessarily
nutates. This threshold is itself an oracle — a demonstration that starts
just above and just below it should look qualitatively different — and it
is the quantitative form of "spin it faster to make it steady."

### 8.6 Where the overlay stops, and why that is honest

The general asymmetric heavy top, and the general nutating symmetric top,
have no closed-form trajectory. For those cases there is simply nothing to
overlay, and the tool draws nothing rather than inventing an
approximation and presenting it as theory. An absent overlay is the honest
statement that no analytic result exists there.

Where an overlay *is* drawn, its divergence from the numerical motion is
not a failure to hide but the very thing VISION Principle 2 and Future
Direction 4 are about. In the classroom regime the two curves sit on top
of each other and the message is "the simulation is trustworthy." In the
weak-torque, long-time regime of §6.4 they slowly part, and that parting
is the visible signature of the secular integration error — the case where
a real physical effect and the numerical error tracking it are the same
size. The overlay is what lets a student see which is which, which is the
whole reason Goal 9 pairs the analytic curve with the numerical one rather
than trusting either alone.
