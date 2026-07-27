# Pseudocode

> **Document hierarchy:** VISION → ARCHITECTURE → DESIGN → **PSEUDOCODE**
> → Code. For the design rationale behind these algorithms, see
> `DESIGN.md`; this document assumes that rationale and does not repeat it.

---

## How to read this document

DESIGN.md argues *why* each algorithm is shaped the way it is. This
document states *what* the algorithm does, in language-agnostic
pseudocode, so that the eventual source in `src/` is a transcription
rather than an invention. Every section cites the DESIGN section it
realizes, so a reader can move up the chain for the reasoning and down to
the code for the implementation.

Not every DESIGN section appears here. The conventions of DESIGN §1 and
the classification of §3.3 are conceptual — they fix meaning and shape
thinking but contain no algorithm to transcribe — so they have no
pseudocode. Where a DESIGN section *is* an algorithm, it is specified
below.

Like DESIGN, this document is drafted section by section; the Contents
table tracks the status of each.

## Notation and conventions

The pseudocode is deliberately close to what the source will look like,
and it reuses the DESIGN §1.4 code names verbatim (`principal_moments`,
`angular_velocity_body`, `body_to_space_quaternion`, and so on) so that a
name means the same thing at every level of the chain.

```
<-              assignment
=               equality test
#               comment to end of line
a.b             field b of record a
f(x): ...       function definition; body is the indented block
return v        the function's result (DESIGN speaks of this as the
                value; there is no hidden output)
for x in xs     iterate in the given (fixed) order
repeat n times  iterate a counted number of times
loop forever    non-terminating loop (the interactive frame loop)
```

Vectors are written as tuples in the body principal-axis frame unless a
name says otherwise (`..._space` for the space frame). Helper operations
used throughout, each a pure function of its arguments:

```
dot(a, b)            scalar (inner) product of two 3-vectors
cross(a, b)          vector (cross) product of two 3-vectors
norm(v)              Euclidean length of a vector
quat_multiply(a, b)  quaternion product, scalar-first (DESIGN 2.2)
quat_conjugate(q)    quaternion conjugate
pure_quat(v)         the quaternion (0, v_x, v_y, v_z) from a 3-vector
```

The seven-number state of DESIGN §2.1 is carried as a record with two
fields, never as a bare array in the pseudocode, so the intent stays
legible:

```
state.body_to_space_quaternion   # 4 numbers, scalar-first, unit norm
state.angular_velocity_body      # 3 numbers, body principal axes
```

The flat-array layout DESIGN §2.1 reserves for a future compiled kernel
is an implementation detail of the code level, not of this one.

---

## Contents

| Section | Topic | DESIGN source | Status |
| --- | --- | --- | --- |
| 1 | The simulation loop | §4.2, §6, §7, §12, §14 | written |
| 2 | Orientation mathematics | §1.3, §2.2, §2.4, §2.6 | written |
| 3 | State and derived quantities | §2.1, §2.5 | written |
| 4 | Inertia and body construction | §3 | written |
| 5 | Equations of motion | §4 | written |
| 6 | Torque models | §5 | written |
| 7 | Integrators | §6 | not yet written |
| 8 | Conservation monitor | §7 | not yet written |
| 9 | Analytic solutions | §8 | not yet written |
| 10 | Poinsot geometry | §9 | not yet written |
| 11 | Reference frames | §10 | not yet written |
| 12 | Scenario load and save | §11 | not yet written |
| 13 | Trajectory retention | §12 | not yet written |
| 14 | Scene description | §13 | not yet written |
| 15 | Controls and time | §14 | not yet written |

---

## 1. The Simulation Loop

This is the spine of the program: the loop that advances the physics and
hands each state to whatever is watching. DESIGN §4.2 fixed the derivative
it steps, §6 the integrator that steps it, §7 the monitor that watches,
and §12 the buffer that remembers; ARCHITECTURE §6 fixed the pacing and
the determinism guarantee. This section shows how they fit together, and
specifies each named subroutine no further than its signature — the
sections that follow fill those in.

The one theme to keep in view is ARCHITECTURE §6.4: the loop is arranged
so that the computed trajectory depends on the scenario alone. Nothing the
monitor, the buffer, the renderer, or the controls do may feed back into a
future state.

### 1.1 One substep: the engine's atomic action

The smallest unit of progress is a single fixed step of the integrator.
The engine builds the pure derivative closure of DESIGN §4.2, hands it to
the selected integrator (DESIGN §6.1), and returns the advanced state.

```
function advance_one_substep(state, time, dt, body, torque_models,
                             integrator):
    # A pure function of (time, state): the derivative of the motion.
    # Closes over the fixed body and the ordered torque list so the
    # integrator can evaluate it at trial points (DESIGN 4.2, 5.1).
    derivative <- function(t, s):
        return state_derivative(t, s, body, torque_models)   # section 5

    # One fixed step. RK4 by default, a structure-preserving scheme
    # for the long regime (DESIGN 6). The integrator renormalizes the
    # quaternion once, after the whole step (DESIGN 6.3).
    return integrator.advance(state, time, dt, derivative)   # section 7
```

Two notes tie this back to the design. The derivative *must* be pure
(DESIGN §4.2, §5.1) because the integrator evaluates it at trial states
off the trajectory; the closure above captures only immutable data, so
purity holds. And the signature threads `time` — a small refinement of the
`advance(state, dt, derivative)` written in DESIGN §6.1 — because a
time-dependent torque (DESIGN §5.7) needs the absolute time at each
trial point; for the torque-free and gravity cases the argument is simply
unused.

### 1.2 The interactive frame loop

The interactive tier (ARCHITECTURE §6.1) runs its own loop: it advances
the physics by a number of substeps the *time controls* choose, refreshes
the on-screen picture, and pumps the windowing system's events — all on a
single thread. The substep count, not a wall clock, sets the pace
(ARCHITECTURE §6.2).

```
function run_interactive(scenario):
    state      <- resolve_initial_state(scenario)      # section 12
    time       <- 0
    dt         <- scenario.fidelity.time_step          # fixed (ARCH 6.2)
    body       <- build_body(scenario)                 # section 4
    torques    <- scenario.torque_models               # fixed order 5.6
    integrator <- select_integrator(scenario)          # section 7
    monitor    <- new_monitor(state, body, torques)    # section 8
    history    <- new_trajectory(scenario.retention)   # section 13
    sinks      <- attach_sinks(scenario)               # section 13

    loop forever:
        controls <- read_controls()                    # section 15

        if controls.mode = REPLAY:
            # Replay reads stored history; it never steps the engine
            # (DESIGN 12.4, 14.4). A read cannot perturb a state.
            state, time <- history.read(controls.replay_cursor)
        else:
            # Live stepping. Pause yields zero substeps; slow motion
            # fewer, fast forward more (ARCHITECTURE 6.3).
            n <- substeps_this_frame(controls)          # section 15
            repeat n times:
                state <- advance_one_substep(state, time, dt, body,
                                             torques, integrator)
                time  <- time + dt
                monitor.update(state, time)             # read-only 8
                history.append(state, time)             # retention 13
                emit(sinks, state, time)                # sink boundary

        scene <- build_scene(state, body, monitor)      # section 14
        render(scene)                                   # render/ only
        pump_events()                                   # windowing
```

Everything after the physics — `monitor.update`, `history.append`,
`emit`, `build_scene`, `render` — only *reads* the state. That is the
loop-level form of the read-only discipline the monitor (DESIGN §7.5) and
retention (DESIGN §12.2) require, and it is what keeps the determinism
guarantee true: turning the display, the monitor, or replay on or off
changes nothing the next `advance_one_substep` computes.

### 1.3 The batch loop

The batch tier (ARCHITECTURE §6.1) is the *same* engine step driven by a
plainer loop: no time controls, no renderer, no event pump. It runs for a
fixed simulated span and writes every state to a recording sink.

```
function run_batch(scenario):
    state      <- resolve_initial_state(scenario)      # section 12
    time       <- 0
    dt         <- scenario.fidelity.time_step
    body       <- build_body(scenario)
    torques    <- scenario.torque_models
    integrator <- select_integrator(scenario)
    monitor    <- new_monitor(state, body, torques)
    sinks      <- attach_sinks(scenario)               # includes hdf5

    while time < scenario.fidelity.integration_span:
        state <- advance_one_substep(state, time, dt, body,
                                     torques, integrator)
        time  <- time + dt
        monitor.update(state, time)
        emit(sinks, state, time)

    finalize(sinks)                                    # flush, close
```

The two loops share `advance_one_substep` verbatim, which is exactly why
ARCHITECTURE §2 can promise the batch run reproduces the interactive one:
the physics is one routine, and only what consumes its output differs. A
batch run needs no in-memory `history` buffer, because its recording sink
already keeps every state on disk (DESIGN §12.6).

### 1.4 Why this is deterministic

Three properties of the loops above, together, give the bit-for-bit
guarantee of ARCHITECTURE §6.4, and each is visible in the pseudocode:

- **The step is fixed.** `dt` is read once from the scenario and never
  varies, so the sequence of states depends only on the scenario, not on
  how the run was paced.
- **Pacing is by count, not clock.** `substeps_this_frame` returns an
  integer chosen by the controls; no wall-clock time is ever read inside
  the physics. A slow machine takes the same steps, just fewer per second
  (ARCHITECTURE §6.2).
- **Consumers are read-only.** The monitor, the history buffer, the
  sinks, and the renderer receive the state but never return one into the
  loop. The only writer of `state` is `advance_one_substep`.

The determinism test of ARCHITECTURE §8.6 checks precisely this: run the
same scenario twice, and under different control sequences, and require
identical trajectories.

---

## 2. Orientation Mathematics

Orientation mathematics is the layer every other algorithm stands on: the
simulation loop (§1), the equations of motion (§5), the conservation
monitor (§8), and the renderer all rotate vectors or read out angles, and
they do it through the operations named here. DESIGN §2.2 fixed the
quaternion conventions, §2.4 the kinematic law, §1.3 the z-x-z Euler
sequence, and §2.6 the degeneracy the read-out must respect. This section
transcribes those into concrete routines; it invents nothing the design
did not already argue for.

### 2.1 Quaternion algebra

All orientation work reduces to a handful of quaternion operations. They
are stated once here, as pure functions, and used by name everywhere else.
Quaternions are **scalar-first** throughout (DESIGN §2.2): the four
components are `(q_w, q_x, q_y, q_z)`, the real part `q_w` first. Mixing
this with the scalar-last order common in other libraries yields a
well-formed quaternion for the wrong rotation, so any quaternion crossing a
library boundary is converted explicitly.

```
function quat_multiply(a, b):
    # Hamilton product, scalar-first. Not commutative: the order
    # encodes the order of the composed rotations (DESIGN 2.2, 2.4).
    (a_w, a_x, a_y, a_z) <- a
    (b_w, b_x, b_y, b_z) <- b
    return (a_w*b_w - a_x*b_x - a_y*b_y - a_z*b_z,   # w
            a_w*b_x + a_x*b_w + a_y*b_z - a_z*b_y,   # x
            a_w*b_y - a_x*b_z + a_y*b_w + a_z*b_x,   # y
            a_w*b_z + a_x*b_y - a_y*b_x + a_z*b_w)   # z

function quat_conjugate(q):
    (q_w, q_x, q_y, q_z) <- q
    return (q_w, -q_x, -q_y, -q_z)      # negate the vector part

function pure_quat(v):
    (v_1, v_2, v_3) <- v
    return (0, v_1, v_2, v_3)           # a 3-vector as a quaternion

function scale_quaternion(factor, q):
    (q_w, q_x, q_y, q_z) <- q
    return (factor*q_w, factor*q_x, factor*q_y, factor*q_z)

function quat_normalize(q):
    # Restore the unit-norm constraint that only unit quaternions
    # represent rotations (DESIGN 2.2).
    (q_w, q_x, q_y, q_z) <- q
    magnitude <- sqrt(q_w^2 + q_x^2 + q_y^2 + q_z^2)
    return scale_quaternion(1 / magnitude, q)
```

Normalization deliberately leaves the **sign** alone. Because `q` and `-q`
name the same physical rotation (the double cover, DESIGN §2.2), forcing a
"canonical" sign would be a silent rewrite of the state; nothing here does
that, so the trajectory comparison of the determinism test (ARCHITECTURE
§8.6) is never tricked by a spurious flip.

### 2.2 Rotating a vector, and the orientation matrix

A quaternion carries a vector from body components to space components by
the **sandwich product** of DESIGN §2.2. Two shapes of the same rotation
are provided because different consumers want different shapes: the
derivative and the monitor rotate one vector at a time (for instance
`angular_momentum_space` of DESIGN §2.5), while the renderer wants the
whole `body_to_space_matrix` (DESIGN §1.4) at once.

```
function rotate_body_to_space(quaternion, vector_body):
    # v_space = q * (0, v_body) * q_conjugate   (DESIGN 2.2).
    sandwich <- quat_multiply(
                    quat_multiply(quaternion, pure_quat(vector_body)),
                    quat_conjugate(quaternion))
    (result_w, result_x, result_y, result_z) <- sandwich
    return (result_x, result_y, result_z)   # vector part; w is 0

function quaternion_to_matrix(quaternion):
    # The body_to_space_matrix equivalent to the sandwich above, valid
    # for a UNIT quaternion (call quat_normalize first if in doubt).
    # Rows and columns are numbered from 0.
    (w, x, y, z) <- quaternion
    return [
        [1 - 2*(y^2 + z^2), 2*(x*y - w*z),     2*(x*z + w*y)    ],
        [2*(x*y + w*z),     1 - 2*(x^2 + z^2), 2*(y*z - w*x)    ],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x^2 + y^2)]]
```

### 2.3 The orientation derivative (kinematics)

The orientation half of the state derivative is the kinematic law of
DESIGN §2.4. This routine returns only `q_dot`; the angular-velocity half,
`omega_dot`, comes from Euler's equations in §5, and the two together form
the derivative the integrator steps (§1.1).

```
function orientation_derivative(quaternion, angular_velocity_body):
    # DESIGN 2.4:  q_dot = 0.5 * q * (0, omega_body)
    # omega is in BODY components, so its pure quaternion multiplies q
    # from the RIGHT. The space-frame form would multiply from the left;
    # pairing one frame's omega with the other form is the silent-inverse
    # error DESIGN 1.2 warns of.
    omega_quaternion <- pure_quat(angular_velocity_body)
    product          <- quat_multiply(quaternion, omega_quaternion)
    return scale_quaternion(0.5, product)
```

Note what this does **not** do: it never renormalizes. The step drifts the
quaternion off the unit sphere by a tiny amount, and the integrator
restores the constraint once, after the whole step (DESIGN §6.3) — not
here, inside a derivative that may be evaluated at several trial points.

### 2.4 Building a quaternion from authoring input

A scenario authors the initial orientation the way a student thinks about
it — as three Euler angles, or as a rotation about an axis (DESIGN §1.3,
§11) — and the loader turns that into the quaternion the state actually
carries. Both builders compose the elementary rotations below.

```
function rotation_quaternion_z(angle):
    half <- angle / 2
    return (cos(half), 0, 0, sin(half))    # rotation about z

function rotation_quaternion_x(angle):
    half <- angle / 2
    return (cos(half), sin(half), 0, 0)    # rotation about x

function quaternion_from_euler(precession_angle, nutation_angle,
                               spin_angle):
    # z-x-z sequence of DESIGN 1.3:
    #   R = R_z(phi) R_x(theta) R_z(psi)
    # The quaternion product mirrors the matrix product, same order.
    q_precession <- rotation_quaternion_z(precession_angle)
    q_nutation   <- rotation_quaternion_x(nutation_angle)
    q_spin       <- rotation_quaternion_z(spin_angle)
    return quat_multiply(q_precession,
                         quat_multiply(q_nutation, q_spin))

function quaternion_from_axis_angle(axis, angle):
    # A rotation of `angle` about a possibly unnormalized axis. The
    # 1/length folds into the vector-part scale so the result is unit.
    (axis_x, axis_y, axis_z) <- axis
    half  <- angle / 2
    scale <- sin(half) / norm(axis)
    return (cos(half), axis_x*scale, axis_y*scale, axis_z*scale)
```

### 2.5 Euler angles as an output

The three Euler angles are what the course text uses and what a student
expects to read, so they are recovered from the quaternion whenever the
display needs them (DESIGN §2.6). They are **output only** — computed from
the state, never fed back into it, so they cannot drift out of agreement
with the orientation they describe.

The recovery inherits the singularity of DESIGN §1.3. When `sin(theta)`
nears zero the precession and spin are individually undetermined and only
a combination of them survives; the read-out must report that combination
and mark the two angles degenerate, rather than print two racing numbers
as if they were measurements (VISION Principle 2, at the interface).

```
function euler_angles_from_quaternion(quaternion):
    R <- quaternion_to_matrix(quaternion)   # rows/cols numbered from 0

    # theta from the (2,2) entry (= cos theta); always well defined,
    # and clamped before arccos against roundoff past +/-1.
    nutation_angle <- arccos(clamp(R[2][2], -1, +1))
    sin_theta      <- sqrt(max(0, 1 - R[2][2]^2))

    if sin_theta > DEGENERACY_TOLERANCE:
        # Generic case: all three angles are separately determined.
        precession_angle <- atan2(R[0][2], -R[1][2])
        spin_angle       <- atan2(R[2][0],  R[2][1])
        return { precession = precession_angle,
                 nutation   = nutation_angle,
                 spin       = spin_angle,
                 degenerate = false }
    else:
        # Gimbal lock (DESIGN 1.3, 2.6): phi and psi collapse into one
        # combination. Report it; flag the pair degenerate.
        combination <- atan2(R[1][0], R[0][0])
        # theta ~ 0  -> combination = phi + psi
        # theta ~ pi -> combination = phi - psi  (sign of cos theta)
        return { precession = combination,   # phi (+/-) psi
                 nutation   = nutation_angle,
                 spin       = none,
                 degenerate = true }
```

Every index above follows from writing out `R = R_z(phi) R_x(theta)
R_z(psi)` by hand (numbering rows and columns from 0):

```
R[2][2] = cos(theta)
R[0][2] =  sin(phi) sin(theta)     R[1][2] = -cos(phi) sin(theta)
R[2][0] =  sin(theta) sin(psi)     R[2][1] =  sin(theta) cos(psi)
R[0][0], R[1][0]  ->  cos, sin of (phi+psi) at theta = 0
                      cos, sin of (phi-psi) at theta = pi
```

Dividing each paired entry cancels the shared `sin(theta)` and leaves the
`atan2` forms above. When `sin(theta)` vanishes those pairs collapse to
zero and carry no information; only `R[1][0]` and `R[0][0]` still vary, and
they encode a single combination — which is exactly the coordinate
singularity DESIGN §1.3 forbids the integrator to touch, surfacing here as
an honest label instead of numerical noise.

---

## 3. State and Derived Quantities

DESIGN §2.1 fixes what the integrator advances; DESIGN §2.5 fixes what is
read back off it. This section states the state record and the pure
functions that compute the derived quantities, keeping the two firmly
apart: the state is the small thing that is stepped and stored, and
everything else is recomputed from it on demand, so the two can never fall
out of agreement.

### 3.1 The integrated state

The state is the seven numbers of DESIGN §2.1, carried as the two-field
record the notation section introduced. Both fields are bare SI floats:
units live only at the scenario boundary (ARCHITECTURE §5.5), and nothing
at this level holds a unit object.

```
record State:
    body_to_space_quaternion    # 4 numbers, scalar-first, unit norm
    angular_velocity_body       # 3 numbers, body principal axes
```

The unit-norm invariant on the quaternion is established when the state is
built from authoring input (the §2.4 builders already return unit
quaternions) and maintained thereafter by the integrator's per-step
renormalization (§2.3, DESIGN §6.3).

**Translation is not in the state** (DESIGN §2.1). For a freely tumbling
body the center of mass drifts uniformly and decouples from the rotation;
for the heavy top the pivot is fixed. Rotation is the whole subject, so the
state carries orientation and angular velocity and nothing else. The
flat seven-number array DESIGN §2.1 reserves for a future compiled kernel
is a code-level packing detail and is not modeled here.

### 3.2 Derived quantities

These are computed from the state and the body, never stored alongside
them, so they cannot silently disagree with the state they summarize
(DESIGN §2.5). Each is a pure function. The `principal_moments` they read
belong to the body constructed in §4.

```
function angular_momentum_body(state, body):
    # Componentwise because the body frame IS the principal-axis frame
    # (DESIGN 1.1, 2.5), where the inertia tensor is diagonal.
    (omega_1, omega_2, omega_3) <- state.angular_velocity_body
    (I_1, I_2, I_3)             <- body.principal_moments
    return (I_1*omega_1, I_2*omega_2, I_3*omega_3)

function angular_momentum_space(state, body):
    # The SAME vector in space components. DESIGN 2.5 writes this as
    # body_to_space_matrix * L_body; rotating the single vector by the
    # sandwich (2.2) is the identical result without forming the matrix.
    momentum_body <- angular_momentum_body(state, body)
    return rotate_body_to_space(state.body_to_space_quaternion,
                                momentum_body)

function kinetic_energy(state, body):
    (omega_1, omega_2, omega_3) <- state.angular_velocity_body
    (I_1, I_2, I_3)             <- body.principal_moments
    return 0.5 * (I_1*omega_1^2 + I_2*omega_2^2 + I_3*omega_3^2)
```

### 3.3 The invariants the motion preserves

Under **torque-free** motion two of the quantities above are constant, and
this is the fact the rest of the program leans on (DESIGN §2.5):

- `angular_momentum_space` is a fixed vector. Note it is the *space-frame*
  form that is conserved: the body-frame components `angular_momentum_body`
  change continuously as the body tumbles beneath the fixed vector, and
  that very difference between the two frames is something worth showing a
  student (§11, DESIGN §10).
- `kinetic_energy` is constant.

Because rotation preserves length, `norm(angular_momentum_body)` and
`norm(angular_momentum_space)` are equal and are themselves conserved; the
magnitude of the angular momentum is frame-independent. These two
invariants are what the conservation monitor watches (§8), the two numbers
the Poinsot construction is built from (§10), and among the sharpest
oracles the test suite has (ARCHITECTURE §8.2). When a torque is present
they are no longer conserved, and the monitor reports drift rather than
asserting constancy.

---

## 4. Inertia and Body Construction

This section builds the body: it turns a shape, or three directly given
numbers, into the immutable record the rest of the program uses. DESIGN §3
is its source throughout. The governing idea is DESIGN §3.8 — everything
here is computed **once**, at construction, and never recomputed while the
simulation runs, because in the body frame the inertia tensor is constant
by definition, and that constancy is exactly what "rigid" means. After
this section runs, the downstream code sees three moments and their axes
and never asks where they came from (ARCHITECTURE §5.1).

### 4.1 The body record

`build_body` (§4.7) returns the record `body` that §1 threads through the
loop. It holds only quantities fixed in the body frame.

```
record Body:
    principal_moments     # (I_1, I_2, I_3), the diagonal inertia
    principal_axes        # 3 columns: body axes in build coordinates
    total_mass            # M
    center_of_mass        # in build coordinates
    top_class             # SPHERICAL | SYMMETRIC | ASYMMETRIC (4.6)
    intermediate_axis     # index 1..3 for an asymmetric top, else none
    geometry              # drawable shape, or none for direct entry
```

### 4.2 Physical validity: the three assertions

Every inertia tensor a physical body can have obeys three properties
(DESIGN §3.1). They are checked wherever a tensor or a set of moments is
produced, because a violation means the numbers are not an inertia tensor
at all.

```
function validate_inertia_tensor(tensor):
    # Symmetry is checked on the FULL tensor, before it is diagonalized
    # (DESIGN 3.1). The other two properties are checked on the moments
    # that diagonalization produces, by validate_moments below.
    assert is_symmetric(tensor)

function validate_moments(moments):
    (I_1, I_2, I_3) <- moments
    # Positive definiteness: every principal moment is strictly > 0.
    if not (I_1 > 0 and I_2 > 0 and I_3 > 0):
        reject("a principal moment is not positive: " + moments)
    # Triangle inequalities. Impossible to violate in nature, easy to
    # mistype (DESIGN 3.1, 3.6): report WHICH fails and BY HOW MUCH,
    # rather than integrate nonsense.
    for (a, b, c) in [(I_1, I_2, I_3), (I_2, I_3, I_1),
                      (I_3, I_1, I_2)]:
        if a + b < c:
            reject("triangle inequality fails: " + a + " + " + b
                   + " < " + c + ", short by " + (c - a - b))
```

### 4.3 Closed forms for the primitives, and the provider seam

For a uniform primitive the inertia tensor is a closed form (DESIGN §3.2).
`analytic_inertia` computes it; a future `numerical_inertia` will integrate
a density field instead. Both return the **same** triple, so no consumer
can tell which ran — the provider seam of ARCHITECTURE §5.1.

```
function inertia_provider(shape, density):
    # The one operation of ARCHITECTURE 5.1: shape + density -> the
    # triple below. analytic_inertia is the closed-form implementation.
    return { mass, center_of_mass, inertia_tensor }

function analytic_inertia(shape, density):
    # For a primitive in its natural orientation the tensor is DIAGONAL,
    # so §4.4 solves no eigenproblem in the common case (DESIGN 3.4).
    mass           <- density * volume_of(shape)
    center_of_mass <- centroid_of(shape)      # e.g. h/4 up a cone (3.2)
    moments        <- principal_moments_of(shape, mass)
    return { mass           = mass,
             center_of_mass = center_of_mass,
             inertia_tensor = diagonal_matrix(moments) }

function principal_moments_of(shape, mass):
    # The closed forms of DESIGN 3.2, keyed by primitive. A few are
    # shown to fix the pattern; the COMPLETE, numerically-verified table
    # is DESIGN 3.2 and is the single source of truth for the constants.
    if shape is Sphere(a):
        m <- (2/5)*mass*a^2
        return (m, m, m)
    if shape is Ellipsoid(a, b, c):
        return ( (mass/5)*(b^2 + c^2),
                 (mass/5)*(c^2 + a^2),
                 (mass/5)*(a^2 + b^2) )
    if shape is Cylinder(a, h):               # symmetry axis is 3
        return ( (mass/12)*(3*a^2 + h^2),
                 (mass/12)*(3*a^2 + h^2),
                 (1/2)*mass*a^2 )
    # ... box, cone, and the five Platonic solids: see DESIGN 3.2.
    # Every Platonic solid is isotropic, I_1 = I_2 = I_3 (DESIGN 3.3),
    # which is why a cube tumbles exactly as a sphere does.
```

### 4.4 Principal axes: diagonalization done once

The body frame is the principal-axis frame (DESIGN §1.1), so the tensor
must be made diagonal. The common case is free; the general case solves one
symmetric eigenproblem, with two corrections DESIGN §3.4 requires.

```
function principal_frame_of(inertia_tensor, geometry):
    if is_diagonal(inertia_tensor):
        # Primitive in natural orientation: axes are the coordinate
        # axes, no eigenproblem, no numerical error (DESIGN 3.4).
        return { moments = diagonal_of(inertia_tensor),
                 axes    = identity_axes }

    (moments, axes) <- symmetric_eigensolver(inertia_tensor)

    # Correction 1 -- handedness (DESIGN 3.4). A symmetric solver may
    # return a left-handed set; a reflection is not a rotation, which is
    # the silent-inverse failure of 1.2 in disguise.
    if determinant(axes) < 0:
        axes <- negate_one_column(axes)

    # Correction 2 -- degenerate subspaces (DESIGN 3.4). When moments
    # coincide the axes are not unique and a solver returns an arbitrary
    # basis that can jump between calls. Pin the degenerate subspace to
    # the geometric symmetry axis so nothing jitters for no reason.
    axes <- canonicalize_degenerate_axes(moments, axes, geometry)

    return { moments = moments, axes = axes }
```

Diagonalization happens **once**, at construction (DESIGN §3.4, §3.8); the
stored axes are reused every frame and never recomputed, precisely so the
arbitrary basis a solver might pick cannot become motion on screen (VISION
Principle 2).

### 4.5 Shifting to a pivot

A heavy top rotates about a fixed pivot, not its center of mass, so its
tensor is shifted by the parallel-axis theorem (DESIGN §3.5). The shifted
tensor is in general **no longer diagonal** in the old axes, so it must be
re-diagonalized by §4.4 — this is not optional, because Euler's equations
(§5) assume a principal-axis frame.

```
function parallel_axis_shift(inertia_com, mass, displacement):
    # DESIGN 3.5:  I_pivot = I_com + M (|d|^2 * Identity - d (outer) d),
    # with d the vector from the center of mass to the pivot.
    distance_squared <- dot(displacement, displacement)
    correction <- mass * (distance_squared * identity_3x3
                          - outer_product(displacement, displacement))
    return inertia_com + correction
```

### 4.6 Classification

The pattern of equality among the moments decides what motion is possible
(DESIGN §3.3), so it is named once and stored on the body. For the
asymmetric top the **intermediate** axis is identified explicitly (DESIGN
§1.4), because the Dzhanibekov flip (VISION Goal 2) is an instability about
that axis and exists only when the three moments are strictly distinct.

```
function classify_top(moments):
    (I_1, I_2, I_3) <- moments
    equal_12 <- approximately_equal(I_1, I_2, MOMENT_TOLERANCE)
    equal_23 <- approximately_equal(I_2, I_3, MOMENT_TOLERANCE)
    equal_13 <- approximately_equal(I_1, I_3, MOMENT_TOLERANCE)

    if equal_12 and equal_23:            # all three equal
        return (SPHERICAL, none)         # no free precession at all
    if equal_12 or equal_23 or equal_13: # exactly two equal
        return (SYMMETRIC, none)         # steady precession
    # All three distinct: the only class with the full Poinsot picture.
    return (ASYMMETRIC, index_of_median(I_1, I_2, I_3))
```

### 4.7 build_body: tying it together

The constructor dispatches on how the body was specified — a shaped
primitive through the provider seam, or three moments entered directly
(DESIGN §3.6, for the Chandler wobble, where only the ratio matters and a
body given this way has no geometry to draw). Either path ends in one
validated, classified, immutable record.

```
function build_body(scenario):
    specification <- scenario.body

    if specification is direct moments:              # DESIGN 3.6
        moments <- (specification.I_1, specification.I_2,
                    specification.I_3)
        validate_moments(moments)                    # reject if unphysical
        principal_moments <- moments
        principal_axes    <- identity_axes           # already principal
        mass              <- specification.mass
        center_of_mass    <- origin
        geometry          <- none                    # nothing to draw

    else:                                            # a shaped primitive
        (mass, center_of_mass, inertia_tensor)
            <- inertia_provider(specification.shape,
                                specification.density)
        validate_inertia_tensor(inertia_tensor)      # symmetry (4.2)
        if specification has a pivot:                # DESIGN 3.5
            displacement <- specification.pivot - center_of_mass
            inertia_tensor <- parallel_axis_shift(inertia_tensor,
                                                  mass, displacement)
        frame <- principal_frame_of(inertia_tensor,  # 4.4; may shortcut
                                    specification.shape)
        principal_moments <- frame.moments
        principal_axes    <- frame.axes
        validate_moments(principal_moments)          # positivity, triangle
        geometry          <- specification.shape

    (top_class, intermediate_axis) <- classify_top(principal_moments)

    # Rigidity as an invariant (DESIGN 3.8): the record is immutable, and
    # changing any of it -- an edge length, the pivot, the moments --
    # builds a NEW body rather than mutating this one, which keeps the
    # scenario record (§11) unambiguous about which body ran.
    return immutable Body {
        principal_moments = principal_moments,
        principal_axes    = principal_axes,
        total_mass        = mass,
        center_of_mass    = center_of_mass,
        top_class         = top_class,
        intermediate_axis = intermediate_axis,
        geometry          = geometry }
```

---

## 5. Equations of Motion

This section is the physics: the derivative the integrator steps. DESIGN
§4 derives it. The `omega_dot` half is where the physics lives — Euler's
equations — and the `q_dot` half is the pure bookkeeping of §2.3 that
records how orientation follows the angular velocity. A torque enters only
the first half; it never appears in `q_dot` (DESIGN §4.2).

### 5.1 Euler's equations: the angular-acceleration half

In the body principal-axis frame the inertia tensor is diagonal, so the
vector equation `I omega_dot + omega x (I omega) = Gamma` separates into
three scalar ones, and inverting `I` is a per-component scalar divide
rather than a matrix solve (DESIGN §4.1).

```
function angular_acceleration_body(angular_velocity_body, body,
                                   torque_body):
    (omega_1, omega_2, omega_3) <- angular_velocity_body
    (I_1, I_2, I_3)             <- body.principal_moments
    (Gamma_1, Gamma_2, Gamma_3) <- torque_body
    # Euler's equations (DESIGN 4.1). The (I_j - I_k) products are the
    # gyroscopic term omega x (I omega), written out per axis.
    return ( ((I_2 - I_3)*omega_2*omega_3 + Gamma_1) / I_1,
             ((I_3 - I_1)*omega_3*omega_1 + Gamma_2) / I_2,
             ((I_1 - I_2)*omega_1*omega_2 + Gamma_3) / I_3 )
```

The gyroscopic term carries the whole of free-rotation behavior, and it
vanishes in exactly the two circumstances DESIGN §4.3 names: when `omega`
lies along a principal axis (so two components are zero and the third is
constant — rotation about a principal axis persists), and for a spherical
top, where it is identically zero for every `omega`. That second case is
why a cube tumbles as a sphere does (§4.6, DESIGN §3.3).

### 5.2 The complete derivative

The two halves close the seven-component system of DESIGN §2.1. This is the
function `advance_one_substep` (§1.1) wraps in a closure and hands to the
integrator; DESIGN §4.2 fixes its signature.

```
function state_derivative(time, state, body, torque_models):
    # PURE: reads the state, returns a derivative, mutates nothing, so a
    # multi-stage integrator (7) may evaluate it at trial states off the
    # trajectory without the evaluations interfering (DESIGN 4.2).
    quaternion            <- state.body_to_space_quaternion
    angular_velocity_body <- state.angular_velocity_body

    # Torque first, summed in body components (5.3).
    torque_body <- total_torque_body(time, state, body, torque_models)

    # The physics half and the bookkeeping half (DESIGN 4.2):
    omega_rate      <- angular_acceleration_body(             # 5.1
                           angular_velocity_body, body, torque_body)
    quaternion_rate <- orientation_derivative(quaternion,     # 2.3
                                              angular_velocity_body)

    # The derivative carries the same two fields as the state, so the
    # integrator advances each by the matching rate.
    return { body_to_space_quaternion = quaternion_rate,
             angular_velocity_body    = omega_rate }
```

### 5.3 How torque enters: the additive body-frame total

DESIGN §4.5 constrains the torque in exactly two ways: it arrives in **body
components**, because that is the frame Euler's equations are written in,
and it is **additive**, so several models acting at once sum to a single
`Gamma`. A torque naturally expressed in space — gravity is the obvious
case — is rotated to body axes *inside its own model* (§6), not here, which
is what lets this sum stay frame-uniform and lets §6 add models without
touching §5.

```
function total_torque_body(time, state, body, torque_models):
    total <- (0, 0, 0)
    for model in torque_models:          # fixed order (DESIGN 5.6)
        total <- total + model.torque_body(time, state, body)   # 6
    return total
```

With an empty torque list the total is the zero vector, and the motion is
torque-free — the case §3.3's invariants and the Poinsot construction (§10)
describe.

### 5.4 The intermediate-axis instability rate

DESIGN §4.4 derives the Dzhanibekov flip rather than treating it as a
curiosity, because the derivation yields a growth rate the tool and the
test suite can both use. A small tilt off the **intermediate** axis grows
as `exp(sigma t)`; the rate depends only on the moments and the spin rate.

```
function instability_growth_rate(body, spin_rate):
    # DESIGN 4.4. Written in terms of the sorted moments so it does not
    # depend on which axis is labeled intermediate; this specializes the
    # axis-1 form (I_3 - I_1)(I_1 - I_2) / (I_2 I_3) of DESIGN 4.4.
    if body.top_class != ASYMMETRIC:
        return 0        # interface guard: no distinct moments, no flip
    (I_min, I_mid, I_max) <- sorted(body.principal_moments)
    return abs(spin_rate) * sqrt(
        (I_max - I_mid) * (I_mid - I_min) / (I_min * I_max) )

function estimate_time_to_flip(growth_rate, initial_tilt):
    # Roughly when to tell a student to watch (DESIGN 4.4), from
    # eps(t) = initial_tilt * exp(growth_rate * t) reaching order one.
    if growth_rate = 0:
        return infinity
    return (1 / growth_rate) * ln(1 / initial_tilt)
```

The `top_class != ASYMMETRIC` guard is DESIGN §4.4's third use of `sigma`
stated as code: rotation about the intermediate axis of a symmetric or
spherical top has no instability to show, so the interface must not offer
the demonstration for a body that provably cannot exhibit it. And a caution
DESIGN §4.4 stresses: started *exactly* on the intermediate axis the body
stays there forever, so the flip must be triggered by a deliberate,
scenario-recorded initial tilt — never by rounding noise, which would make
the demonstration irreproducible (VISION Goal 11).

### 5.5 Conservation rates

Two exact rate laws follow from the equations of motion and are the
foundation of the conservation monitor (§8). They hold **whether or not** a
torque acts, which is what keeps the monitor meaningful once gravity is
switched on (DESIGN §4.6).

```
function energy_rate(state, torque_body):
    # DESIGN 4.6: d(kinetic_energy)/dt = Gamma . omega, in body axes.
    return dot(torque_body, state.angular_velocity_body)

# The companion law is d(angular_momentum_space)/dt = Gamma_space: the
# space-frame angular momentum changes at exactly the space-frame torque.
# Both rates vanish when Gamma = 0, recovering the torque-free
# conservation of energy and of L_space (§3.3, DESIGN 4.6).
```

The monitor does not test for constancy — that would go blind the instant a
torque acted. It integrates these rates over each interval and compares the
result against the observed change in energy and momentum, reporting the
discrepancy. That discrepancy, not the raw drift, is the honest measure of
integration error under torque (§8, DESIGN §4.6, VISION Principle 2).

---

## 6. Torque Models

A torque model supplies the `Gamma` that §5 takes as given. DESIGN §5 fixes
their common interface and specifies the two the first version ships. The
whole design turns on one restraint: the equations of motion name no
particular torque, so a new model is an addition that never touches §5, and
anything that cannot be written through this interface is a signal that it
is not a torque at all (§6.5 is the worked example).

### 6.1 The interface

Every model answers one question (DESIGN §5.1): at the current instant,
what torque acts on this body? Each presents a single operation,

```
torque_body(time, state, body) -> 3-vector in body components
```

obeying the two constraints of DESIGN §4.5 — **body** components and
**additive** — and, like the derivative it feeds, it is **pure**. A model
that recorded or accumulated anything would corrupt the multi-stage
integrator, which evaluates the derivative at trial states off the
trajectory (DESIGN §5.1). The full state is passed to every model even
though each uses only part of it — gravity needs the orientation, damping
the angular velocity, a driven torque the time — so that one signature
serves them all.

Each model below is written as a constructor that captures its fixed
parameters once and returns an object exposing `torque_body`; the total
(§5.3) iterates over a list of these.

### 6.2 Torque-free motion is the empty list

Torque-free motion is not a code path. It is an **empty** list of models,
whose sum `total_torque_body` (§5.3) returns as the zero vector. There is
no null model and no `if torque is none` branch — deliberately, because
torque-free motion is the most-used configuration (VISION Goal 1), and a
branch only the common case takes is one whose failure is found late
(DESIGN §5.2).

### 6.3 Uniform gravity through a pivot

The heavy top (VISION Goal 3) turns about a fixed pivot while gravity acts
at the center of mass, displaced from it. Gravity is naturally a
space-frame vector — it points down whatever the body does — so it is
rotated into the body frame and crossed with the **constant** body-frame
lever arm (DESIGN §5.3). That ordering does one rotation per step, not two,
and keeps the constant arm visibly constant.

```
function rotate_space_to_body(quaternion, vector_space):
    # The inverse of rotate_body_to_space (2.2): because q maps body to
    # space, its conjugate maps space to body.
    return rotate_body_to_space(quat_conjugate(quaternion),
                                vector_space)

function make_gravity_torque(gravity_space, pivot_to_com_body):
    # gravity_space is the acceleration vector (points down); the lever
    # arm pivot_to_com_body is fixed in the body frame (DESIGN 3.8, 5.3).
    function torque_body(time, state, body):
        gravity_body <- rotate_space_to_body(
                            state.body_to_space_quaternion,
                            gravity_space)
        weight_body  <- body.total_mass * gravity_body   # force = M g
        return cross(pivot_to_com_body, weight_body)      # tau = r x F
    return model exposing torque_body
```

Two obligations this model inherits from the body it acts on. The inertia
tensor must be the one **about the pivot**, which `build_body` produces via
the parallel-axis shift when a pivot is specified (§4.5, DESIGN §3.5); and
the classic tractable top has `pivot_to_com_body` along the symmetry axis,
the one displacement that keeps the tensor diagonal. The steady-precession
rate `M g l / (I_3 omega_3)` that this configuration admits is developed as
an analytic oracle and overlay in §9 (DESIGN §5.3, §8), not here.

### 6.4 Viscous damping

A simple, honest external dissipation: a torque opposing the angular
velocity (DESIGN §5.4). It is the right model for a body immersed in a
fluid or dragging on its mount — and, as §6.5 insists, *not* a model of
internal friction.

```
function make_viscous_damping(damping_coefficient):
    function torque_body(time, state, body):
        # Its power is Gamma . omega = -c |omega|^2 <= 0 (DESIGN 5.4),
        # so energy falls monotonically and the body spins down. Angular
        # momentum falls too, because this is a genuine external torque.
        return -damping_coefficient * state.angular_velocity_body
    return model exposing torque_body
```

### 6.5 Internal dissipation is not a torque

Internal dissipation — a body losing energy to its own deformation while
nothing outside exerts a torque — is defined by **angular momentum
conserved and energy not**. No external torque can produce that pairing:
`Gamma = 0` is what conserving `L` requires, and it forces
`Gamma . omega = 0` and hence constant energy. Nor can a *rigid* body,
whose motion is already fully determined by fixed `L_space` and constant
body-frame `I`, with no freedom left to dissipate. So internal dissipation
is a statement that the body is **not rigid** (DESIGN §5.5).

For that reason it does **not** implement the §6.1 interface. It belongs to
a separate category of *state modifiers* applied after the torque sum,
constrained to hold `angular_momentum_space` fixed while reducing
`kinetic_energy` — driving the body toward rotation about its
maximum-moment axis, the minimum-energy state at fixed `|L|`. It is **not
scheduled for the first version** and carries no pseudocode here; it is
named so that the torque interface is not mistakenly widened to hold
something that does not belong in it (DESIGN §5.5, VISION Principle 12).

### 6.6 Composition and the ordering rule

The total torque is the sum over all active models (§5.3). Addition
commutes, so the *physics* is order-independent — but floating-point
addition is **not associative**, and ARCHITECTURE §6.4 promises bit-for-bit
identical trajectories. The model list therefore has a fixed order,
recorded in the scenario (§11) and iterated in that recorded order by
`total_torque_body`. This costs nothing and removes an irreproducibility
that would otherwise surface only as a slowly growing last-digit
difference — nearly impossible to diagnose after the fact (DESIGN §5.6).

### 6.7 Room for later models

Because nothing in §5 or §6 names a specific torque, later ones arrive as
additions (DESIGN §5.7). Two are anticipated, and both already fit the
§6.1 interface:

- **Electromagnetic torque** (VISION Future Direction 3), of the form
  `cross(magnetic_moment, magnetic_field)` — orientation-dependent,
  exactly as gravity is.
- **Driven or time-dependent torque**, for forced-precession
  demonstrations, which is the reason `time` sits in the signature though
  no shipping model reads it yet.
