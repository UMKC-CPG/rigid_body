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
| 3 | State and derived quantities | §2.1, §2.5 | not yet written |
| 4 | Inertia and body construction | §3 | not yet written |
| 5 | Equations of motion | §4 | not yet written |
| 6 | Torque models | §5 | not yet written |
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
