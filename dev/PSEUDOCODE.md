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
| 2 | Orientation mathematics | §1.3, §2.2, §2.4, §2.6 | not yet written |
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
