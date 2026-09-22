# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when
working with code in this repository.

> **Session setup:** run `/rename RIGID_BODY` (and a `/color`) at the
> start of each session so that concurrent sessions on different
> projects stay visually distinct in the terminal.

## Document Hierarchy

This project uses a five-level document chain. All design documents
live in `dev/`. Read them in order when starting work:

1. `dev/VISION.md` — goals, principles, non-negotiables
2. `dev/ARCHITECTURE.md` — layout, modules, dependencies
3. `dev/DESIGN.md` — algorithms, data structures, math
4. `dev/PSEUDOCODE.md` — language-agnostic algorithm specs
5. Source code in `src/`

`dev/TODO.md` tracks tasks organized by level. Use `/focus` to start
a session and `/refine` to check consistency across the chain.

**DESIGN and PSEUDOCODE are single files here**, 127 KB and 152 KB
as of 2026-09-22, holding the numbered sections themselves. The
`scattering` tool and the physdemo skeleton split theirs into
`dev/design/` and `dev/pseudocode/` with an index in each top file,
because on earlier projects a single file reached 800 KB, at which
point no reader could load it to change a paragraph. Splitting this
project's is an open item in `dev/TODO.md`; until then, read a
section by its heading and never the whole file. The slash commands
serve both layouts.

`dev/spikes/README.md` records the throwaway checks the chain cites.

## Chain Discipline: No Code Without Pseudocode

The chain runs downward: VISION → ARCHITECTURE → DESIGN → PSEUDOCODE
→ code. A new feature enters at the top and flows down. Each level is
the specification for the one below it, and each is written before
that one exists.

**The gate.** Before editing any file under `src/`, the governing
PSEUDOCODE section must already exist and must already describe the
change. If it does not, stop and write it first, and say so plainly
rather than proceeding and back-filling afterward. A back-filled level
is not a level: nobody reviewed the code against it, so it records
what was built rather than specifying what should have been.

**Announce the level.** When beginning a coded task, name the
PSEUDOCODE section that governs it before touching `src/`. Being
unable to name one is itself the answer — the pseudocode is missing
and must be written.

**A detailed TODO entry is not pseudocode.** This is the trap that
has actually caught us. A task entry that names the functions, the
files, and the call sequence reads like a specification and is
detailed enough to code from directly. It is not a level of the chain.
It is never checked against DESIGN by `/refine`, and once its box is
ticked it is never read again. When a task's plan grows specific
enough to implement from, that specificity belongs in PSEUDOCODE —
move it there and leave the TODO entry pointing at the section.

**Why this matters more than it looks.** The chain is what lets a
reader who has been away trust the source. Code with a pseudocode
section above it can be checked against a spec that a human agreed
to. Code without one can only be checked against itself, which is no
check at all. Skipping the level costs nothing on the day it is
skipped and costs the project its reviewability forever after.

**The one legitimate upward edit.** Pseudocode may be brought into
agreement with existing code only where that code has first been
verified to implement DESIGN faithfully. Anywhere else, a
disagreement between pseudocode and code means the *code* is wrong.
Never edit the pseudocode to match code merely because the code is
already written.

**New work grafted onto existing code needs a seam inventory.**
Writing a level from the level above is sufficient only when the new
work is a new subsystem. When it must attach to a running program,
the lower levels take that program as a SECOND input, and a graft
point specified without reading the code on the other side of it is
specified from imagination.

So a DESIGN or PSEUDOCODE section that modifies existing code is not
finished until it names, for every quantity the new code consumes or
produces: where it comes from, who allocates it, who loads it, and
when. Write that inventory into the section. It is what a later
reader checks the prose against, and it is where specification errors
have actually occurred on earlier projects — every one of them at a
boundary with a routine nobody had read, none of them in the
algorithm itself.

The inventory also settles structure that would otherwise look like
taste. If a quantity is loaded inside an existing loop, a new
consumer of it has to sit in that loop; that is a consequence of the
seam, not a preference.

**Verify the section you are copying from.** Modelling new pseudocode
on an existing section propagates that section's defects sideways
into work that then looks independently derived. Check the model
section against its own code first.

## Level Awareness

During development conversations, the programmer may shift between
levels of the design chain without explicitly noticing. For example,
a discussion about a code fix may drift into questioning an
algorithm's design, or a design discussion may surface a conflict
with a core principle.

When you notice the conversation has moved to a different level than
where it started, say so briefly. For example: "This sounds like it's
becoming an ARCHITECTURE question — should we capture it there before
continuing with the code?" The goal is awareness, not interruption.
Let the programmer decide whether to switch context, propagate the
change to the appropriate document, or stay focused and defer.

Do not enforce rigid boundaries. The levels exist to organize
thinking, not to prevent it. A developer who is on a productive train
of thought should not be stopped — but when the thought resolves,
help them recognize which documents it touches so nothing is left
inconsistent.

## Secondary Agents and Forks

Do NOT spin up secondary agents, forks, or background subagents to
edit documents or code without the programmer's explicit say-so.
Asking is mandatory, and it must be a plain, visible question — "Do
you want me to fan this out to a separate agent, or should I just do
it inline here?" — never buried inside an obtuse command or a
multi-step sequence the programmer cannot easily see and veto.

The default for any bounded edit (a known set of spots across a few
files) is to do it INLINE in the main thread, one edit at a time, so
the programmer can follow along. Forks are expensive (they clone the
whole conversation context) and confusing to watch alongside the main
thread. Reserve them for genuinely parallel work or broad read-only
searches, and only after an explicit, transparent yes.

## Coding Style

### Line Length

Lines MUST NOT exceed 80 characters. This is a hard limit.

Short statements whose content is naturally brief (`implicit none`,
`endif`, `return`) are fine at their natural length — there is
nothing to fill them with. The rule applies when content is
available: long expository comments, argument lists, complex
expressions, and so on.

**Common failure mode — do not do this:**
```fortran
   ! Guard each deallocation because
   !   this subroutine is called from
   !   multiple program paths.
```
Those three ~35-character lines have plenty of content to fill longer
lines. Write them as:
```fortran
   ! Guard each deallocation because this subroutine is called
   !   from multiple program paths.
```
The idea: let each line run toward 80 before wrapping. Do not break
at natural phrase boundaries when the line is only at 40-50
characters. The result will be fewer, fuller lines — not lines padded
with filler.

`/lint` audits and mechanically repairs both violations; see
"Reflow Tooling" below.

### Documentation and Naming

CRITICAL: All program code must include rich, expressive
documentation so that students reading the source can easily follow
what is happening. Every function, class, and non-trivial block
should carry clear explanatory comments or docstrings that describe
purpose, inputs, outputs, and any relevant physics or math.

Variable names must be readable and self-documenting. Avoid cryptic
one- or two-letter abbreviations. Prefer concise but meaningful names
that a reader can understand without cross-referencing a legend.
Slightly-too-long names are far better than opaque short ones.

Good naming examples for an electronic structure program:
- `elec_mom` instead of `em` or `electron_momentum`
- `nuc_pot` instead of `np` or `nuclear_potential`
- `grid_spacing` instead of `gs` or `the_spacing_between_grid_points`

Apply similar logic to the current project if it is something else.

The goal is a middle ground: short enough to keep expressions tidy,
long enough that any student can read the code cold and follow the
logic without guessing what a variable holds.

### Documentation Preservation

This is an academic codebase used by students who frequently need to
read and understand the source. When refactoring or restructuring
code:

- **Preserve all existing documentation.** Every comment block, usage
  note, option explanation, and conceptual description must be
  carried over. Do not summarize or abbreviate it away.
- **Use the appropriate format.** In Python: module, class, and
  method docstrings, argparse help text, and inline comments. In
  Fortran: header comment blocks and inline comments.
- **Explain the "why", not just the "what".** Students benefit from
  the physics or chemistry motivation, not just the code mechanics.

### Structured Comment Blocks

Some comments contain *structured* content whose visual layout is
itself meaningful: equations with aligned `=` signs, ASCII tables,
multi-line derivations, sub-lists keyed by hand-aligned labels, or
commented-out code you may re-enable later. These blocks must be
marked so the reflow tools leave them alone. Without the marker the
tool treats them as flowing prose and mashes the lines into a
paragraph, destroying the layout.

The marker convention is per-language but conceptually identical: the
comment opener is *doubled* to signal "structured content, do not
reflow."

**Python** — prefix the lines with `##` instead of `#`. Any line
beginning with `##` is protected and is never reflowed. Use it for
commented-out code, multi-line derivations with aligned `=`, ASCII
tables, and any block whose visual layout carries meaning.

```python
# This prose comment may be reflowed by rewrap_prose.py.

## K_theta = 0.15 * sqrt(K_arm1 * K_arm2) * scale
##         = 0.15 * sqrt(400 * 900) * 1.0
##         = 90.0
```

**Fortran** — prefix the lines with `!!` for structured prose, or use
`!` immediately followed by content (no space) for commented-out
code. Both forms are recognised as "do not touch."

```fortran
! This prose comment may be reflowed by rewrap_prose.py.

!! K_theta = 0.15 * sqrt(K_arm1 * K_arm2) * scale
!!         = 0.15 * sqrt(400 * 900) * 1.0
!!         = 90.0

!do i = 1, n           ! commented-out code: no space after !
!  call compute(i)
!end do
```

When writing a comment, ask: "if a reflow tool joined these lines
into one paragraph, would I lose meaningful structure?" If yes, use
the doubled form. Plain `#` or `! ` (single, followed by a space) is
correct only for genuine free-flowing prose.

### Reflow Tooling

Two deterministic helpers live in `.claude/commands/scripts/` and are
driven by `/lint`. Prefer them over hand-editing: they are
idempotent, they are far cheaper than reflowing by hand, and they do
not silently reword prose the way a manual pass can.

- `rewrap_prose.py FILE [--check]` — reflows comment paragraphs
  (Fortran `!`, Python `#`) and prose paragraphs inside Python
  docstrings into the 70-80 band.
- `rewrap_code.py FILE [--check]` — finds over-wrapped code blocks
  (lines split across 3-4 lines that would fit on 1-2) and compacts
  them.

Run either with `--check` first for a dry run. Neither touches the
protected forms described above.

## Attribution

CRITICAL: Attribution is extremely important in scientific
programming.

If program code or algorithms are derived from existing citable
resources, then it is important to include citations to that work
when producing any documents that are derived from them.

Similarly, it is anticipated that this program will be ingested by an
LLM or similar AI system. Therefore, directives should be placed in
appropriate places throughout the code base informing the AI system
of its responsibility to properly attribute this project and the
references therein when producing any additional source code derived
from this code base.

## Command Logging

Every user-invokable script appends the issued command line to a file
named `command` in the current working directory — a dated `Date:` /
`Cmnd: <argv>` block per run — so the exact invocation is recoverable
later. This is the group's standard idiom (the project template's
`XYZ.py`); both commands share it as `cli/support.record_command()`,
called from the fronts — the executable script's `__main__` block and
`console_main()` — and never from `main()`, so that the test suite can
call `main(argv)` without leaving `command` files behind.

The log is a convenience and MUST NOT stop a run. Where the working
directory cannot be written — a student standing inside a shared,
read-only installation, among the example scenarios — the method says
so in one line on standard error and returns.

## What This Is

An interactive, real-time teaching tool for **rigid-body rotation** in
a graduate theoretical-mechanics course, and the companion of the
scattering tool in `../scattering/`. A student sets up a body and its
initial conditions, then watches the motion evolve live alongside the
conserved quantities that govern it: the torque-free tumbling of an
asymmetric top with the full Poinsot construction, the
intermediate-axis (Dzhanibekov) instability, and gyroscopic precession
and nutation, each shown in both the body frame and the space frame.
`README.md` has the user's view; `dev/VISION.md` the goals.

## Repository Layout

```
dev/              Design document chain (single-file DESIGN and
                  PSEUDOCODE; see "Document Hierarchy")
  spikes/         Verified throwaway checks cited by the chain
scenarios         Symbolic link to src/rigid_body/examples/
src/
  rigid_body/     The importable library, by concern, plus:
    cli/          Bodies of the commands (rbsim, rbbatch), and
                  support.py, inherited from the physdemo skeleton
    defaults/     The shipped rc files, rbsimrc.py and rbbatchrc.py
    examples/     Ready-to-run example scenarios (TOML)
  scripts/        Thin executable fronts for cli/
tests/            pytest suite: unit/, integration/, regression/
.rigid_body/      Machine-local rc overrides (never tracked)
```

## Inherited From the Suite

Some files are the suite's, not this tool's, and `dev/PSEUDOCODE.md`
section 16 lists them: the fronts in `src/scripts/`, `cli/support.py`,
`render/offscreen.py`, the `defaults/` and `examples/` packages'
`__init__.py`, `tests/conftest.py`, the installed-copy and offscreen
tests, and `.claude/commands/`. They are governed by the suite's
`dev/PSEUDOCODE.md` section 4. A change to one of them is made in the
suite's skeleton (`../physdemo/template/`) first and carried here with
`physdemo-new-tool --refresh --package rigid_body --command rbsim
--title "Rigid Body" .` and again with `--command rbbatch`; do not
edit this copy on its own. `physdemo-check-tool .` verifies the
suite's contract; run both before a release.

## Running

The tool reaches a user in two ways that run the same code
(`dev/ARCHITECTURE.md` §9.5): **Route A**, the `physdemo` suite
(`../physdemo/`, `github.com/UMKC-CPG/physdemo`), which owns a shared
environment and links the scripts in `src/scripts/` — for a shared
computer, and for development here (`sdemo` sources the suite's
`activate.sh`); and **Route B**, `pip install` of this repository,
which creates `rbsim` and `rbbatch` from `pyproject.toml` — for a
laptop, including Windows.

```bash
sdemo                                # activate the suite (Route A)
rbsim --check                        # can this computer run and draw?
rbsim dzhanibekov                    # a packaged example, by name
rbsim scenarios/dzhanibekov.toml     # the same file, in a clone
rbbatch dzhanibekov -o out.h5        # Tier 2
```

Rules that keep both routes working, and that are tested
(`tests/unit/test_installed_copy.py`):

- **Everything a run needs is inside `src/rigid_body/`,** because that
  is all `pip install` delivers: code, `defaults/*rc.py`,
  `examples/*.toml`. Find such files through the package
  (`importlib.resources`, or a path from a module's own resolved
  `__file__`), never relative to a script or the working directory.
- **A command's body is a module in `cli/`.** `src/scripts/<name>.py`
  is only a front: shebang, executable bit, `src/` put on the path
  from `Path(__file__).resolve()` (it is normally run through a
  symbolic link), then a call into `cli/`. It defines no function.
- **A new third-party import is declared in `pyproject.toml`,** with a
  lower bound no tighter than the suite's `requirements.in`, and is
  added to the suite first.
- **Offscreen drawing decides VTK's window class before VTK is
  imported** (`render/offscreen.py`, PSEUDOCODE §16.4), so nothing in
  `cli/` imports the renderer at module level.
- Put no absolute path and nothing specific to one computer in this
  repository; site notes belong in `physdemo/site/`.
- Assume the working directory may be read-only and the home directory
  small: a failed side-effect write is one line on standard error,
  never a traceback, and never stops the physics.

## Versions and Releases

The version is `MAJOR.MINOR.PATCH` and is stated in **exactly one
place**: `__version__` in `src/rigid_body/__init__.py`. `pyproject.toml`
reads it from there (`dynamic = ["version"]`), so never write a
version number into `pyproject.toml`, and never let a second copy
appear anywhere else.

**Why it matters.** `pip` decides whether to update an installed copy
by comparing this number, not the code. A laptop that installed
`0.9.0` and runs `pip install --upgrade <URL>` gets nothing until the
number changes, however much `main` has moved. So an unbumped version
is a release that students cannot receive.

**Cutting a release**, when the user asks for one:

1. Bump `__version__`: PATCH for fixes that change no behaviour a run
   file can see; MINOR for new features or new run-file keys; MAJOR
   when a run file written for the old version no longer loads or no
   longer means the same thing. Before `1.0.0`, MINOR carries new
   milestones and MAJOR stays at 0.
2. Run the test suite.
3. Commit with a message beginning `Release <version>:` and a line on
   what changed for a user.
4. Tag it `v<version>` exactly (`v0.9.0`), annotated: `git tag -a
   v0.9.0 -m "Release 0.9.0"`. The tag must match `__version__` so
   that `pip install .../refs/tags/v0.9.0.zip` installs what it says.
5. Tell the user to push with tags (`git push --follow-tags`), and
   that the README's install URL can now point at the tag.

The older milestone tags (`v0.8-detector` and the like) mark chain
stages, not releases; leave them alone and do not imitate their form
for a release.

**Reading the version.** `rigid_body.__version__` at run time;
`pip show rigid_body` for an installed copy. A `--version` flag on the
commands is not yet provided; add it as an ordinary chain change if
wanted.

## Testing

```bash
# Run the test suite.
pytest tests/ -v
```
