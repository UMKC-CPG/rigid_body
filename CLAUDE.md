# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with
code in this repository.

## Document Hierarchy

This project uses a five-level document chain. All design documents live
in `dev/`. Read them in order when starting work:

1. `dev/VISION.md` — goals, principles, non-negotiables
2. `dev/ARCHITECTURE.md` — layout, modules, dependencies
3. `dev/DESIGN.md` — algorithms, data structures, math
4. `dev/PSEUDOCODE.md` — language-agnostic algorithm specs
5. Source code in `src/`

`dev/TODO.md` tracks tasks organized by level. Use `/focus` to start a
session and `/refine` to check consistency across the chain.

## Level Awareness

During development conversations, the programmer may shift between levels
of the design chain without explicitly noticing. For example, a discussion
about a code fix may drift into questioning an algorithm's design, or a
design discussion may surface a conflict with a core principle.

When you notice the conversation has moved to a different level than where
it started, say so briefly. For example: "This sounds like it's becoming
an ARCHITECTURE question — should we capture it there before continuing
with the code?" The goal is awareness, not interruption. Let the programmer
decide whether to switch context, propagate the change to the appropriate
document, or stay focused and defer.

Do not enforce rigid boundaries. The levels exist to organize thinking,
not to prevent it. A developer who is on a productive train of thought
should not be stopped — but when the thought resolves, help them recognize
which documents it touches so nothing is left inconsistent.

## Coding Style

### Line Length

Lines MUST NOT exceed 80 characters. This is a hard limit.

### Documentation and Naming

CRITICAL: All program code must include rich, expressive documentation
so that students reading the source can easily follow what is happening.
Every function, class, and non-trivial block should carry clear
explanatory comments or docstrings that describe purpose, inputs,
outputs, and any relevant physics or math.

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

## Project Overview

<!-- Replace this section with your project description, how to run it,
dependencies, and architecture summary. -->

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

## Dependencies

<!-- List your project's dependencies here. -->

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
