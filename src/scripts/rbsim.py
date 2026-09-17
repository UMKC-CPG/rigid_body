#!/usr/bin/env python3

"""rbsim -- the executable front of the interactive simulation.

This file is deliberately almost empty. The command's body is
``rigid_body.cli.rbsim`` (PSEUDOCODE Section 16), because the tool is
reached in two ways that must run the same code (ARCHITECTURE Sections 3.9
and 9.5):

- through THIS script, which the ``physdemo`` suite links into its
  commands and which a clone runs directly, with nothing installed;
- through the ``rbsim`` console script that ``pip install`` creates,
  which calls ``rigid_body.cli.rbsim.console_main``.

Three things here are load-bearing, and the suite depends on them: the
``#!/usr/bin/env python3`` first line and the executable bit, so the
script runs by name with whatever Python the active environment provides;
and the RESOLVED path below, because the script is normally run through a
symbolic link, and the unresolved path would name the link rather than
this file.

Run ``rbsim --help`` for the usage.
"""

import sys
from pathlib import Path

# The package is src/rigid_body/, beside this scripts/ directory. With the
# package installed this line changes nothing; without it, this is what
# lets the tool run from a fresh clone with no install step.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rigid_body.cli.rbsim import main, record_command     # noqa: E402

if __name__ == "__main__":
    # The invocation is logged here, at the real entry point, and never
    # inside main(), so that the test suite can call main(argv) without
    # leaving ``command`` files behind (CLAUDE.md, "Command Logging").
    record_command()
    sys.exit(main())
