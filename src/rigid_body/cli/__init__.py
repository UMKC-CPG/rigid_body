"""The bodies of the command-line entry points (ARCHITECTURE Section 3.9).

A command is reached in two ways that must run the same code: the
executable scripts in ``src/scripts/``, which the ``physdemo`` suite links
and a clone runs directly, and the console scripts that ``pip install``
creates from ``pyproject.toml``. So each command's body lives here, in
the library, and the two fronts are a few lines each.

Nothing in the package imports ``cli/``; it sits at the top of the
dependency graph (ARCHITECTURE Section 4) and holds no physics.

Attribution: this module is part of the rigid_body teaching tool.
"""
