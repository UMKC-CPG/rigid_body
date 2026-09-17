"""The example scenarios, packaged so that they reach every user.

This directory holds data, not code: each ``*.toml`` file here is a
complete, ready-to-run scenario (DESIGN Section 11). It is a package only
so that the files are installed along with the library and can be found
through it (``rigid_body.cli.support``), which is what lets
``rbsim dzhanibekov`` and ``rbsim --examples`` work identically in a
clone, in a shared ``physdemo`` suite, and in a ``pip``-installed copy
(ARCHITECTURE Section 9.5). The repository's top-level ``scenarios`` is a
symbolic link to this directory.

Attribution: this module is part of the rigid_body teaching tool.
"""
