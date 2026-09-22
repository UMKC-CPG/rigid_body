"""The example run files, packaged so that they reach every user.

This directory holds data, not code: each `*.toml` file here is a
complete, ready-to-run run file. It is a package only so that the
files are installed along with the library and can be found through
it (`rigid_body.cli.support.example_files`), which is what lets a
command run an example by its bare name, and `--examples` copy them
out, identically in a clone, in a shared `physdemo` suite, and in a
`pip`-installed copy (physdemo contract C8). A clone keeps a
top-level symbolic link to this directory as a short path; nothing
may depend on it.

Attribution: this module is part of the Rigid Body teaching tool.
"""
