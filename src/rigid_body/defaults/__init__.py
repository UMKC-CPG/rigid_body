"""The shipped resource-control (rc) defaults.

``rbsimrc.py`` and ``rbbatchrc.py`` here are the documented sets of
machine-local defaults and the last place the commands look (ARCHITECTURE
Section 7; DESIGN Section 15.2). They are inside the package, rather than
beside the entry-point scripts, because an installed copy of the tool has
no script directory: the package is the one location that exists however
the tool was obtained. A user who wants to change a default runs
``rbsim --write-rc`` (or ``rbbatch --write-rc``), which copies the file
into the working directory, and edits the copy.

Attribution: this module is part of the rigid_body teaching tool.
"""
