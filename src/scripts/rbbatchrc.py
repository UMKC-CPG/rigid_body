#!/usr/bin/env python3

"""Resource-control defaults for ``rbbatch.py`` (the ``XYZrc.py`` idiom).

This file holds what is machine-dependent and rarely changed for a batch
run -- where output lands and how it is written -- and nothing that can
affect the computed trajectory. Per ARCHITECTURE Section 7, any value that
influences the physics lives in the scenario file, never here: the rc file
governs convenience and environment, the scenario governs physics. A user
may keep a personal copy of this file on ``$RIGID_BODY_RC`` to override the
defaults shipped beside the script.
"""


def parameters_and_defaults():
    """Return the batch script's machine-dependent default settings."""
    param_dict = {
        # Directory the HDF5 output and its XDMF companion are written to
        # when no explicit output path is given on the command line.
        "output_directory": ".",
        # Whether to write the XDMF companion for ParaView beside the HDF5.
        "write_xdmf": True,
        # How many samples the sink buffers before flushing to disk. Larger
        # trades a little memory for fewer, larger writes.
        "flush_every": 2048,
        # Whether to run the conservation monitor alongside the batch run
        # and report its final drift trend. Diagnostic only; it never
        # touches the trajectory (Section 8.3).
        "enable_monitor": True}
    return param_dict


if __name__ == '__main__':
    print(parameters_and_defaults())
