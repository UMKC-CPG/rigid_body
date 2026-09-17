"""The batch job, ``rbbatch`` (Tier 2, ARCHITECTURE Section 3.9).

The batch entry point: it loads a scenario file, runs the same engine the
interactive tier uses (PSEUDOCODE Section 1.3) with no renderer, and writes
the full state stream to HDF5 with an XDMF companion for ParaView. The
scenario is embedded in the output as provenance, so any result traces back
to the exact run that produced it, and because the run is deterministic the
same scenario reproduces the same trajectory bit-for-bit on any machine
(ARCHITECTURE Section 6.4).

The script follows the ``XYZ.py`` / ``XYZrc.py`` idiom of the other CPG
codes: a :class:`ScriptSettings` class pulls machine-dependent defaults
from ``rbbatchrc.py`` and reconciles them with the command line, with
precedence ``rc defaults < scenario < command line`` (ARCHITECTURE Section
7). Nothing the rc file supplies can change the physics; that lives in the
scenario alone.

WHERE THIS CODE LIVES, AND WHY. This module is the body of the ``rbbatch``
command. It is inside the package, not in ``src/scripts/``, because the
command is reached in two ways that must run the same code (ARCHITECTURE
Sections 3.9 and 9.5): the executable ``src/scripts/rbbatch.py``, which
the physdemo suite links and a clone runs directly; and the console script
that ``pip install`` creates from ``pyproject.toml``, which calls
:func:`console_main` below. A scenario may be given as the bare name of a
packaged example (PSEUDOCODE Section 16).
"""

import argparse
import os
import sys
from typing import NamedTuple, Optional

from rigid_body.cli.support import (
    copy_rc_file, load_rc_defaults, locate_scenario, record_command)
from rigid_body.scenario.serialization import (
    load_scenario, scenario_to_toml, build_run_components,
    run_batch_from_scenario)
from rigid_body.sinks.hdf5_sink import Hdf5Sink, _xdmf_path_for
from rigid_body.analysis.conservation_monitor import ConservationMonitor

RC_FILENAME = "rbbatchrc.py"


# The defaults used when no rbbatchrc.py can be found (so the command and
# its testable core run even in a damaged installation). They mirror
# defaults/rbbatchrc.parameters_and_defaults.
_BUILTIN_RC_DEFAULTS = {
    "output_directory": ".",
    "write_xdmf": True,
    "flush_every": 2048,
    "enable_monitor": True}


class BatchResult(NamedTuple):
    """The outcome of one batch run, for reporting and for tests.

    ``output_path`` and ``xdmf_path`` are where the results landed (the
    latter ``None`` when the XDMF companion was suppressed);
    ``sample_count`` is how many states were written; ``final_state`` is the
    last integrated state; and ``drift_trend`` is the conservation monitor's
    secular-growth summary, or ``None`` when the monitor was disabled.
    """

    output_path: str
    xdmf_path: Optional[str]
    sample_count: int
    final_state: object
    drift_trend: object


def default_output_path(scenario_path, output_directory):
    """Return the default HDF5 path for a scenario: its stem, in the dir."""
    stem = os.path.splitext(os.path.basename(scenario_path))[0]
    return os.path.join(output_directory, stem + ".h5")


def run_batch_job(scenario_path, output_path, write_xdmf=True,
                  flush_every=2048, enable_monitor=True):
    """Load a scenario, run it, and write the trajectory to disk.

    The testable core of the script, free of argument parsing: it loads the
    scenario, attaches an HDF5 recording sink carrying the scenario as
    embedded provenance, optionally runs a conservation monitor over the
    run's own body and torques, and advances the deterministic batch loop.
    Returns a :class:`BatchResult`.
    """
    scenario = load_scenario(scenario_path)

    recorder = Hdf5Sink(
        output_path, scenario_toml=scenario_to_toml(scenario),
        flush_every=flush_every, write_xdmf=write_xdmf)

    monitor = None
    if enable_monitor:
        # Build the monitor from the identical resolved pieces the run
        # uses, so it watches the run's own body and torques (Section 8.1).
        initial_state, body, torque_models, _integrator = (
            build_run_components(scenario))
        monitor = ConservationMonitor(
            initial_state, body, torque_models)

    final_state = run_batch_from_scenario(
        scenario, [recorder], monitor=monitor)

    drift_trend = None
    if monitor is not None:
        drift_trend = monitor.residual_trend.summary()

    return BatchResult(
        output_path=output_path,
        xdmf_path=_xdmf_path_for(output_path) if write_xdmf else None,
        sample_count=recorder.sample_count,
        final_state=final_state,
        drift_trend=drift_trend)


class ScriptSettings:
    """User settings, reconciled from the rc file and the command line.

    The variable values are pulled from ``rbbatchrc.py`` and then reconciled
    with the command-line parameters, with the command line taking
    precedence (ARCHITECTURE Section 7).
    """

    def __init__(self, command_line_args=None):
        """Resolve settings from the rc file and the command line.

        Passing ``command_line_args`` (a list of strings) is what lets the
        test suite drive this class without touching ``sys.argv``. The
        invocation is NOT logged here; the fronts do that (support.py).
        """
        default_rc = load_rc_defaults(RC_FILENAME, _BUILTIN_RC_DEFAULTS)
        self.assign_rc_defaults(default_rc)
        arguments = self.parse_command_line(command_line_args)
        self.reconcile(arguments)

    def assign_rc_defaults(self, default_rc):
        """Seed the settings from the resource-control defaults."""
        self.output_directory = default_rc["output_directory"]
        self.write_xdmf = default_rc["write_xdmf"]
        self.flush_every = default_rc["flush_every"]
        self.enable_monitor = default_rc["enable_monitor"]
        # Filled in from the command line during reconciliation.
        self.scenario_path = None
        self.output_path = None
        self.write_rc = False

    def parse_command_line(self, command_line_args=None):
        """Build the parser and return the parsed arguments."""
        description_text = """
Run a saved rigid-body scenario as a deterministic batch job, writing the
full trajectory to HDF5 with an XDMF companion for ParaView. The scenario
is embedded in the output as provenance.
"""
        epilog_text = """
Defaults are taken from ./rbbatchrc.py or $RIGID_BODY_RC/rbbatchrc.py (make
the first with --write-rc) and may be overridden on the command line. The
physics lives entirely in the scenario file; the rc file governs only where
and how output is written. The scenario may be the bare name of a packaged
example (rbbatch dzhanibekov); `rbsim --examples` copies them here.
"""
        parser = argparse.ArgumentParser(
            prog="rbbatch",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description=description_text, epilog=epilog_text)
        self.add_parser_arguments(parser)
        arguments = parser.parse_args(command_line_args)
        if (arguments.scenario is None) == (not arguments.write_rc):
            parser.error("give a scenario, or --write-rc")
        return arguments

    def add_parser_arguments(self, parser):
        """Declare the batch script's command-line arguments."""
        parser.add_argument(
            "scenario", nargs="?", default=None,
            help="Path to the scenario TOML file to run, or the bare name "
                 "of a packaged example.")
        parser.add_argument(
            "--write-rc", dest="write_rc", action="store_true",
            help="Copy the shipped rbbatchrc.py here, to edit, and exit.")
        parser.add_argument(
            "-o", "--output", dest="output", default=None,
            help="Output HDF5 path. Default: the scenario's stem in the "
                 "output directory.")
        parser.add_argument(
            "--output-dir", dest="output_directory",
            default=self.output_directory,
            help=f"Directory for default output. "
                 f"Default: {self.output_directory}")
        parser.add_argument(
            "--no-xdmf", dest="write_xdmf", action="store_false",
            default=self.write_xdmf,
            help="Suppress the XDMF companion for ParaView.")
        parser.add_argument(
            "--no-monitor", dest="enable_monitor", action="store_false",
            default=self.enable_monitor,
            help="Skip the conservation-drift report.")
        parser.add_argument(
            "--flush-every", dest="flush_every", type=int,
            default=self.flush_every,
            help=f"Samples buffered before a disk write. "
                 f"Default: {self.flush_every}")

    def reconcile(self, arguments):
        """Let the command line override the rc defaults."""
        self.write_rc = arguments.write_rc
        self.scenario_path = arguments.scenario
        self.output_directory = arguments.output_directory
        self.write_xdmf = arguments.write_xdmf
        self.enable_monitor = arguments.enable_monitor
        self.flush_every = arguments.flush_every
        self.output = arguments.output
        # The default output name comes from the scenario's stem, which is
        # known only once a bare example name has been resolved (main).
        self.output_path = None


def report(result):
    """Print a short human summary of a completed batch run."""
    print(f"Wrote {result.sample_count} samples to {result.output_path}")
    if result.xdmf_path is not None:
        print(f"XDMF companion: {result.xdmf_path}")
    if result.drift_trend is not None:
        print("Conservation drift trend (relative growth per unit time):")
        print(f"  energy:   {result.drift_trend.energy_growth_rate:.3e}")
        print(f"  momentum: {result.drift_trend.momentum_growth_rate:.3e}")


def main(command_line_args=None):
    """Assemble the settings, run the batch job, and report the outcome.
    Returns the exit status. Accepting an argument list lets the test
    suite drive this without ``sys.argv``."""
    settings = ScriptSettings(command_line_args)
    if settings.write_rc:
        return copy_rc_file(RC_FILENAME)
    # A scenario that is missing or wrong is the commonest mistake a
    # student makes; it earns a message and status 2, not a traceback.
    try:
        scenario_path = str(locate_scenario(settings.scenario_path,
                                            command_name="rbbatch"))
        output_path = settings.output or default_output_path(
            scenario_path, settings.output_directory)
        result = run_batch_job(
            scenario_path, output_path,
            write_xdmf=settings.write_xdmf,
            flush_every=settings.flush_every,
            enable_monitor=settings.enable_monitor)
    except (FileNotFoundError, ValueError, KeyError) as problem:
        print(f"rbbatch: {problem}", file=sys.stderr)
        return 2
    report(result)
    return 0


def console_main():
    """The front that ``pip install`` creates (pyproject.toml,
    ``[project.scripts]``). It is the real entry point on that route, so
    it is where the invocation is logged; ``main()`` itself never logs."""
    record_command()
    sys.exit(main())
