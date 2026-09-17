#!/usr/bin/env python3

"""Run a saved scenario as a batch job (Tier 2, ARCHITECTURE Section 3.9).

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
"""

import argparse
import os
import sys
from datetime import datetime
from typing import NamedTuple, Optional

# realpath, not abspath: this script is normally run through a
# symbolic link that the physdemo suite keeps to it, and the
# package must be found beside the real file, not beside the link.
# The package lives one directory up from this script (in ``src/``); put it
# on the path so ``rigid_body`` imports resolve when the script is run
# directly, e.g. ``python3 src/scripts/rbbatch.py scenario.toml``.
_SOURCE_DIRECTORY = os.path.dirname(
    os.path.dirname(os.path.realpath(__file__)))
if _SOURCE_DIRECTORY not in sys.path:
    sys.path.insert(0, _SOURCE_DIRECTORY)

from rigid_body.scenario.serialization import (          # noqa: E402
    load_scenario, scenario_to_toml, build_run_components,
    run_batch_from_scenario)
from rigid_body.sinks.hdf5_sink import Hdf5Sink, _xdmf_path_for  # noqa: E402
from rigid_body.analysis.conservation_monitor import (   # noqa: E402
    ConservationMonitor)


# The defaults used when no rbbatchrc.py can be imported (so the script and
# its testable core run even without the rc file present). They mirror
# rbbatchrc.parameters_and_defaults.
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


def _load_rc_defaults():
    """Return the rc defaults, preferring a machine copy on $RIGID_BODY_RC.

    Looks first for an ``rbbatchrc.py`` on the ``$RIGID_BODY_RC`` directory
    (a user's machine-specific overrides), then the copy shipped beside
    this script, and falls back to built-in defaults if neither imports.
    """
    rc_dir = os.getenv("RIGID_BODY_RC")
    if rc_dir and os.path.isfile(os.path.join(rc_dir, "rbbatchrc.py")):
        sys.path.insert(1, rc_dir)
    try:
        from rbbatchrc import parameters_and_defaults
    except ImportError:
        return dict(_BUILTIN_RC_DEFAULTS)
    return parameters_and_defaults()


class ScriptSettings:
    """User settings, reconciled from the rc file and the command line.

    The variable values are pulled from ``rbbatchrc.py`` and then reconciled
    with the command-line parameters, with the command line taking
    precedence (ARCHITECTURE Section 7).
    """

    def __init__(self):
        default_rc = _load_rc_defaults()
        self.assign_rc_defaults(default_rc)
        arguments = self.parse_command_line()
        self.reconcile(arguments)
        self.record_command_line()

    def assign_rc_defaults(self, default_rc):
        """Seed the settings from the resource-control defaults."""
        self.output_directory = default_rc["output_directory"]
        self.write_xdmf = default_rc["write_xdmf"]
        self.flush_every = default_rc["flush_every"]
        self.enable_monitor = default_rc["enable_monitor"]
        # Filled in from the command line during reconciliation.
        self.scenario_path = None
        self.output_path = None

    def parse_command_line(self):
        """Build the parser and return the parsed arguments."""
        description_text = """
Run a saved rigid-body scenario as a deterministic batch job, writing the
full trajectory to HDF5 with an XDMF companion for ParaView. The scenario
is embedded in the output as provenance.
"""
        epilog_text = """
Defaults are taken from ./rbbatchrc.py or $RIGID_BODY_RC/rbbatchrc.py and
may be overridden on the command line. The physics lives entirely in the
scenario file; the rc file governs only where and how output is written.
"""
        parser = argparse.ArgumentParser(
            prog="rbbatch",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description=description_text, epilog=epilog_text)
        self.add_parser_arguments(parser)
        return parser.parse_args()

    def add_parser_arguments(self, parser):
        """Declare the batch script's command-line arguments."""
        parser.add_argument(
            "scenario", help="Path to the scenario TOML file to run.")
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
        self.scenario_path = arguments.scenario
        self.output_directory = arguments.output_directory
        self.write_xdmf = arguments.write_xdmf
        self.enable_monitor = arguments.enable_monitor
        self.flush_every = arguments.flush_every
        self.output_path = arguments.output or default_output_path(
            self.scenario_path, self.output_directory)

    def record_command_line(self):
        """Append the invocation to a ``command`` log, as the idiom does.

        The log is a convenience and must never stop a run. The usual way
        to fail here is a student standing inside a shared, read-only
        installation, among the example scenarios; say so in one line and
        carry on (CLAUDE.md, "Command Logging").
        """
        try:
            with open("command", "a") as command_log:
                stamp = datetime.now().strftime("%b. %d, %Y: %H:%M:%S")
                command_log.write(f"Date: {stamp}\n")
                command_log.write("Cmnd:")
                for argument in sys.argv:
                    command_log.write(f" {argument}")
                command_log.write("\n\n")
        except OSError as problem:
            print(f"note: cannot write ./command here ({problem.strerror}); "
                  "continuing without the command log", file=sys.stderr)


def report(result):
    """Print a short human summary of a completed batch run."""
    print(f"Wrote {result.sample_count} samples to {result.output_path}")
    if result.xdmf_path is not None:
        print(f"XDMF companion: {result.xdmf_path}")
    if result.drift_trend is not None:
        print("Conservation drift trend (relative growth per unit time):")
        print(f"  energy:   {result.drift_trend.energy_growth_rate:.3e}")
        print(f"  momentum: {result.drift_trend.momentum_growth_rate:.3e}")


def main():
    """Assemble the settings, run the batch job, and report the outcome."""
    settings = ScriptSettings()
    result = run_batch_job(
        settings.scenario_path, settings.output_path,
        write_xdmf=settings.write_xdmf, flush_every=settings.flush_every,
        enable_monitor=settings.enable_monitor)
    report(result)


if __name__ == "__main__":
    main()
