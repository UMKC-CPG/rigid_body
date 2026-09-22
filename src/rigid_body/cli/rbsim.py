"""The interactive simulation, ``rbsim`` (Tier 1, ARCHITECTURE Section 3.9).

The interactive entry point: it loads a scenario file, opens a vedo window,
and runs the interactive frame loop (PSEUDOCODE Section 1.2) -- advancing
the physics at the pace the time controls choose and drawing the two-frame
Poinsot view every frame. It is the same engine step the batch tier runs,
wrapped in a renderer, live controls, and an event pump.

The script follows the ``XYZ.py`` / ``XYZrc.py`` idiom of the other CPG
codes: a :class:`ScriptSettings` class pulls machine-dependent defaults from
``rbsimrc.py`` (window size, default layout and palette) and reconciles them
with the command line, precedence ``rc defaults < scenario < command line``
(ARCHITECTURE Section 7). Nothing the rc file supplies can change the
physics; that lives in the scenario alone.

The work is factored into a testable core, :func:`run_interactive_job`,
which builds the renderer and controls from the scenario and settings and
runs the session. It accepts an injected renderer and controls source, so
the whole wiring is exercised headlessly with fakes, and an offscreen preview
(``--frames N --offscreen``) renders a fixed number of frames with no input.

Live controls: space pauses and resumes, ``s`` single-steps, ``-`` slows,
``+`` speeds up, ``n`` returns to normal, and ``q`` quits.

WHERE THIS CODE LIVES, AND WHY. This module is the body of the ``rbsim``
command. It is inside the package, not in ``src/scripts/``, because the
command is reached in two ways that must run the same code (ARCHITECTURE
Sections 3.9 and 9.5): the executable ``src/scripts/rbsim.py``, which the
physdemo suite links and a clone runs directly; and the console script
that ``pip install`` creates from ``pyproject.toml``, which calls
:func:`console_main` below. Getting to a first run -- the packaged
examples, ``--check``, ``--write-rc`` -- is PSEUDOCODE Section 16; the
helpers it calls are ``cli/support.py``, INHERITED from the physdemo
skeleton (Section 16.2), which holds no command name and no dependency
list, so this module supplies ``COMMAND_NAME``,
``CHECKED_DISTRIBUTIONS``, and :func:`run_offscreen`.
"""

import argparse
import os
import sys

from rigid_body.cli.support import (
    copy_examples, copy_rc_file, load_rc_defaults, locate_run_file,
    record_command, self_check)
from rigid_body.scenario.serialization import load_scenario
from rigid_body.render.palettes import select_palette, LIGHT_PALETTE
from rigid_body.ui.vedo_controls import (
    VedoControlsSource, AutoControlsSource, control_legend_lines)
from rigid_body.ui.interactive_session import run_interactive_session

# The renderer is NOT imported here. VTK fixes its window class when it is
# first imported, and an offscreen run must choose that class first
# (PSEUDOCODE Section 16.4); so run_interactive_job decides, then imports.
# It also keeps ``--help`` and a mistyped path from paying for VTK's import.

COMMAND_NAME = "rbsim"
RC_FILENAME = "rbsimrc.py"

# The packages ``rbsim --check`` reports on: the ones this tool imports,
# by the names ``pip`` knows them by. pyproject.toml declares the same
# set (minus the Python 3.10 TOML backport), and a test keeps the two in
# agreement. It lives here and not in support.py because it differs per
# tool and the inherited module must not.
CHECKED_DISTRIBUTIONS = ("numpy", "scipy", "matplotlib", "vedo", "vtk",
                         "h5py", "pint", "tomli_w")


_BUILTIN_RC_DEFAULTS = {
    "window_width": 1280,
    "window_height": 960,
    "layout": "scenario",
    "palette": "scenario"}


def resolve_palette(palette_name):
    """Return the named palette, falling back to light if it is unknown.

    A scenario may name a palette the built-ins do not provide; rather than
    refuse to open the window, the interactive tier falls back to the light
    scheme, since the palette is presentation and never physics.
    """
    try:
        return select_palette(palette_name)
    except ValueError:
        return LIGHT_PALETTE


class _FrameSavingRenderer:
    """Wrap a renderer to write each drawn frame to a numbered PNG.

    This is how the interactive scene is captured without a live window
    (the headless path for a node with no display): it draws through the
    real renderer, then screenshots the frame to ``frame_00000.png`` and so
    on. The saved sequence can be viewed directly or assembled into a video
    with ffmpeg (ARCHITECTURE Section 9.1).
    """

    def __init__(self, inner_renderer, directory):
        self.inner_renderer = inner_renderer
        self.directory = directory
        os.makedirs(directory, exist_ok=True)
        self.frame_count = 0

    @property
    def plotter(self):
        return self.inner_renderer.plotter

    def render(self, scene, state):
        self.inner_renderer.render(scene, state)
        path = os.path.join(
            self.directory, f"frame_{self.frame_count:05d}.png")
        self.inner_renderer.screenshot(path)
        self.frame_count += 1

    def screenshot(self, path=None, as_array=False):
        return self.inner_renderer.screenshot(path, as_array)

    def close(self):
        self.inner_renderer.close()


def run_interactive_job(scenario_path, window_size=(1280, 960),
                        layout="scenario", palette="scenario",
                        offscreen=False, frames=None, screenshot=None,
                        save_frames=None, renderer=None,
                        controls_source=None):
    """Load a scenario, build the renderer and controls, and run the loop.

    The testable core of the script. ``layout`` and ``palette`` are either a
    concrete choice or ``"scenario"`` to take the value from the scenario's
    presentation zone. ``screenshot`` saves the final frame to a PNG, and
    ``save_frames`` saves every frame to a directory as an image sequence;
    either capture implies an offscreen run, since its purpose is to see the
    motion without a live window (the headless path for a node with no
    display). Without an injected renderer it builds a :class:`VedoRenderer`;
    without an injected controls source it uses live vedo controls, or an
    :class:`AutoControlsSource` when running offscreen or for a fixed frame
    count. Returns the controls source used.
    """
    scenario = load_scenario(scenario_path)
    presentation = scenario.presentation

    # Capturing to disk is a headless operation, so it forces offscreen
    # rendering -- a live GL window is neither needed nor wanted for it.
    capturing = screenshot is not None or save_frames is not None
    if capturing:
        offscreen = True

    own_renderer = renderer is None
    if renderer is None:
        if offscreen:
            # Before VTK is imported: EGL on Linux whatever DISPLAY says,
            # nothing elsewhere (PSEUDOCODE Section 16.4).
            from rigid_body.render.offscreen import prepare_offscreen
            prepare_offscreen()
        from rigid_body.render.vedo_renderer import VedoRenderer
        chosen_layout = (presentation.layout if layout == "scenario"
                         else layout)
        chosen_palette = resolve_palette(
            presentation.palette if palette == "scenario" else palette)
        renderer = VedoRenderer(
            chosen_palette, layout=chosen_layout, size=window_size,
            offscreen=offscreen, legend_lines=control_legend_lines())
    if save_frames is not None:
        renderer = _FrameSavingRenderer(renderer, save_frames)

    if controls_source is None:
        nominal_substeps = scenario.fidelity.substeps_per_frame
        if offscreen or frames is not None:
            controls_source = AutoControlsSource(
                nominal_substeps,
                max_frames=frames if frames is not None else 1)
        else:
            controls_source = VedoControlsSource(
                renderer.plotter, nominal_substeps)

    try:
        run_interactive_session(
            scenario, renderer, controls_source, max_frames=frames)
        if screenshot is not None:
            renderer.screenshot(screenshot)
    finally:
        if own_renderer:
            renderer.close()
    return controls_source


class ScriptSettings:
    """User settings, reconciled from the rc file and the command line."""

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
        self.window_width = default_rc["window_width"]
        self.window_height = default_rc["window_height"]
        self.layout = default_rc["layout"]
        self.palette = default_rc["palette"]
        self.scenario_path = None
        self.offscreen = False
        self.frames = None
        self.screenshot = None
        self.save_frames = None
        self.examples = None
        self.write_rc = False
        self.check = False

    def parse_command_line(self, command_line_args=None):
        """Build the parser and return the parsed arguments."""
        description_text = """
Launch the interactive rigid-body simulation from a scenario file: open a
vedo window and run the frame loop, drawing the body and space frames side
by side. Space pauses, s single-steps, - slows, + speeds up, n normal, q
quits.
"""
        epilog_text = """
Defaults are taken from ./rbsimrc.py or $RIGID_BODY_RC/rbsimrc.py (make the
first with --write-rc) and may be overridden on the command line. The
physics lives entirely in the scenario file; the rc file governs only the
window and the default presentation.

    rbsim --check                 can this computer run and draw it?
    rbsim dzhanibekov             a packaged example, by bare name
    rbsim --examples              copy the example scenarios here
    rbsim dzhanibekov.toml        your own (edited) copy
"""
        parser = argparse.ArgumentParser(
            prog="rbsim",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description=description_text, epilog=epilog_text)
        self.add_parser_arguments(parser)
        arguments = parser.parse_args(command_line_args)
        # Exactly one thing to do (PSEUDOCODE Section 16.3): a run, or
        # one of the utilities.
        requested = [arguments.scenario is not None,
                     arguments.examples is not None,
                     arguments.write_rc, arguments.check]
        if sum(requested) != 1:
            parser.error("give a scenario, or exactly one of --examples, "
                         "--write-rc, --check")
        return arguments

    def add_parser_arguments(self, parser):
        """Declare the interactive script's command-line arguments."""
        parser.add_argument(
            "scenario", nargs="?", default=None,
            help="Path to the scenario TOML file to run, or the bare name "
                 "of a packaged example (see --examples).")
        parser.add_argument(
            "--examples", dest="examples", nargs="?", const=".",
            default=None, metavar="DIR",
            help="Copy the packaged example scenarios into DIR (default: "
                 "here) and exit. Never overwrites a file already there.")
        parser.add_argument(
            "--write-rc", dest="write_rc", action="store_true",
            help="Copy the shipped rbsimrc.py here, to edit, and exit.")
        parser.add_argument(
            "--check", dest="check", action="store_true",
            help="Check that this computer can run and draw the tool, "
                 "and exit.")
        parser.add_argument(
            "--width", dest="window_width", type=int,
            default=self.window_width,
            help=f"Window width. Default: {self.window_width}")
        parser.add_argument(
            "--height", dest="window_height", type=int,
            default=self.window_height,
            help=f"Window height. Default: {self.window_height}")
        parser.add_argument(
            "--layout", dest="layout", default=self.layout,
            help="'side_by_side', 'single', or 'scenario'. "
                 f"Default: {self.layout}")
        parser.add_argument(
            "--palette", dest="palette", default=self.palette,
            help="'light', 'dark', 'color_blind_safe', or 'scenario'. "
                 f"Default: {self.palette}")
        parser.add_argument(
            "--frames", dest="frames", type=int, default=None,
            help="Render this many frames and exit, instead of running "
                 "until the window is closed.")
        parser.add_argument(
            "--offscreen", dest="offscreen", action="store_true",
            help="Render without a window (for a preview or a headless "
                 "node); implies an automatic fixed-length run.")
        parser.add_argument(
            "--screenshot", dest="screenshot", default=None,
            help="Save the final frame to this PNG path (implies "
                 "offscreen). A quick way to see a scenario with no "
                 "display.")
        parser.add_argument(
            "--save-frames", dest="save_frames", default=None,
            help="Save every frame as a numbered PNG in this directory "
                 "(implies offscreen); assemble into a video with ffmpeg.")

    def reconcile(self, arguments):
        """Let the command line override the rc defaults."""
        self.scenario_path = arguments.scenario
        self.window_width = arguments.window_width
        self.window_height = arguments.window_height
        self.layout = arguments.layout
        self.palette = arguments.palette
        self.frames = arguments.frames
        self.offscreen = arguments.offscreen
        self.screenshot = arguments.screenshot
        self.save_frames = arguments.save_frames
        self.examples = arguments.examples
        self.write_rc = arguments.write_rc
        self.check = arguments.check


def run_offscreen(scenario_path, frames):
    """Run ``scenario_path`` for ``frames`` frames offscreen THROUGH THE
    ORDINARY CODE PATH (:func:`run_interactive_job`) and return the last
    frame as an array. This is what ``--check`` runs;
    ``support.self_check`` calls ``prepare_offscreen()`` before it, so
    the renderer may be imported here. An injected renderer is not
    closed by ``run_interactive_job`` (it closes only a renderer it
    built), which is what lets the picture be read back afterwards."""
    from rigid_body.render.vedo_renderer import VedoRenderer
    renderer = VedoRenderer(LIGHT_PALETTE, size=(640, 480),
                            offscreen=True)
    run_interactive_job(scenario_path, frames=frames, offscreen=True,
                        renderer=renderer)
    image = renderer.screenshot(as_array=True)
    renderer.close()
    return image


def main(command_line_args=None):
    """Assemble the settings and run the interactive session, or one of
    the first-run utilities. Returns the exit status. Accepting an
    argument list lets the test suite drive this without ``sys.argv``."""
    settings = ScriptSettings(command_line_args)
    if settings.examples is not None:
        return copy_examples(settings.examples, COMMAND_NAME)
    if settings.write_rc:
        return copy_rc_file(RC_FILENAME, ".", COMMAND_NAME)
    if settings.check:
        return self_check(run_offscreen, COMMAND_NAME,
                          CHECKED_DISTRIBUTIONS)

    # A scenario that is missing or wrong is the commonest mistake a
    # student makes; it earns a message and status 2, not a traceback.
    try:
        scenario_path = locate_run_file(settings.scenario_path,
                                        COMMAND_NAME, noun="scenario")
        run_interactive_job(
            scenario_path,
            window_size=(settings.window_width, settings.window_height),
            layout=settings.layout, palette=settings.palette,
            offscreen=settings.offscreen, frames=settings.frames,
            screenshot=settings.screenshot,
            save_frames=settings.save_frames)
    except (FileNotFoundError, ValueError, KeyError) as problem:
        print(f"{COMMAND_NAME}: {problem}", file=sys.stderr)
        return 2
    if settings.screenshot is not None:
        print(f"Saved final frame to {settings.screenshot}")
    if settings.save_frames is not None:
        print(f"Saved frame sequence to {settings.save_frames}/")
    return 0


def console_main():
    """The front that ``pip install`` creates (pyproject.toml,
    ``[project.scripts]``). It is the real entry point on that route, so
    it is where the invocation is logged; ``main()`` itself never logs."""
    record_command()
    sys.exit(main())
