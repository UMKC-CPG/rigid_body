#!/usr/bin/env python3

"""Launch the interactive simulation (Tier 1, ARCHITECTURE Section 3.9).

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
"""

import argparse
import os
import sys
from datetime import datetime

# realpath, not abspath: this script is normally run through a
# symbolic link that the physdemo suite keeps to it, and the
# package must be found beside the real file, not beside the link.
# The package lives one directory up from this script (in ``src/``); put it
# on the path so ``rigid_body`` imports resolve when run directly.
_SOURCE_DIRECTORY = os.path.dirname(
    os.path.dirname(os.path.realpath(__file__)))
if _SOURCE_DIRECTORY not in sys.path:
    sys.path.insert(0, _SOURCE_DIRECTORY)

from rigid_body.scenario.serialization import load_scenario  # noqa: E402
from rigid_body.render.palettes import (                     # noqa: E402
    select_palette, LIGHT_PALETTE)
from rigid_body.render.vedo_renderer import VedoRenderer     # noqa: E402
from rigid_body.ui.vedo_controls import (                    # noqa: E402
    VedoControlsSource, AutoControlsSource, control_legend_lines)
from rigid_body.ui.interactive_session import (              # noqa: E402
    run_interactive_session)


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


def _load_rc_defaults():
    """Return the rc defaults, preferring a machine copy on $RIGID_BODY_RC."""
    rc_dir = os.getenv("RIGID_BODY_RC")
    if rc_dir and os.path.isfile(os.path.join(rc_dir, "rbsimrc.py")):
        sys.path.insert(1, rc_dir)
    try:
        from rbsimrc import parameters_and_defaults
    except ImportError:
        return dict(_BUILTIN_RC_DEFAULTS)
    return parameters_and_defaults()


class ScriptSettings:
    """User settings, reconciled from the rc file and the command line."""

    def __init__(self):
        default_rc = _load_rc_defaults()
        self.assign_rc_defaults(default_rc)
        arguments = self.parse_command_line()
        self.reconcile(arguments)
        self.record_command_line()

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

    def parse_command_line(self):
        """Build the parser and return the parsed arguments."""
        description_text = """
Launch the interactive rigid-body simulation from a scenario file: open a
vedo window and run the frame loop, drawing the body and space frames side
by side. Space pauses, s single-steps, - slows, + speeds up, n normal, q
quits.
"""
        epilog_text = """
Defaults are taken from ./rbsimrc.py or $RIGID_BODY_RC/rbsimrc.py and may be
overridden on the command line. The physics lives entirely in the scenario
file; the rc file governs only the window and the default presentation.
"""
        parser = argparse.ArgumentParser(
            prog="rbsim",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description=description_text, epilog=epilog_text)
        self.add_parser_arguments(parser)
        return parser.parse_args()

    def add_parser_arguments(self, parser):
        """Declare the interactive script's command-line arguments."""
        parser.add_argument(
            "scenario", help="Path to the scenario TOML file to run.")
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


def main():
    """Assemble the settings and run the interactive session."""
    settings = ScriptSettings()
    run_interactive_job(
        settings.scenario_path,
        window_size=(settings.window_width, settings.window_height),
        layout=settings.layout, palette=settings.palette,
        offscreen=settings.offscreen, frames=settings.frames,
        screenshot=settings.screenshot, save_frames=settings.save_frames)
    if settings.screenshot is not None:
        print(f"Saved final frame to {settings.screenshot}")
    if settings.save_frames is not None:
        print(f"Saved frame sequence to {settings.save_frames}/")


if __name__ == "__main__":
    main()
