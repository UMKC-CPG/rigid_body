"""What the two commands share: the rc lookup, the command log, the
packaged examples, and the self-check (PSEUDOCODE Section 16.2; DESIGN
Section 15).

A student who installed the tool with ``pip``, or who is using a shared
installation that someone else made and that they cannot write to, has no
idea where the example scenarios or the rc files are, and should not need
to. Everything in this module therefore finds its files THROUGH THE
PACKAGE, never relative to a script or to the working directory. That one
rule is what makes a clone, a linked ``physdemo`` suite, and an installed
copy behave identically (ARCHITECTURE Section 9.5).

Nothing here touches the physics. These are ways of getting to a run.

Attribution: this module is part of the rigid_body teaching tool. Its
design follows the companion ``scattering`` tool's ``cli/examples.py``.
"""

import importlib.util
import os
import platform
import shutil
import sys
from datetime import datetime
from importlib import metadata, resources
from pathlib import Path

# The shipped rc files, located from this module's own resolved position
# so that they are found however the package was obtained.
PACKAGE_DEFAULTS_DIR = Path(__file__).resolve().parents[1] / "defaults"

# Invocations that are not runs, and so are not logged to ``command``
# (DESIGN Section 15.3): asking for help, copying files, checking.
UTILITY_FLAGS = ("-h", "--help", "--examples", "--write-rc", "--check")

# The packages ``rbsim --check`` reports on: the ones this tool imports,
# by the names ``pip`` knows them by. pyproject.toml declares the same
# set, and a test keeps the two in agreement.
CHECKED_DISTRIBUTIONS = ("numpy", "scipy", "matplotlib", "vedo", "vtk",
                         "h5py", "pint", "tomli_w")


def rc_search_path():
    """The directories searched for an rc file, in order: the working
    directory, so that a user can keep a modified copy beside their data;
    ``$RIGID_BODY_RC``, for a machine-wide copy; and the package, which
    always has one (DESIGN Section 15.2)."""
    candidates = [Path.cwd()]
    machine_directory = os.getenv("RIGID_BODY_RC")
    if machine_directory:
        candidates.append(Path(machine_directory))
    candidates.append(PACKAGE_DEFAULTS_DIR)
    return candidates


def load_rc_defaults(rc_filename, builtin_defaults, search_path=None):
    """Return the rc defaults from the first ``rc_filename`` found.

    The file is loaded BY PATH from an explicit list. The earlier lookup
    imported it by name through ``sys.path``, which holds the script's
    directory and not the working directory, so the ``./rbsimrc.py`` that
    the help text promised was never actually read. ``builtin_defaults``
    is the fallback of last resort, so that a damaged installation still
    starts.
    """
    for directory in (search_path or rc_search_path()):
        candidate = Path(directory) / rc_filename
        if candidate.is_file():
            specification = importlib.util.spec_from_file_location(
                candidate.stem, candidate)
            module = importlib.util.module_from_spec(specification)
            specification.loader.exec_module(module)
            return dict(module.parameters_and_defaults())
    return dict(builtin_defaults)


def record_command():
    """Append the invocation to a ``command`` log, as the idiom does.

    Called by the two fronts of a command -- the executable script and the
    console script -- and never by ``main()``, so that the test suite can
    call ``main(argv)`` without leaving ``command`` files behind.

    The log is a convenience and must never stop a run. The usual way to
    fail here is a student standing inside a shared, read-only
    installation, among the example scenarios; say so in one line and
    carry on (CLAUDE.md, "Command Logging").
    """
    if any(argument in UTILITY_FLAGS for argument in sys.argv):
        return
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


def example_files():
    """Every packaged example scenario, as a dictionary from its bare name
    (``dzhanibekov``) to its path, in name order."""
    directory = Path(str(resources.files("rigid_body.examples")))
    return {path.stem: path for path in sorted(directory.glob("*.toml"))}


def locate_scenario(argument, command_name="rbsim"):
    """Turn the command line's scenario argument into a path.

    A file that exists is returned as it is; a real file always wins.
    Otherwise a BARE name (no directory part) that matches a packaged
    example, with or without ``.toml``, selects that example, and one line
    on standard error says so. The rule is deliberately narrow:
    ``rbsim results/dzhanibekov.toml`` with a mistyped directory must fail
    rather than quietly run something else.

    Raises FileNotFoundError with a message a student can act on.
    """
    if Path(argument).exists():
        return Path(argument)
    examples = example_files()
    if Path(argument).name == argument:
        stem = (argument[:-len(".toml")] if argument.endswith(".toml")
                else argument)
        if stem in examples:
            print(f"using the packaged example {examples[stem]}",
                  file=sys.stderr)
            return examples[stem]
    names = ", ".join(examples) or "(none found)"
    raise FileNotFoundError(
        f"{argument}: no such scenario. Packaged examples: {names}. Run "
        f"one by name ({command_name} {next(iter(examples), 'NAME')}), or "
        "copy them here to edit with: rbsim --examples")


def copy_without_overwriting(sources, directory):
    """Copy each source file into ``directory``, never replacing a file
    that is already there, and say what happened to each. Returns an exit
    status: 0, or 1 if the directory cannot be written.

    Not overwriting is the point: a student's edited copy is worth more
    than a fresh one, and a second ``rbsim --examples`` must be harmless.
    """
    directory = Path(directory)
    try:
        directory.mkdir(parents=True, exist_ok=True)
        for source in sources:
            target = directory / Path(source).name
            if target.exists():
                print(f"kept   {target} (already here)")
            else:
                shutil.copyfile(source, target)
                print(f"wrote  {target}")
    except OSError as problem:
        print(f"cannot write in {directory} ({problem.strerror}). Choose "
              "a directory you can write, for example:\n"
              "    rbsim --examples ~/rigid-body-runs", file=sys.stderr)
        return 1
    return 0


def copy_examples(directory):
    """``rbsim --examples [DIR]``: the packaged scenarios, to edit."""
    return copy_without_overwriting(example_files().values(), directory)


def copy_rc_file(rc_filename, directory="."):
    """``--write-rc``: a command's shipped rc defaults, to edit. The
    working directory is the first place the rc file is looked for, so the
    copy takes effect where it lands."""
    return copy_without_overwriting([PACKAGE_DEFAULTS_DIR / rc_filename],
                                    directory)


def installed_versions():
    """The version of each checked distribution, or None if missing."""
    versions = {}
    for name in CHECKED_DISTRIBUTIONS:
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def self_check(run_interactive_job):
    """``rbsim --check``: can this computer run the tool and draw it?

    Reports the versions in use, runs the packaged ``dzhanibekov`` scenario
    for a few frames offscreen THROUGH THE ORDINARY CODE PATH
    (``run_interactive_job``, passed in so that this module need not import
    the one that imports it), and reads the picture back. The read-back
    matters: a window without a working OpenGL context accepts every draw
    call and draws nothing (ARCHITECTURE Section 9.3), so "it did not
    crash" proves nothing about the picture.

    Writes no file. Returns 0 and prints ``RESULT: PASS``, or returns 1 and
    prints ``RESULT: FAIL -- <reason>``. Every exception is caught on
    purpose: this function's one job is to turn whatever goes wrong on an
    unfamiliar computer into a line a student can send on.
    """
    import time
    print(f"python      {platform.python_version()}  ({sys.executable})")
    print(f"platform    {platform.system()} {platform.release()} "
          f"{platform.machine()}")
    versions = installed_versions()
    for name, version in versions.items():
        print(f"{name:<11} {version or 'MISSING'}")
    missing = [name for name, version in versions.items() if not version]
    if missing:
        print(f"RESULT: FAIL -- not installed: {', '.join(missing)}")
        return 1

    try:
        # Decide VTK's window class BEFORE the renderer is imported.
        from rigid_body.render.offscreen import prepare_offscreen
        prepare_offscreen()
        from rigid_body.render.palettes import LIGHT_PALETTE
        from rigid_body.render.vedo_renderer import VedoRenderer

        started = time.perf_counter()
        renderer = VedoRenderer(LIGHT_PALETTE, size=(640, 480),
                                offscreen=True)
        # An injected renderer is not closed by run_interactive_job, which
        # is what lets the picture be read back afterwards.
        run_interactive_job(example_files()["dzhanibekov"], frames=3,
                            offscreen=True, renderer=renderer)
        image = renderer.screenshot(as_array=True)
        renderer.close()
        print(f"drew        3 frames offscreen in "
              f"{time.perf_counter() - started:.1f} s")
    except Exception as problem:                  # noqa: BLE001 (see above)
        print(f"RESULT: FAIL -- {type(problem).__name__}: {problem}")
        return 1

    if image.min() == image.max():
        print("RESULT: FAIL -- the picture is blank: no working OpenGL "
              "context for offscreen drawing on this computer")
        return 1
    print("RESULT: PASS")
    return 0
