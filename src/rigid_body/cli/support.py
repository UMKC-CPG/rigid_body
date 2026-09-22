"""What every command of the tool shares: the rc lookup, the command
log, the packaged examples, the rc copy, and the self-check (physdemo
PSEUDOCODE 4.5). INHERITED from the physdemo member-tool skeleton;
a change is made there and carried here.

A student who installed the tool with `pip`, or who is using a shared
installation that someone else made and that they cannot write to,
has no idea where the example run files or the rc file are, and
should not need to. Everything in this module therefore finds its
files THROUGH THE PACKAGE, never relative to a script or to the
working directory. That one rule is what makes a clone, a linked
`physdemo` suite, and an installed copy behave identically.

Nothing here touches the physics. These are ways of getting to a run.

Attribution: this module is inherited from the physdemo suite's
member-tool skeleton (github.com/UMKC-CPG/physdemo), which
reconciled the `rigid_body` and `scattering` tools' copies of it; it
is part of the Rigid Body teaching tool.
"""

import importlib.util
import os
import platform
import shutil
import sys
import time
from datetime import datetime
from importlib import metadata, resources
from pathlib import Path

# The shipped rc files, located from this module's own resolved
# position so that they are found however the package was obtained.
PACKAGE_DEFAULTS_DIR = Path(__file__).resolve().parents[1] / 'defaults'

# The environment variable naming a machine-wide rc directory.
RC_ENVIRONMENT_VARIABLE = 'RIGID_BODY_RC'

# Invocations that are not runs, and so are not logged to `command`:
# asking for help, copying files, checking the machine.
UTILITY_FLAGS = ('-h', '--help', '--examples', '--write-rc', '--check')

# Nothing here names a command or the distributions a self-check
# reports: those differ per tool and per command, and this file must
# not, so the command module passes them in (physdemo PSEUDOCODE 4.7).


def rc_search_path():
    """The directories searched for an rc file, in order: the working
    directory, so that a user can keep a modified copy beside their
    data; the directory named by the rc environment variable, for a
    machine-wide copy; and the package, which always has one."""
    candidates = [Path.cwd()]
    machine_directory = os.getenv(RC_ENVIRONMENT_VARIABLE)
    if machine_directory:
        candidates.append(Path(machine_directory))
    candidates.append(PACKAGE_DEFAULTS_DIR)
    return candidates


def load_rc_defaults(rc_filename, builtin_defaults, search_path=None):
    """Return the rc defaults from the first `rc_filename` found.

    The file is loaded BY PATH from an explicit list, never imported
    by name through `sys.path`, which holds the script's directory
    and not the working directory. `builtin_defaults` is the fallback
    of last resort, so that a damaged installation still starts.
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
    """Append the invocation to a `command` log in the working
    directory, as a dated block naming the full argument vector.

    Called by the two fronts of a command, the executable script and
    the console script, and never by `main()`, so that the test
    suite can call `main(argv)` without leaving `command` files
    behind. Utility invocations are not runs and are not logged.

    The log is a convenience and must never stop a run. The usual
    way to fail here is a student standing inside a shared,
    read-only installation, among the example run files; say so in
    one line and carry on (physdemo contract C6, C16).
    """
    if any(argument in UTILITY_FLAGS for argument in sys.argv):
        return
    try:
        with open('command', 'a') as command_log:
            stamp = datetime.now().strftime('%b. %d, %Y: %H:%M:%S')
            command_log.write(f'Date: {stamp}\n')
            command_log.write('Cmnd:')
            for argument in sys.argv:
                command_log.write(f' {argument}')
            command_log.write('\n\n')
    except OSError as problem:
        print(f'note: cannot write ./command here ({problem.strerror}); '
              'continuing without the command log', file=sys.stderr)


def example_files():
    """Every packaged example run file, as a dictionary from its bare
    name (`circle`) to its path, in name order."""
    directory = Path(str(resources.files('rigid_body.examples')))
    return {path.stem: path for path in sorted(directory.glob('*.toml'))}


def locate_run_file(argument, command_name, noun='run file'):
    """Turn the command line's run-file argument into a path. `noun`
    is what the tool calls its input in messages ("scenario" in the
    rigid-body tool).

    A file that exists is returned as it is; a real file always wins.
    Otherwise a BARE name (no directory part) that matches a packaged
    example, with or without `.toml`, selects that example, and one
    line on standard error says so. The rule is deliberately narrow:
    `<command> results/circle.toml` with a mistyped directory must
    fail rather than quietly run something else.

    Raises FileNotFoundError with a message a student can act on.
    """
    if Path(argument).exists():
        return Path(argument)
    examples = example_files()
    if Path(argument).name == argument:
        stem = (argument[:-len('.toml')] if argument.endswith('.toml')
                else argument)
        if stem in examples:
            print(f'using the packaged example {examples[stem]}',
                  file=sys.stderr)
            return examples[stem]
    names = ', '.join(examples) or '(none found)'
    raise FileNotFoundError(
        f'{argument}: no such {noun}. Packaged examples: {names}. Run '
        f'one by name ({command_name} {next(iter(examples), "NAME")}), '
        f'or copy them here to edit with: {command_name} --examples')


def copy_without_overwriting(sources, directory, command_name):
    """Copy each source file into `directory`, never replacing a file
    that is already there, and say what happened to each. Returns an
    exit status: 0, or 1 if the directory cannot be written.

    Not overwriting is the point: a student's edited copy is worth
    more than a fresh one, and a second `--examples` must be harmless.
    """
    directory = Path(directory)
    try:
        directory.mkdir(parents=True, exist_ok=True)
        for source in sources:
            target = directory / Path(source).name
            if target.exists():
                print(f'kept   {target} (already here)')
            else:
                shutil.copyfile(source, target)
                print(f'wrote  {target}')
    except OSError as problem:
        print(f'cannot write in {directory} ({problem.strerror}). Choose '
              'a directory you can write, for example:\n'
              f'    {command_name} --examples ~/rigid_body-runs',
              file=sys.stderr)
        return 1
    return 0


def copy_examples(directory, command_name):
    """`--examples [DIR]`: the packaged run files, to edit."""
    return copy_without_overwriting(example_files().values(), directory,
                                    command_name)


def copy_rc_file(rc_filename, directory, command_name):
    """`--write-rc`: a command's shipped rc defaults, to edit. The
    working directory is the first place the rc file is looked for,
    so the copy takes effect where it lands."""
    return copy_without_overwriting([PACKAGE_DEFAULTS_DIR / rc_filename],
                                    directory, command_name)


def installed_versions(checked_distributions):
    """The version of each checked distribution, or None if missing."""
    versions = {}
    for name in checked_distributions:
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def self_check(run_offscreen, command_name, checked_distributions):
    """`--check`: can this computer run the tool and draw it?

    Reports the versions of `checked_distributions` (the command
    module's list, which pyproject.toml declares and a test keeps in
    agreement), runs the first packaged example for two frames
    offscreen THROUGH THE ORDINARY CODE PATH
    (`run_offscreen(run_file_path, frames)`, passed in so that this
    module never imports the module that imports it), and reads the
    picture back. The read-back matters: a window without a working
    OpenGL context accepts every draw call and draws nothing, so "it
    did not crash" proves nothing about the picture.

    Writes no file. Returns 0 and prints `RESULT: PASS`, or returns 1
    and prints `RESULT: FAIL -- <reason>`. Every exception is caught
    on purpose: this function's one job is to turn whatever goes
    wrong on an unfamiliar computer into a line a student can send on.
    """
    print(f'python      {platform.python_version()}  ({sys.executable})')
    print(f'platform    {platform.system()} {platform.release()} '
          f'{platform.machine()}')
    versions = installed_versions(checked_distributions)
    for name, version in versions.items():
        print(f'{name:<11} {version or "MISSING"}')
    missing = [name for name, version in versions.items() if not version]
    if missing:
        print(f'RESULT: FAIL -- not installed: {", ".join(missing)}')
        return 1

    try:
        # Decide VTK's window class BEFORE the renderer is imported.
        from rigid_body.render.offscreen import prepare_offscreen
        prepare_offscreen()
        started = time.perf_counter()
        image = run_offscreen(next(iter(example_files().values())),
                              frames=2)
        print(f'drew        2 frames offscreen in '
              f'{time.perf_counter() - started:.1f} s')
    except Exception as problem:                  # noqa: BLE001 (see above)
        print(f'RESULT: FAIL -- {type(problem).__name__}: {problem}')
        return 1

    if image.min() == image.max():
        print('RESULT: FAIL -- the picture is blank: no working OpenGL '
              'context for offscreen drawing on this computer')
        return 1
    print('RESULT: PASS')
    return 0
