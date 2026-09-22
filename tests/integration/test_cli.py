"""Verifies PSEUDOCODE Section 16 (with the inherited
``cli/support.py`` of 16.2): the packaged examples, the rc lookup,
the command log, the self-check, and errors that are messages. Everything
here must hold in a clone, in a linked suite, and in an installed copy, so
nothing here looks for a file except through the package."""

import os
import sys
from pathlib import Path

import pytest

from rigid_body.cli import rbbatch, rbsim, support
from rigid_body.cli.support import PACKAGE_DEFAULTS_DIR
from rigid_body.scenario.serialization import load_scenario

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def read_only_directory(tmp_path):
    """A directory the test cannot write, as a student finds a shared
    installation. Skips where permissions do not bind (root)."""
    locked = tmp_path / "shared"
    locked.mkdir()
    locked.chmod(0o555)
    if os.access(locked, os.W_OK):
        pytest.skip("this user can write a read-only directory")
    yield locked
    locked.chmod(0o755)


def test_packaged_examples_load_and_match_scenarios():
    packaged = support.example_files()
    assert "dzhanibekov" in packaged
    for path in packaged.values():
        load_scenario(path)                       # raises if one is broken
    assert {p.name for p in (REPO / "scenarios").glob("*.toml")} == \
        {p.name for p in packaged.values()}


def test_locate_run_file_with_the_scenario_noun(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    packaged = support.example_files()["dzhanibekov"]
    locate = support.locate_run_file
    assert locate(str(packaged), "rbsim", noun="scenario") == packaged
    assert locate("dzhanibekov", "rbsim", noun="scenario") == packaged
    assert locate("dzhanibekov.toml", "rbsim", noun="scenario") == packaged
    assert "using the packaged example" in capsys.readouterr().err
    (tmp_path / "dzhanibekov.toml").write_text("")
    assert locate("dzhanibekov.toml", "rbsim", noun="scenario") == \
        Path("dzhanibekov.toml")                  # a local file wins
    for wrong in ("sub/dzhanibekov", "no_such_example"):
        with pytest.raises(FileNotFoundError,
                           match="no such scenario.*free_tumble"):
            locate(wrong, "rbsim", noun="scenario")


def test_copy_examples_never_overwrites(tmp_path, capsys):
    target = tmp_path / "my runs"
    assert support.copy_examples(target, "rbsim") == 0
    names = sorted(p.name for p in target.iterdir())
    assert names == sorted(p.name for p in support.example_files().values())
    edited = target / "dzhanibekov.toml"
    edited.write_text("# my edit\n")
    capsys.readouterr()
    assert support.copy_examples(target, "rbsim") == 0
    assert edited.read_text() == "# my edit\n"
    assert capsys.readouterr().out.count("kept") == len(names)


def test_copy_into_a_read_only_directory_is_a_message(read_only_directory,
                                                      capsys):
    assert support.copy_examples(read_only_directory, "rbsim") == 1
    assert "Choose a directory you can write" in capsys.readouterr().err


def test_rc_lookup_order(tmp_path, monkeypatch):
    package = support.load_rc_defaults("rbsimrc.py", {})
    assert package == support.load_rc_defaults(
        "rbsimrc.py", {}, [PACKAGE_DEFAULTS_DIR])
    machine = tmp_path / "machine"
    machine.mkdir()
    (machine / "rbsimrc.py").write_text(
        "def parameters_and_defaults():\n    return {'window_width': 1}\n")
    working = tmp_path / "work"
    working.mkdir()
    (working / "rbsimrc.py").write_text(
        "def parameters_and_defaults():\n    return {'window_width': 2}\n")
    monkeypatch.setenv("RIGID_BODY_RC", str(machine))
    monkeypatch.chdir(tmp_path)                   # no rc file here
    assert support.load_rc_defaults("rbsimrc.py", {})["window_width"] == 1
    monkeypatch.chdir(working)
    assert support.load_rc_defaults("rbsimrc.py", {})["window_width"] == 2
    assert support.load_rc_defaults("rbsimrc.py", {"built": "in"},
                                    [tmp_path]) == {"built": "in"}


def test_written_rc_files_load_and_equal_the_defaults(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert rbsim.main(["--write-rc"]) == 0
    assert rbbatch.main(["--write-rc"]) == 0
    for name in ("rbsimrc.py", "rbbatchrc.py"):
        assert support.load_rc_defaults(name, {}, [tmp_path]) == \
            support.load_rc_defaults(name, {}, [PACKAGE_DEFAULTS_DIR])


@pytest.mark.parametrize("command", [rbsim, rbbatch])
def test_missing_scenario_is_status_2_not_a_traceback(command, tmp_path,
                                                      monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    assert command.main(["no_such.toml"]) == 2
    assert "Packaged examples" in capsys.readouterr().err


def test_broken_scenario_is_status_2(tmp_path, capsys):
    broken = tmp_path / "broken.toml"
    broken.write_text("[body]\nkind = 'no_such_kind'\n")
    assert rbsim.main([str(broken)]) == 2
    assert "rbsim:" in capsys.readouterr().err


@pytest.mark.parametrize("arguments", [[], ["x.toml", "--examples"],
                                       ["--check", "--write-rc"]])
def test_rbsim_does_exactly_one_thing(arguments):
    with pytest.raises(SystemExit) as refusal:
        rbsim.main(arguments)
    assert refusal.value.code == 2


@pytest.mark.parametrize("flag", ["--examples", "--write-rc", "--check",
                                  "--help"])
def test_utility_invocations_are_not_logged(tmp_path, monkeypatch, flag):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(sys, "argv", ["rbsim", flag])
    support.record_command()
    assert not (tmp_path / "command").exists()


def test_command_log_in_a_read_only_directory_is_only_a_note(
        read_only_directory, monkeypatch, capsys):
    monkeypatch.chdir(read_only_directory)
    monkeypatch.setattr(sys, "argv", ["rbsim", "dzhanibekov"])
    support.record_command()                      # must not raise
    assert "continuing without the command log" in capsys.readouterr().err


def test_self_check_passes_and_writes_nothing(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    status = rbsim.main(["--check"])
    output = capsys.readouterr().out
    if "the picture is blank" in output:
        pytest.skip("no offscreen GL context on this computer")
    assert status == 0 and "RESULT: PASS" in output
    assert list(tmp_path.iterdir()) == []


def test_batch_by_bare_name_writes_beside_the_working_directory(
        tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert rbbatch.main(["dzhanibekov", "--no-xdmf"]) == 0
    assert (tmp_path / "dzhanibekov.h5").is_file()
