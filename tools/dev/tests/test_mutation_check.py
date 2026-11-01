"""mutation_check.py: the Hard rule 1 harness must never leave a file broken, and must
only call a break caught when a test actually failed.

Each scene builds a tiny module and test in a temporary folder, so the verdict is known.
Run by: python3 -m pytest tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ tools/dev/tests/ -q
"""
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import mutation_check as mc  # noqa: E402

RUNNER = f"{sys.executable} -m pytest -q -p no:cacheprovider"

MODULE = "def double(x):\n    return 2 * x\n"
TEST = "from mod import double\n\ndef test_double():\n    assert double(3) == 6\n"


@pytest.fixture
def scene(tmp_path):
    (tmp_path / "mod.py").write_text(MODULE)
    (tmp_path / "test_mod.py").write_text(TEST)
    return tmp_path


def run(scene, old, new):
    item = mc.Break("b", "mod.py", old, new, "test_mod.py")
    return mc.run_one(item, RUNNER, scene, scene / "backup")


def test_a_break_the_test_sees_is_caught_and_the_file_restored(scene):
    out = run(scene, "return 2 * x", "return 3 * x")
    assert out.verdict == "caught"
    assert (scene / "mod.py").read_text() == MODULE
    assert not any((scene / "backup").iterdir())


def test_a_break_the_test_cannot_see_is_missed(scene):
    out = run(scene, "return 2 * x", "return x * 2")
    assert out.verdict == "MISSED"
    assert (scene / "mod.py").read_text() == MODULE


def test_a_break_that_stops_collection_is_an_error_not_a_catch(scene):
    out = run(scene, "return 2 * x", "return 2 *")
    assert out.verdict == "ERROR"
    assert (scene / "mod.py").read_text() == MODULE


@pytest.mark.parametrize("old", ["not in the file", "x"])
def test_old_text_must_occur_exactly_once(scene, old):
    out = run(scene, old, "y")
    assert out.verdict == "NOT APPLIED"
    assert (scene / "mod.py").read_text() == MODULE


def test_the_file_is_restored_even_if_the_runner_blows_up(scene, monkeypatch):
    def boom(*args, **kwargs):
        raise KeyboardInterrupt
    monkeypatch.setattr(mc.subprocess, "run", boom)
    with pytest.raises(KeyboardInterrupt):
        run(scene, "return 2 * x", "return 3 * x")
    assert (scene / "mod.py").read_text() == MODULE


def test_load_refuses_a_break_with_a_missing_field(tmp_path):
    spec = tmp_path / "b.yaml"
    spec.write_text(yaml.safe_dump([{"name": "n", "file": "f", "old": "a", "new": "b"}]))
    with pytest.raises(ValueError, match="missing test"):
        mc.load(spec)


def test_main_exits_nonzero_when_anything_is_not_caught(scene):
    spec = scene / "b.yaml"
    spec.write_text(yaml.safe_dump([
        {"name": "caught", "file": "mod.py", "old": "return 2 * x", "new": "return 3 * x", "test": "test_mod.py"},
        {"name": "missed", "file": "mod.py", "old": "return 2 * x", "new": "return x * 2", "test": "test_mod.py"}]))
    assert mc.main([str(spec), "--runner", RUNNER, "--root", str(scene)]) == 1
    assert mc.main([str(spec), "--runner", RUNNER, "--root", str(scene), "--only", "caught"]) == 0
    assert (scene / "mod.py").read_text() == MODULE
