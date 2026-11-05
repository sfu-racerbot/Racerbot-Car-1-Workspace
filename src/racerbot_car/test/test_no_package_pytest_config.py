"""No test suite below the workspace root may carry its own pytest config.

pytest's rootdir is the nearest directory, from a suite upward, holding ``pytest.ini`` or
``.pytest.ini`` (even empty), ``tox.ini`` with ``[pytest]``, ``setup.cfg`` with
``[tool:pytest]`` or ``pyproject.toml`` with ``[tool.pytest.ini_options]`` (``[tool.pytest]``
from pytest 9); https://docs.pytest.org/en/stable/reference/customize.html#finding-the-rootdir.
Such a file inside a package makes that package its own rootdir, so pytest never loads the
workspace ``conftest.py``, and that package's tests see the car's real Jetson serial and USB
devices (CLAUDE.md, ``docs/car-profiles.md#tests-on-a-car``). Measured 2026-11-02:
``src/opponent_perception/setup.cfg``'s ``[tool:pytest]`` made that package its own rootdir
and the workspace fixture never ran there.
"""
import os
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]
PRUNE = {'.git', 'node_modules', '__pycache__', '.pytest_cache', 'site-packages'}
ALWAYS = {'pytest.ini', '.pytest.ini'}
SECTION = {
    'tox.ini': re.compile(r'^\[pytest\]\s*$', re.M),
    'setup.cfg': re.compile(r'^\[tool:pytest\]\s*$', re.M),
    'pyproject.toml': re.compile(r'^\[tool\.pytest(?:\.ini_options)?\]\s*$', re.M),
}
# A config that cannot be removed, with why its suites can never reach a car probe. The reason is
# checked by test_the_allowed_configs_hold_code_that_never_imports_racerbot_car.
ALLOWED = {
    'src/f1tenth_system/teleop_tools/pytest.ini':
        'upstream f1tenth/teleop_tools file (junit_family only), vendored unchanged; '
        'joy_teleop, key_teleop and mouse_teleop never import racerbot_car',
}


def find_pytest_configs(root):
    """Paths, relative to root, of every file under src/ and tools/ that pytest takes as config."""
    found = []
    for base in ('src', 'tools'):
        for dirpath, dirnames, filenames in os.walk(root / base):
            dirnames[:] = [d for d in dirnames if d not in PRUNE and not d.startswith('.venv')]
            for name in filenames:
                path = Path(dirpath) / name
                if name in ALWAYS or (
                        name in SECTION
                        and SECTION[name].search(path.read_text(errors='replace'))):
                    found.append(path.relative_to(root).as_posix())
    return sorted(found)


def test_no_suite_below_the_workspace_root_has_its_own_pytest_config():
    found = find_pytest_configs(ROOT)
    assert found == sorted(ALLOWED), (
        'pytest config below the workspace root (it drops the workspace conftest.py that hides '
        "the car's serial and USB devices): register markers from the package's conftest.py "
        'with config.addinivalue_line instead. Found: ' + ', '.join(found))


def _rootdir_of(suite):
    out = subprocess.run(
        [sys.executable, '-m', 'pytest', '--collect-only', '-p', 'no:cacheprovider', str(suite)],
        capture_output=True, text=True, timeout=60, env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
    match = re.search(r'^rootdir: (.+?)(?:, configfile:.*)?$', out.stdout, re.M)
    assert match, out.stdout + out.stderr
    return Path(match.group(1))


def test_the_finder_reports_exactly_the_files_pytest_takes_as_a_packages_rootdir(tmp_path):
    # oracle: pytest itself. A tree shaped like the workspace (an empty [pytest] at the top) holds
    # packages with and without config; whichever package pytest makes its own rootdir must be
    # exactly the set the finder reports.
    (tmp_path / 'pytest.ini').write_text('[pytest]\n')
    packages = {
        'src/ini/pytest.ini': '',
        'src/dotini/.pytest.ini': '',
        'src/tox/tox.ini': '[pytest]\njunit_family = xunit2\n',
        'src/cfg/setup.cfg': '[develop]\nscript_dir = x\n[tool:pytest]\ntestpaths = test\n',
        'tools/toml/pyproject.toml': '[tool.pytest.ini_options]\nminversion = "6.0"\n',
        'src/plaincfg/setup.cfg': '[develop]\nscript_dir = x\n[install]\ninstall_scripts = x\n',
        'src/plaintox/tox.ini': '[tox]\nenvlist = py3\n',
        'tools/plaintoml/pyproject.toml': '[tool.black]\nline-length = 99\n',
        'src/nopkgconfig/setup.py': 'from setuptools import setup\nsetup()\n',
    }
    for rel, text in packages.items():
        path = tmp_path / rel
        (path.parent / 'test').mkdir(parents=True)
        path.write_text(text)
        (path.parent / 'test' / 'test_x.py').write_text('def test_x():\n    pass\n')
    own_rootdir = {
        rel for rel in packages
        if _rootdir_of(tmp_path / Path(rel).parent / 'test') == (tmp_path / rel).parent}
    every_version = {'src/cfg/setup.cfg', 'src/ini/pytest.ini', 'src/tox/tox.ini',
                     'tools/toml/pyproject.toml'}
    # Newer pytest also reads .pytest.ini; 7.4.4, Jazzy's apt pytest on both cars, does not
    # (measured 2026-11-02). The finder flags it either way, since any machine may run a newer one.
    newer_only = {'src/dotini/.pytest.ini'}
    assert every_version <= own_rootdir <= every_version | newer_only
    found = find_pytest_configs(tmp_path)
    assert own_rootdir <= set(found)
    assert found == sorted(every_version | newer_only)


def test_the_finder_skips_vendored_trees_and_flags_pytest_9_tables(tmp_path):
    # oracle: pytest 9 reads a native [tool.pytest] table (docs link above); a config inside
    # node_modules, .git or a virtualenv is never a suite of this workspace.
    (tmp_path / 'tools/native').mkdir(parents=True)
    (tmp_path / 'tools/native/pyproject.toml').write_text('[tool.pytest]\nminversion = "9.0"\n')
    for skipped in ('src/a/node_modules/b', 'src/a/.git/c', 'tools/.venv-perception/d',
                    'src/a/lib/site-packages/e'):
        (tmp_path / skipped).mkdir(parents=True)
        (tmp_path / skipped / 'pytest.ini').write_text('[pytest]\n')
    assert find_pytest_configs(tmp_path) == ['tools/native/pyproject.toml']
    assert find_pytest_configs(tmp_path / 'empty') == []


def test_the_allowed_configs_hold_code_that_never_imports_racerbot_car():
    # oracle: invariant. An allowed config's suites run without the workspace fixture, which is
    # safe only while nothing under it can resolve the car (racerbot_car's identity, hardware and
    # layers modules are the only probes of the serial and USB devices).
    for rel, reason in ALLOWED.items():
        assert reason
        tree = (ROOT / rel).parent
        assert (ROOT / rel).is_file(), rel
        users = [p.relative_to(ROOT).as_posix() for p in tree.rglob('*.py')
                 if re.search(r'\bracerbot_car\b', p.read_text(errors='replace'))]
        assert users == [], f'{rel} is allowed only while nothing under it uses racerbot_car: {users}'
