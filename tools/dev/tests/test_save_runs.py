"""Run the real save script in disposable repositories; Task 7 is the oracle."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]


def git(repo, *args):
    return subprocess.check_output(['git', *args], cwd=repo, text=True).strip()


@pytest.fixture
def repo(tmp_path):
    (tmp_path / 'tools').mkdir()
    shutil.copy(ROOT / 'tools/save_runs.sh', tmp_path / 'tools/save_runs.sh')
    # Real identity resolver; empty profiles make this independent of host serial.
    shutil.copytree(ROOT / 'src/racerbot_car', tmp_path / 'src/racerbot_car',
                    ignore=shutil.ignore_patterns('__pycache__'))
    (tmp_path / 'cars').mkdir()
    (tmp_path / 'cars/machines.yaml').write_text('{}\n')
    for car in ('rb1', 'rb2'):
        (tmp_path / 'cars' / car).mkdir()
        (tmp_path / 'cars' / car / 'car.yaml').write_text('{}\n')
        folder = tmp_path / 'data' / car / 'racerbot_runs'
        folder.mkdir(parents=True)
        (folder / 'run.txt').write_text(car)
    (tmp_path / 'data/README.md').write_text('Runs\n')
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    pgrep = bindir / 'pgrep'
    pgrep.write_text('#!/bin/sh\nexit 1\n')
    pgrep.chmod(0o755)
    git(tmp_path, 'init', '-q')
    git(tmp_path, 'config', 'user.email', 'test@example.invalid')
    git(tmp_path, 'config', 'user.name', 'test')
    git(tmp_path, 'add', 'tools', 'src', 'cars', 'bin')
    git(tmp_path, 'commit', '-qm', 'start')
    return tmp_path


def save(repo, car='rb1'):
    env = dict(os.environ, PATH=f'{repo / "bin"}:{os.environ["PATH"]}',
               RACERBOT_CARS_DIR=str(repo / 'cars'))
    env.pop('RACERBOT_CAR', None)
    if car:
        env['RACERBOT_CAR'] = car
    return subprocess.run(['bash', 'tools/save_runs.sh', '--no-push'], cwd=repo,
                          env=env, capture_output=True, text=True)


def test_stages_only_this_car(repo):
    result = save(repo)
    assert result.returncode == 0, result.stderr
    committed = git(repo, 'diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD').splitlines()
    assert committed == ['data/README.md', 'data/rb1/racerbot_runs/run.txt']
    assert git(repo, 'ls-files', '--others', '--exclude-standard', 'data/rb2') == \
        'data/rb2/racerbot_runs/run.txt'


def test_refuses_without_car(repo):
    before = git(repo, 'rev-parse', 'HEAD')
    result = save(repo, car=None)
    assert result.returncode != 0
    assert 'RACERBOT_CAR' in result.stderr
    assert git(repo, 'rev-parse', 'HEAD') == before
    assert not git(repo, 'diff', '--cached', '--name-only')


def test_refuses_file_over_100mb(repo):
    subprocess.run(['truncate', '-s', '101M', str(repo / 'data/rb1/large.mcap')], check=True)
    before = git(repo, 'rev-parse', 'HEAD')
    result = save(repo)
    assert result.returncode != 0
    assert '100 MB' in result.stderr
    assert git(repo, 'rev-parse', 'HEAD') == before
    assert not git(repo, 'diff', '--cached', '--name-only')
