"""Data commit limits, exercised against disposable repositories.

Step 3 mutation checks: the rejection, staged-blob, boundary, push refusal and
hook setup tests respectively caught a 200 MiB limit, working-tree sizes, >=,
a disabled push check and a missing hooksPath. The other tests caught a 40 MiB
hook limit, incorrect car/dev-box output, rejecting all staged data and an
LFS mention. All passed after reverting the mutations.
"""
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]
MIB = 1024 * 1024


def git(repo, *args):
    return subprocess.check_output(['git', *args], cwd=repo, text=True).strip()


@pytest.fixture
def repo(tmp_path):
    (tmp_path / 'tools/dev').mkdir(parents=True)
    for name in ('tools/save_runs.sh', 'tools/dev/setup_clone.sh',
                 'tools/dev/hooks/pre-commit'):
        source = ROOT / name
        if source.exists():
            target = tmp_path / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    shutil.copytree(ROOT / 'src/racerbot_car', tmp_path / 'src/racerbot_car',
                    ignore=shutil.ignore_patterns('__pycache__'))
    (tmp_path / 'cars/rb1').mkdir(parents=True)
    (tmp_path / 'cars/rb1/car.yaml').write_text('{}\n')
    (tmp_path / 'cars/machines.yaml').write_text('{}\n')
    (tmp_path / 'data/rb1').mkdir(parents=True)
    (tmp_path / 'bin').mkdir()
    pgrep = tmp_path / 'bin/pgrep'
    pgrep.write_text('#!/bin/sh\nexit 1\n')
    pgrep.chmod(0o755)
    git(tmp_path, 'init', '-q')
    git(tmp_path, 'config', 'user.name', 'test')
    git(tmp_path, 'config', 'user.email', 'test@example.invalid')
    git(tmp_path, 'commit', '--allow-empty', '-qm', 'start')
    return tmp_path


def env(repo, car='rb1', **extra):
    result = dict(os.environ, PATH=f'{repo / "bin"}:{os.environ["PATH"]}',
                  RACERBOT_CARS_DIR=str(repo / 'cars'), PYTHONPATH='')
    result.pop('RACERBOT_CAR', None)
    result.pop('SAVE_RUNS_PUSH_LIMIT_BYTES', None)
    if car:
        result['RACERBOT_CAR'] = car
    result.update(extra)
    return result


def sparse(path, size):
    subprocess.run(['truncate', '-s', str(size), str(path)], check=True)


def hook(repo):
    return subprocess.run(['bash', 'tools/dev/hooks/pre-commit'], cwd=repo,
                          capture_output=True, text=True)


def run_setup(repo, car='rb1'):
    return subprocess.run(['bash', 'tools/dev/setup_clone.sh'], cwd=repo,
                          env=env(repo, car), capture_output=True, text=True)


def save(repo, limit):
    return subprocess.run(['bash', 'tools/save_runs.sh', '--no-push'], cwd=repo,
                          env=env(repo, SAVE_RUNS_PUSH_LIMIT_BYTES=str(limit)),
                          capture_output=True, text=True)


def test_pre_commit_rejects_96mb_file(repo):
    sparse(repo / 'data/rb1/large bag', 96 * MIB)
    git(repo, 'add', 'data')
    result = hook(repo)
    assert result.returncode == 1
    assert 'data/rb1/large bag' in result.stderr
    assert '96' in result.stderr and 'MB' in result.stderr


def test_pre_commit_allows_50mb_file(repo):
    sparse(repo / 'data/rb1/bag', 50 * MIB)
    git(repo, 'add', 'data')
    assert hook(repo).returncode == 0


def test_pre_commit_judges_the_staged_blob_not_the_working_copy(repo):
    path = repo / 'data/rb1/bag'
    sparse(path, 50 * MIB)
    git(repo, 'add', 'data')
    sparse(path, 96 * MIB)
    assert hook(repo).returncode == 0


def test_pre_commit_boundary(repo):
    path = repo / 'data/rb1/bag'
    sparse(path, 95 * MIB)
    git(repo, 'add', 'data')
    assert hook(repo).returncode == 0
    sparse(path, 95 * MIB + 1)
    git(repo, 'add', 'data')
    assert hook(repo).returncode == 1


def test_setup_clone_installs_the_hook(repo):
    assert run_setup(repo).returncode == 0
    assert git(repo, 'config', 'core.hooksPath') == 'tools/dev/hooks'
    sparse(repo / 'data/rb1/large', 96 * MIB)
    git(repo, 'add', 'data')
    result = subprocess.run(['git', 'commit', '-qm', 'large'], cwd=repo,
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert 'data/rb1/large' in result.stderr


def test_setup_clone_names_the_car(repo):
    result = run_setup(repo)
    assert result.returncode == 0
    assert 'rb1' in result.stdout


def test_setup_clone_on_a_dev_box_says_so(repo):
    result = run_setup(repo, car=None)
    assert result.returncode == 0
    assert 'dev box: set RACERBOT_CAR for the simulator' in result.stdout


def test_save_runs_refuses_over_push_limit(repo):
    (repo / 'data/rb1/one').write_bytes(b'a' * 100)
    (repo / 'data/rb1/two').write_bytes(b'b' * 101)
    before = git(repo, 'rev-parse', 'HEAD')
    result = save(repo, 200)
    assert result.returncode == 1
    assert 'push too large:' in result.stderr
    assert 'GitHub refuses pushes over 2 GB' in result.stderr
    assert git(repo, 'rev-parse', 'HEAD') == before
    assert not git(repo, 'diff', '--cached', '--name-only')


def test_save_runs_allows_just_under_push_limit(repo):
    (repo / 'data/rb1/one').write_bytes(b'a' * 100)
    (repo / 'data/rb1/two').write_bytes(b'b' * 99)
    result = save(repo, 200)
    assert result.returncode == 0, result.stderr
    assert git(repo, 'show', 'HEAD:data/rb1/two') == 'b' * 99


def test_save_runs_never_mentions_lfs():
    assert 'lfs' not in (ROOT / 'tools/save_runs.sh').read_text().lower()
