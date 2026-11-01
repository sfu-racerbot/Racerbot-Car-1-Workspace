"""Task 7: replay defaults are per-car, and explicit paths need no identity."""
from pathlib import Path
import sys

import pytest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import grip_replay
import slam_replay


@pytest.mark.parametrize('car', ['rb1', 'rb2'])
def test_grip_lookup_uses_selected_car(tmp_path, monkeypatch, car):
    monkeypatch.setenv('RACERBOT_WS', str(tmp_path))
    run = tmp_path / 'data' / car / 'racerbot_runs' / 'example'
    run.mkdir(parents=True)
    assert grip_replay.resolve_run('example', car) == run


def test_grip_explicit_path_needs_no_car(tmp_path, monkeypatch):
    monkeypatch.delenv('RACERBOT_CAR', raising=False)
    assert grip_replay.resolve_run(str(tmp_path)) == tmp_path


def test_grip_cli_refuses_without_car(monkeypatch, capsys):
    monkeypatch.delenv('RACERBOT_CAR', raising=False)
    with pytest.raises(SystemExit) as exc:
        grip_replay.main(['no-such-run'])
    assert exc.value.code == 2
    assert 'RACERBOT_CAR' in capsys.readouterr().err


@pytest.mark.parametrize('car', ['rb1', 'rb2'])
@pytest.mark.parametrize('from_env', [False, True])
def test_grip_cli_selects_car(tmp_path, monkeypatch, car, from_env):
    # Task 7: an explicit --car beats the environment, and reports stay in
    # that car's run. Use a tiny synthetic cache, so no ROS or recorded bag is needed.
    monkeypatch.setenv('RACERBOT_WS', str(tmp_path))
    monkeypatch.setenv('RACERBOT_CAR', car if from_env else
                       ('rb2' if car == 'rb1' else 'rb1'))
    analysis = tmp_path / 'data' / car / 'racerbot_runs/example/analysis'
    analysis.mkdir(parents=True)
    np.savez(analysis / grip_replay.CACHE,
             drive=np.array([[0.0, 0.0], [0.1, 0.1]]),
             gyro=np.array([[0.0, 0.0], [0.1, 0.0]]),
             odom=np.array([[0.0, 1.0], [0.1, 1.0]]))
    argv = ['example'] if from_env else ['example', '--car', car]
    assert grip_replay.main(argv) == 0
    assert (analysis / grip_replay.REPORT).is_file()
    other = 'rb2' if car == 'rb1' else 'rb1'
    assert not (tmp_path / 'data' / other).exists()


@pytest.mark.parametrize('car', ['rb1', 'rb2'])
@pytest.mark.parametrize('from_env', [False, True])
def test_slam_output_default_uses_selected_car(tmp_path, monkeypatch, car, from_env):
    monkeypatch.setenv('RACERBOT_WS', str(tmp_path))
    monkeypatch.setenv('RACERBOT_CAR', car if from_env else
                       ('rb2' if car == 'rb1' else 'rb1'))
    argv = ['source'] if from_env else ['source', '--car', car]
    args = slam_replay._parser().parse_args(argv)
    assert slam_replay.replay_output_root(args.out_root, args.car) == \
        tmp_path / 'data' / car / 'racerbot_runs'


def test_slam_explicit_output_needs_no_car(tmp_path, monkeypatch):
    monkeypatch.delenv('RACERBOT_CAR', raising=False)
    args = slam_replay._parser().parse_args(['source', '--out-root', str(tmp_path)])
    assert slam_replay.replay_output_root(args.out_root, args.car) == tmp_path


def test_slam_cli_refuses_without_car(tmp_path, monkeypatch, capsys):
    monkeypatch.delenv('RACERBOT_CAR', raising=False)
    proc = tmp_path / 'proc'
    proc.mkdir()
    monkeypatch.setattr(slam_replay, 'PROC_ROOT', proc)
    assert slam_replay.main(['source']) == 2
    assert 'RACERBOT_CAR' in capsys.readouterr().err
