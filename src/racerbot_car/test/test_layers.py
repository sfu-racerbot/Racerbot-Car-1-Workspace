import importlib.util
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racerbot_car.layers import (MissingCalibrationError, check_block_keys_match,
                                  effective_config, effective_values, layered, overlay_files,
                                  repo_root, shared_config_path)


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return str(path.resolve())


def block(**params):
    return {'ros__parameters': params}


def test_car_overlays_lists_calibration_and_tuning(tmp_path, monkeypatch):
    from racerbot_car.layers import car_overlays
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(tmp_path))
    cal = Path(write(tmp_path / 'rb1/calibration/z.yaml', {}))
    tuning = Path(write(tmp_path / 'rb1/tuning/a.yaml', {}))
    write(tmp_path / 'rb2/calibration/other.yaml', {})
    write(tmp_path / 'rb1/imu/other.yaml', {})
    assert car_overlays('rb1') == sorted([cal, tuning])


def test_repo_root_source_tree(monkeypatch):
    monkeypatch.delenv('RACERBOT_WS', raising=False)
    root = Path(__file__).resolve().parents[3]
    assert repo_root() == root


def test_repo_root_env_override(tmp_path, monkeypatch):
    (tmp_path / 'src/racerbot_car').mkdir(parents=True)
    monkeypatch.setenv('RACERBOT_WS', str(tmp_path))
    assert repo_root() == tmp_path


def test_shared_config_path_known_and_unknown():
    assert shared_config_path('vesc.yaml') == repo_root() / 'src/f1tenth_system/f1tenth_stack/config/vesc.yaml'
    with pytest.raises(KeyError, match='vesc.yaml'):
        shared_config_path('unknown.yaml')


def test_effective_config_rb1_vesc_to_odom_gain():
    # Five tape runs, 2026-10-21: docs/hardware-reference.md.
    assert effective_config('vesc.yaml', 'vesc_to_odom_node', 'rb1')['speed_to_erpm_gain'] == -4040.0


def test_effective_config_uses_overlay_over_shared(tmp_path, monkeypatch):
    root = tmp_path / 'checkout'
    (root / 'src/racerbot_car').mkdir(parents=True)
    write(root / 'src/gap_follow/config/gap_follow.yaml',
          {'gap_follow_node': block(max_steering_angle=0.1, arc_guard_max_correction=0.2)})
    write(tmp_path / 'cars/rb1/calibration/gap_follow.yaml',
          {'gap_follow_node': block(max_steering_angle=0.3, arc_guard_max_correction=0.2)})
    monkeypatch.setenv('RACERBOT_WS', str(root))
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(tmp_path / 'cars'))
    assert effective_config('gap_follow.yaml', 'gap_follow_node', 'rb1')['max_steering_angle'] == 0.3


def test_layered_order_shared_calibration_tuning(tmp_path):
    shared = write(tmp_path / 'shared', {})
    cal = write(tmp_path / 'rb1/calibration/mux.yaml', {})
    tuning = write(tmp_path / 'rb1/tuning/mux.yaml', {})
    assert layered(shared, 'mux.yaml', 'rb1', tmp_path) == [shared, cal, tuning]


def test_layered_only_existing_overlays(tmp_path):
    tuning = write(tmp_path / 'rb1/tuning/mux.yaml', {})
    assert overlay_files('mux.yaml', 'rb1', tmp_path) == [tuning]


def test_layered_raises_when_required_key_missing(tmp_path):
    write(tmp_path / 'rb1/calibration/vesc.yaml', {'/**': block(servo_max=1)})
    with pytest.raises(MissingCalibrationError, match='servo_min') as exc:
        layered('shared', 'vesc.yaml', 'rb1', tmp_path)
    assert 'rb1' in str(exc.value)
    assert 'vesc_to_odom_node.speed_to_erpm_gain' in str(exc.value)
    assert str(tmp_path / 'rb1/calibration/vesc.yaml') in str(exc.value)


def test_layered_raises_when_calibration_overlay_absent(tmp_path):
    with pytest.raises(MissingCalibrationError, match=r'calibration/vesc\.yaml'):
        layered('shared', 'vesc.yaml', 'rb1', tmp_path)


def test_layered_unsplit_config_returns_shared_only(tmp_path):
    assert layered('shared', 'mux.yaml', 'rb1', tmp_path) == ['shared']


def test_layered_uses_current_car(tmp_path, monkeypatch):
    from racerbot_car import identity
    write(tmp_path / 'rb1/tuning/mux.yaml', {})
    rb2 = write(tmp_path / 'rb2/tuning/mux.yaml', {})
    monkeypatch.setattr(identity, 'current_car_id', lambda: 'rb2')
    assert layered('shared', 'mux.yaml', cars_dir=tmp_path) == ['shared', rb2]


def test_effective_values_overlay_wins(tmp_path):
    a = write(tmp_path / 'a', {'/**': block(servo_min=0.05)})
    b = write(tmp_path / 'b', {'/**': block(servo_min=0.15)})
    assert effective_values([a, b], 'n')['servo_min'] == 0.15


def test_effective_values_tuning_beats_calibration(tmp_path):
    paths = [write(tmp_path / 'shared', {'n': block(x=0, retained=7)}),
             write(tmp_path / 'calibration', {'n': block(x=1)}),
             write(tmp_path / 'tuning', {'n': block(x=2)})]
    assert effective_values(paths, 'n') == {'x': 2, 'retained': 7}


# Recorded ROS 2 Jazzy measurement, 2026-10-31.
@pytest.mark.parametrize('a,b,expected', [
    ({'/**': block(x=1, y=1), 'n': block(x=2)}, {'/**': block(x=3, y=3)}, {'x': 2, 'y': 3}),
    ({'n': block(x=2)}, {'/**': block(x=3)}, {'x': 3}),
    ({'/n': block(x=5), '/**': block(x=1)}, {'n': block(x=2)}, {'x': 2}),
])
def test_effective_values_rcl_cases(tmp_path, a, b, expected):
    assert effective_values([write(tmp_path / 'a', a), write(tmp_path / 'b', b)], '/n') == expected


def test_check_block_keys_match_flags_slash_and_wildcard_mismatch(tmp_path):
    shared = write(tmp_path / 'a', {'n': block(x=1, y=2)})
    overlay = write(tmp_path / 'b', {'/n': block(x=2), '/**': block(y=3)})
    problems = check_block_keys_match(shared, overlay)
    assert len(problems) == 2 and any('/n' in p for p in problems) and any('/**' in p for p in problems)


def test_check_block_keys_match_clean(tmp_path):
    a = write(tmp_path / 'a', {'n': block(x=1)})
    b = write(tmp_path / 'b', {'n': block(x=2)})
    assert check_block_keys_match(a, b) == []


def test_effective_values_wildcard_and_named_blocks(tmp_path):
    p = write(tmp_path / 'a', {'/**': block(x=1), '/**/n': block(y=2), '/ns/n': block(z=3)})
    assert effective_values([p], '/ns/n') == {'x': 1, 'y': 2, 'z': 3}
    assert effective_values([p], None) == {'x': 1}


def test_effective_values_nested_dotted(tmp_path):
    p = write(tmp_path / 'a', {'n': block(a={'b': {'c': [1, 2]}})})
    assert effective_values([p], 'n') == {'a.b.c': [1, 2]}


def test_effective_values_other_node_block_ignored(tmp_path):
    p = write(tmp_path / 'a', {'other': block(x=1), 'n': block(y=2)})
    assert effective_values([p], 'n') == {'y': 2}


def test_effective_values_basename_matches_fqn_block(tmp_path):
    path = write(tmp_path / 'params.yaml', {'/n': block(x=5)})
    assert effective_values([path], 'n')['x'] == 5


def test_effective_values_basename_and_fqn_agree(tmp_path):
    shared = write(tmp_path / 'shared.yaml', {
        '/**': block(x=1), '/n': block(x=5), 'n': block(y=2),
        '/**/n': block(z=3)})
    overlay = write(tmp_path / 'overlay.yaml', {'/**': block(x=9)})
    expected = {'x': 5, 'y': 2, 'z': 3}
    assert effective_values([shared, overlay], 'n') == expected
    assert effective_values([shared, overlay], '/n') == expected


@pytest.mark.parametrize('fqn', ['/demo_node', '/ns2/named_node'])
def test_effective_values_agrees_with_launch_params(fqn):
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location('launch_params', root / 'tools/dev/launch_params.py')
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ModuleNotFoundError as exc:
        if exc.name in ('ament_index_python', 'launch', 'launch_ros'):
            pytest.skip('needs a sourced ROS 2 Jazzy environment: run under rb')
        raise
    fixtures = root / 'tools/dev/tests/fixtures/launch_params'
    paths = [str(fixtures / name) for name in ('demo.yaml', 'overlay.yaml')]
    expected = module.flatten_params([(p, True) for p in paths], fqn)
    assert expected and effective_values(paths, fqn) == expected
