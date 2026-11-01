import importlib.util
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racerbot_car.layers import (MissingCalibrationError, check_block_keys_match,
                                  effective_values, layered, overlay_files)


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return str(path.resolve())


def block(**params):
    return {'ros__parameters': params}


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
    with pytest.raises(MissingCalibrationError, match='servo_min'):
        layered('shared', 'vesc.yaml', 'rb1', tmp_path)


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
    paths = [write(tmp_path / str(i), {'n': block(x=i)}) for i in range(3)]
    assert effective_values(paths, 'n') == {'x': 2}


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
