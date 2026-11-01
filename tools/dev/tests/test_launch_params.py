"""Run with: RB_WS=$PWD rb python3 -m pytest tools/dev/tests/test_launch_params.py."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools/dev'))
from launch_params import CaptureError, capture, diff_captures, flatten_params, load_matrix, normalize, resolve_args

FIXTURES = Path(__file__).parent / 'fixtures/launch_params'


@pytest.fixture
def demo(tmp_path):
    return capture(str(FIXTURES / 'demo_launch.py'), [f'marker:={tmp_path}/spawned'])


def node(result, name):
    return next(item for item in result['nodes'] if item['name'] == name)


def stub_probes():
    if 'range_libc' not in sys.modules and importlib.util.find_spec('range_libc') is None:
        return [{'import': 'range_libc', 'simulated': 'stub'}]
    return []


# oracle: fixture YAML and launch descriptions specify the expected behavior.
def test_capture_never_spawns(demo, tmp_path):
    assert not (tmp_path / 'spawned').exists()
    assert demo['processes'][0]['cmd'] == ['touch', '<TMP>/spawned']
    assert node(demo, '/demo_node')['executable'] == 'does_not_exist_exe'


def test_flatten_later_entry_wins(demo):
    assert node(demo, '/demo_node')['params']['servo_min'] == 0.15
    assert node(demo, '/demo_node')['params']['servo_max'] == 0.75


def test_flatten_ignores_other_node_block():
    assert flatten_params([(str(FIXTURES / 'demo.yaml'), True)], '/demo_node')['servo_min'] == 0.05


def test_flatten_wildcard_block_applies(demo):
    assert node(demo, '/demo_node')['params']['use_sim_time'] is False


def test_flatten_nested_dict_dotted(demo):
    assert node(demo, '/demo_node')['params']['human_control.axis_mappings.drive-steering_angle.scale'] == 0.38


def test_inline_dict_param(demo):
    assert node(demo, '/demo_node')['params']['inline'] == 3


def test_rule_param_parsed():
    assert flatten_params([('x:=0.5', False)], '/n') == {'x': 0.5}


def test_jazzy_inline_tuple_as_list():
    # oracle: ROS arrays are JSON lists even when Jazzy emits a Python tuple tag.
    assert flatten_params([(str(FIXTURES / 'tuple.yaml'), True)], '/demo_node') == {
        'origin': [1.0, 2.0, 3.0]}


def test_condition_false_node_absent(demo):
    assert {n['name'] for n in demo['nodes']} == {'/demo_node', '/ns2/named_node'}


def test_include_and_namespace_followed(demo):
    assert node(demo, '/ns2/named_node')['params']['gain'] == 2.0


def test_missing_package_include_recorded():
    # oracle: missing include is reported and does not hide its sibling node.
    result = capture(str(FIXTURES / 'missing_include_launch.py'), [])
    assert node(result, '/after_include')['executable'] == 'exe'
    assert result['missing_includes'] == [{'package': 'no_such_pkg_xyz', 'deferred': False}]


def test_event_handler_actions_captured_as_deferred():
    result = capture(str(FIXTURES / 'handler_launch.py'), [])
    assert node(result, '/after_node')['deferred'] is True
    assert result['processes'] == [{'cmd': ['true'], 'deferred': False}]


def test_lifecycle_node_is_captured_without_ros_manager(monkeypatch):
    import launch_ros.utilities.lifecycle_event_manager as manager

    def forbidden(context):
        raise RuntimeError('lifecycle manager must not create a ROS node')

    monkeypatch.setattr(manager, 'get_ros_node', forbidden)
    result = capture(str(FIXTURES / 'lifecycle_launch.py'), [])
    assert node(result, '/lifecycle_node')['executable'] == 'lifecycle_exe'


def test_broken_launch_raises_capture_error():
    with pytest.raises(CaptureError, match='car id unknown: 999'):
        capture(str(FIXTURES / 'broken_launch.py'), [])


def test_pgrep_probe_simulated_no_match():
    expected = stub_probes()
    result = capture(str(FIXTURES / 'probe_launch.py'), [])
    assert node(result, '/bridge_node')['executable'] == 'bridge_exe'
    assert result['probes'] == expected + [{'cmd': ['pgrep', '-f', 'some_pattern'], 'simulated': 'no match'}]


def test_other_subprocess_refused():
    with pytest.raises(CaptureError, match='launch-time subprocess not allowed'):
        capture(str(FIXTURES / 'forbidden_launch.py'), [])


def test_shutdown_handler_recorded_never_run():
    result = capture(str(FIXTURES / 'shutdown_launch.py'), [])
    assert node(result, '/safe')['executable'] == 'exe'
    assert any(item.startswith('OnShutdown:') for item in result['deferred_callables'])


def test_write_inside_sandbox_recorded(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    result = capture(str(FIXTURES / 'writes_launch.py'), [])
    assert '<SANDBOX>/home/x/out.txt' in result['writes']
    assert not (tmp_path / 'x/out.txt').exists()


def test_write_outside_sandbox_refused(tmp_path):
    target = tmp_path / 'outside.txt'
    with pytest.raises(CaptureError, match='outside the oracle sandbox'):
        capture(str(FIXTURES / 'writes_outside_launch.py'), [f'target:={target}'])
    assert not target.exists()


def test_git_and_ps_probes_simulated():
    expected = stub_probes()
    result = capture(str(FIXTURES / 'git_ps_launch.py'), [])
    assert node(result, '/n128')['executable'] == 'exe'
    assert result['probes'] == expected + [
        {'cmd': ['git', '-C', '/', 'status'], 'simulated': 'not a repository'},
        {'cmd': ['ps', '-e'], 'simulated': 'empty'},
    ]


def test_dpkg_query_probe_simulated():
    # oracle: an empty successful simulated package listing has length zero.
    result = capture(str(FIXTURES / 'dpkg_probe_launch.py'), [])
    assert node(result, '/n0')['executable'] == 'exe'
    assert {'cmd': ['dpkg-query', '-W', 'ros-jazzy-*'], 'simulated': 'empty'} in result['probes']


def test_range_libc_stubbed_when_missing(monkeypatch):
    # oracle: missing CUDA module is simulated only for the duration of capture.
    monkeypatch.delitem(sys.modules, 'range_libc', raising=False)
    original = importlib.util.find_spec
    monkeypatch.setattr(importlib.util, 'find_spec',
                        lambda name, *args: None if name == 'range_libc' else original(name, *args))
    result = capture(str(FIXTURES / 'range_libc_launch.py'), [])
    assert node(result, '/range_node')['executable'] == 'exe'
    assert {'import': 'range_libc', 'simulated': 'stub'} in result['probes']
    assert 'range_libc' not in sys.modules


def test_capture_one_stdout_is_pure_json(tmp_path):
    command = [sys.executable, str(ROOT / 'tools/dev/launch_params.py'), 'capture-one',
               '--file', str(FIXTURES / 'demo_launch.py'), '--', f'marker:={tmp_path}/m']
    process = subprocess.run(command, capture_output=True, text=True, check=True)
    assert node(json.loads(process.stdout), '/demo_node')['executable'] == 'does_not_exist_exe'


def test_normalize_replaces_ws_home_tmp():
    assert normalize({'x': ['/workspace/a', os.path.expanduser('~/b'), tempfile.gettempdir() + '/one/two']}, '/workspace') == {
        'x': ['<WS>/a', '<HOME>/b', '<TMP>/two']}


def record(name, params, files=None):
    return {'namespace': '/', 'name': name, 'package': 'p', 'executable': name,
            'params': params, 'param_files': files or [], 'remaps': [], 'deferred': False}


def test_diff_reports_added_removed_changed():
    a = {'nodes': [record('/n', {'servo': 0.05, 'old': 3})], 'processes': []}
    b = {'nodes': [record('/n', {'servo': 0.15}), record('/new', {})], 'processes': []}
    assert diff_captures(a, b) == ['+ node /new', '- /n old=3', '~ /n servo 0.05 -> 0.15']


def test_diff_ignores_param_files():
    a = {'nodes': [record('/n', {'x': 1}, ['a'])], 'processes': []}
    b = copy.deepcopy(a)
    b['nodes'][0]['param_files'] = ['b']
    assert diff_captures(a, b) == []
    b['nodes'][0]['params']['x'] = 2
    assert diff_captures(a, b) == ['~ /n x 1 -> 2']


def test_diff_reports_missing_include():
    # oracle: added and removed missing includes are visible in the diff.
    a = {'nodes': [], 'processes': [], 'missing_includes': [{'package': 'old', 'deferred': False}]}
    b = {'nodes': [], 'processes': [], 'missing_includes': [{'package': 'new', 'deferred': True}]}
    assert diff_captures(a, b) == ['+ missing_include new', '- missing_include old']


def test_resolve_args_data_placeholder(tmp_path):
    entry = {'args': ['run:={data:racerbot_auto}/x']}
    assert resolve_args(entry, 'rb1', str(tmp_path)) == [f'run:={tmp_path}/data/racerbot_auto/x']
    (tmp_path / 'data/rb1/racerbot_auto').mkdir(parents=True)
    assert resolve_args(entry, 'rb1', str(tmp_path)) == [f'run:={tmp_path}/data/rb1/racerbot_auto/x']


def test_matrix_covers_every_launch_file():
    matrix = load_matrix(str(ROOT / 'tools/dev/launch_matrix.yaml'))
    excluded_roots = {'racerbot_a', 'racerbot_b', 'web_dashboards', 'realsense-ros',
                      'particle_filter', 'range_libc', 'transport_drivers'}
    files = {p for p in (ROOT / 'src').rglob('*launch*.py')
             if 'launch' in p.relative_to(ROOT / 'src').parts and
             p.relative_to(ROOT / 'src').parts[0] not in excluded_roots}
    files.add(ROOT / 'src/race_diagnostics/launch/record_run.py')
    pairs = set()
    for path in files:
        package = next(parent / 'package.xml' for parent in path.parents if (parent / 'package.xml').is_file())
        pairs.add((ET.parse(package).getroot().findtext('name'), path.name))
    entries = {(e['package'], e['launch']) for e in matrix['entries']}
    excluded = {ROOT / e['path'] for e in matrix['excluded'] if e['reason'].strip()}
    missing = {p for p in files if p not in excluded and
               (ET.parse(next(parent / 'package.xml' for parent in p.parents if (parent / 'package.xml').is_file())).getroot().findtext('name'), p.name) not in entries}
    assert not missing
    assert entries <= pairs


def test_matrix_ids_unique_and_safe():
    entries = load_matrix(str(ROOT / 'tools/dev/launch_matrix.yaml'))['entries']
    ids = [e['id'] for e in entries]
    assert ids
    assert len(ids) == len(set(ids))
    assert all(re.fullmatch(r'[a-z0-9][a-z0-9._-]*', item) for item in ids)


def test_matrix_run_dir_fixture_complete():
    # oracle: every matrix argument referencing the shared run has both inputs
    # required by race_launch.py and localize_run_launch.py.
    matrix = load_matrix(str(ROOT / 'tools/dev/launch_matrix.yaml'))
    fixture_marker = 'fixtures/launch_params/run_dir'
    references = [arg for entry in matrix['entries'] for car in ('rb1', 'rb2')
                  for arg in resolve_args(entry, car, str(ROOT)) if fixture_marker in arg]
    assert references
    for arg in references:
        path = Path(arg.split(':=', 1)[1])
        directory = path.parent if path.name == 'map.yaml' else path
        assert directory.is_dir()
        assert (directory / 'map.yaml').is_file()
        assert (directory / 'raceline_profiled.csv').is_file()
