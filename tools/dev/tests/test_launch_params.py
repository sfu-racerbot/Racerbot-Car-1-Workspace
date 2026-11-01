"""Run with: RB_WS=$PWD rb python3 -m pytest tools/dev/tests/test_launch_params.py."""
import copy
import gc
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


def test_normalize_replaces_run_timestamps():
    # Oracle: YYYYMMDD-HHMMSS run names are variable; longer numbers are not.
    assert normalize('/x/racerbot_runs/20261031-203927/bag', '/workspace') == (
        '/x/racerbot_runs/<TIMESTAMP>/bag')
    assert normalize('120261031-203927', '/workspace') == '120261031-203927'


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
    relative = (tmp_path / 'spawned').relative_to(tempfile.gettempdir())
    assert demo['processes'][0]['cmd'] == ['touch', f'<TMP>/{relative}']
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


def test_inline_parameter_captures_back_to_back(tmp_path):
    launch_file = tmp_path / 'inline_launch.py'
    launch_file.write_text(
        'import gc\n'
        'from launch import LaunchDescription\n'
        'from launch_ros.actions import Node\n'
        'def generate_launch_description():\n'
        '    gc.collect()\n'
        '    node = Node(\n'
        '        package="demo", executable="not_started", name="inline",\n'
        '        parameters=[{"gain": 3}])\n'
        '    node.cycle = node\n'
        '    return LaunchDescription([node])\n')
    # Exercise collection during launch loading as well as inline parameters.
    # Removing capture's pre-collection did not reproduce the original cleanup
    # failure here; the production comment records why that safeguard stays.
    enabled = gc.isenabled()
    gc.disable()
    try:
        first = capture(str(launch_file), [])
        second = capture(str(launch_file), [])
        assert first['nodes'][0]['params']['gain'] == 3
        assert second['nodes'][0]['params']['gain'] == 3
        assert second['errors'] == []
    finally:
        if enabled:
            gc.enable()
        gc.collect()


def test_rule_param_parsed():
    assert flatten_params([('x:=0.5', False)], '/n') == {'x': 0.5}


def rcl_entries(case):
    return [(str(FIXTURES / 'rcl' / f'case_{case}_{part}.yaml'), True)
            for part in ('a', 'b')]


def test_rcl_semantics_case_1():
    # oracle: controller's real rclpy/Jazzy measurement, 2026-10-31.
    assert flatten_params(rcl_entries(1), '/n') == {'x': 2, 'y': 3}


def test_rcl_semantics_case_2():
    # oracle: controller's real rclpy/Jazzy measurement, 2026-10-31.
    assert flatten_params(rcl_entries(2), '/n') == {'x': 3}


def test_rcl_semantics_case_3():
    # oracle: controller's real rclpy/Jazzy measurement, 2026-10-31.
    assert flatten_params(rcl_entries(3), '/n') == {'x': 2}


def test_rcl_rule_overrides_follow_all_files():
    # oracle: round-5 model puts all -p rules last, in their own order.
    first, second = rcl_entries(1)
    entries = [('x:=9', False), first, ('x:=4', False), second]
    assert flatten_params(entries, '/n') == {'x': 4, 'y': 3}


def test_rcl_unnamed_blocks_preserve_entry_order():
    # oracle: round-5 model preserves exact keys and their first-appearance order.
    result = capture(str(FIXTURES / 'rcl/unnamed_launch.py'), [])
    captured = json.loads(json.dumps(result, sort_keys=True))
    assert captured['nodes'][0]['name'] is None
    assert captured['nodes'][0]['params'] == {'x': 1}
    assert captured['nodes'][0]['blocks'] == [
        {'key': '/n', 'params': {'x': 5}},
        {'key': '/**', 'params': {'x': 1}},
        {'key': 'n', 'params': {'x': 2}},
    ]
    assert captured['nodes'][1]['params'] == {'x': 3, 'y': 3}
    assert captured['nodes'][1]['blocks'] == [
        {'key': '/**', 'params': {'x': 3, 'y': 3}},
        {'key': 'n', 'params': {'x': 2}},
    ]


def test_diff_reports_unnamed_block_order_change():
    # oracle: entry order determines overrides, so a reorder must be visible.
    a = {'nodes': [record(None, {})], 'processes': []}
    a['nodes'][0]['executable'] = 'exe'
    a['nodes'][0]['blocks'] = [
        {'key': '/**', 'params': {'x': 1}}, {'key': 'n', 'params': {'x': 2}}]
    b = copy.deepcopy(a)
    b['nodes'][0]['blocks'].reverse()
    assert diff_captures(a, b) == [
        '~ p/exe blocks [{"key": "/**", "params": {"x": 1}}, {"key": "n", "params": {"x": 2}}]'
        ' -> [{"key": "n", "params": {"x": 2}}, {"key": "/**", "params": {"x": 1}}]']


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


def test_missing_include_package_not_found_error_type():
    # oracle: a typed missing-package error need not contain the message pattern.
    result = capture(str(FIXTURES / 'missing_include_type_launch.py'), [])
    assert result['missing_includes'] == [{'package': 'no_such_pkg_abc', 'deferred': False}]
    assert node(result, '/after_include')['executable'] == 'exe'


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


def test_os_fork_refused():
    # oracle: OS process creation is refused before the fixture can fork.
    original = os.fork
    with pytest.raises(CaptureError, match='^launch-time process creation not allowed by the oracle: fork$'):
        capture(str(FIXTURES / 'fork_launch.py'), [])
    assert os.fork is original


def test_os_spawn_refused():
    # oracle: even a harmless true process must not be created by capture.
    original = os.spawnlp
    with pytest.raises(CaptureError, match='^launch-time process creation not allowed by the oracle: spawnlp$'):
        capture(str(FIXTURES / 'spawn_launch.py'), [])
    assert os.spawnlp is original


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


def test_truncate_outside_sandbox_refused(tmp_path):
    # oracle: a refused truncate must preserve the outside file's contents.
    target = tmp_path / 'outside.txt'
    target.write_text('keep this', encoding='utf-8')
    with pytest.raises(CaptureError, match=f'launch-time write outside the oracle sandbox: {re.escape(str(target))}'):
        capture(str(FIXTURES / 'truncate_launch.py'), [f'target:={target}'])
    assert target.read_text(encoding='utf-8') == 'keep this'


@pytest.mark.parametrize('api', ['builtins', 'io', 'os'])
@pytest.mark.parametrize('symlink', [False, True])
def test_device_open_refused(tmp_path, api, symlink):
    # oracle: all three open APIs must refuse the device, even via a symlink.
    alias = tmp_path / 'device'
    alias.symlink_to('/dev/tty')
    target = str(alias) if symlink else '/dev/tty'
    with pytest.raises(CaptureError, match='^launch-time device open refused by the oracle: /dev/tty$'):
        capture(str(FIXTURES / 'device_launch.py'), [f'target:={target}', f'api:={api}'])


@pytest.mark.parametrize('api', ['builtins', 'io', 'os'])
def test_dev_null_allowed(api):
    # oracle: the explicitly allowed null device supplies an empty read.
    result = capture(str(FIXTURES / 'device_launch.py'), ['target:=/dev/null', f'api:={api}'])
    assert node(result, '/device_read')['params'] == {'bytes_read': 0}


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
        'x': ['<WS>/a', '<HOME>/b', '<TMP>/one/two']}


def test_normalize_keeps_temp_relative_path():
    # oracle: distinct temp subdirectories must remain distinguishable.
    tmp = tempfile.gettempdir()
    assert normalize([f'{tmp}/old/map.yaml', f'{tmp}/new/map.yaml'], '/workspace') == [
        '<TMP>/old/map.yaml', '<TMP>/new/map.yaml']


def test_normalize_matrix_dir_beats_ws():
    # oracle: longest path prefix is replaced first.
    assert normalize(['/a/tools/dev/x', '/a/src/y'], '/a', matrix_dir='/a/tools/dev') == [
        '<MATRIX>/x', '<WS>/src/y']


def test_capture_one_normalizes_matrix_dir(tmp_path):
    # oracle: CLI passes its matrix directory into the capture normalizer.
    matrix_dir = tmp_path / 'matrix'
    matrix_dir.mkdir()
    command = [sys.executable, str(ROOT / 'tools/dev/launch_params.py'), 'capture-one',
               '--file', str(FIXTURES / 'demo_launch.py'), '--matrix-dir', str(matrix_dir),
               '--', f'marker:={matrix_dir}/marker']
    process = subprocess.run(command, capture_output=True, text=True, check=True)
    assert json.loads(process.stdout)['processes'][0]['cmd'] == ['touch', '<MATRIX>/marker']


def record(name, params, files=None):
    return {'namespace': '/', 'name': name, 'package': 'p', 'executable': name,
            'params': params, 'param_files': files or [], 'remaps': [], 'deferred': False}


def test_diff_reports_added_removed_changed():
    a = {'nodes': [record('/n', {'servo': 0.05, 'old': 3})], 'processes': []}
    b = {'nodes': [record('/n', {'servo': 0.15}), record('/new', {})], 'processes': []}
    assert diff_captures(a, b) == ['+ node /new', '- /n old=3', '~ /n servo 0.05 -> 0.15']


# oracle: scalar type changes are behavioral differences, including nested values.
def test_diff_reports_bool_to_int():
    a = {'nodes': [record('/n', {'x': True})], 'processes': []}
    b = {'nodes': [record('/n', {'x': 1})], 'processes': []}
    assert diff_captures(a, b) == ['~ /n x true -> 1']


def test_diff_reports_int_to_float():
    a = {'nodes': [record('/n', {'x': 1})], 'processes': []}
    b = {'nodes': [record('/n', {'x': 1.0})], 'processes': []}
    assert diff_captures(a, b) == ['~ /n x 1 -> 1.0']


@pytest.mark.parametrize('old, new, expected', [
    (1, None, '~ /n x 1 -> null'),
    (None, 1, '~ /n x null -> 1'),
    (1, '1', '~ /n x 1 -> "1"'),
])
def test_diff_reports_value_to_null(old, new, expected):
    a = {'nodes': [record('/n', {'x': old})], 'processes': []}
    b = {'nodes': [record('/n', {'x': new})], 'processes': []}
    assert diff_captures(a, b) == [expected]


def test_diff_reports_type_change_inside_list():
    a = {'nodes': [record('/n', {'x': [{'nested': [True]}]})], 'processes': []}
    b = {'nodes': [record('/n', {'x': [{'nested': [1]}]})], 'processes': []}
    assert diff_captures(a, b) == ['~ /n x [{"nested": [true]}] -> [{"nested": [1]}]']


def test_diff_reports_type_change_inside_unnamed_blocks():
    a = {'nodes': [record(None, {})], 'processes': []}
    a['nodes'][0]['executable'] = 'exe'
    a['nodes'][0]['blocks'] = {'/**': {'x': [True]}}
    b = copy.deepcopy(a)
    b['nodes'][0]['blocks']['/**']['x'] = [1]
    assert diff_captures(a, b) == ['~ p/exe blocks {"/**": {"x": [true]}} -> {"/**": {"x": [1]}}']


def test_diff_reports_duplicate_named_node():
    # oracle: each repeated identity represents another node, not an overwrite.
    a = {'nodes': [record('/x', {})], 'processes': []}
    b = {'nodes': [record('/x', {}), record('/x', {})], 'processes': []}
    assert diff_captures(a, b) == ['+ node /x#2']
    assert diff_captures(b, a) == ['- node /x#2']
    b['nodes'][1]['params'] = {'gain': 2}
    c = copy.deepcopy(b)
    c['nodes'][1]['params']['gain'] = 3
    assert diff_captures(b, c) == ['~ /x#2 gain 2 -> 3']


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
    assert resolve_args(entry, 'rb1', str(tmp_path)) == [f'run:={tmp_path}/data/rb1/racerbot_auto/x']
    (tmp_path / 'data/rb1/racerbot_auto').mkdir(parents=True)
    assert resolve_args(entry, 'rb1', str(tmp_path)) == [f'run:={tmp_path}/data/rb1/racerbot_auto/x']


def test_resolve_args_matrix_dir_placeholder(tmp_path):
    # oracle: the placeholder resolves against the matrix directory, not ws.
    matrix_dir = tmp_path / 'matrix'
    entry = {'args': ['run:={matrix_dir}/tests/fixtures/run']}
    assert resolve_args(entry, 'rb1', '/other/workspace', matrix_dir=str(matrix_dir)) == [
        f'run:={matrix_dir}/tests/fixtures/run']


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
                  for arg in resolve_args(entry, car, str(ROOT), matrix_dir=str(ROOT / 'tools/dev'))
                  if fixture_marker in arg]
    assert references
    for arg in references:
        path = Path(arg.split(':=', 1)[1])
        directory = path.parent if path.name == 'map.yaml' else path
        assert directory.is_dir()
        assert (directory / 'map.yaml').is_file()
        assert (directory / 'raceline_profiled.csv').is_file()
