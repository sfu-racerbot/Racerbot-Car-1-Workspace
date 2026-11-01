"""Run with: RB_WS=$PWD rb python3 -m pytest tools/dev/tests/test_launch_params.py."""
import copy
import os
from pathlib import Path
import re
import sys
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


def test_condition_false_node_absent(demo):
    assert {n['name'] for n in demo['nodes']} == {'/demo_node', '/ns2/named_node'}


def test_include_and_namespace_followed(demo):
    assert node(demo, '/ns2/named_node')['params']['gain'] == 2.0


def test_event_handler_actions_captured_as_deferred():
    result = capture(str(FIXTURES / 'handler_launch.py'), [])
    assert node(result, '/after_node')['deferred'] is True
    assert result['processes'] == [{'cmd': ['true'], 'deferred': False}]


def test_broken_launch_raises_capture_error():
    with pytest.raises(CaptureError, match='car id unknown: 999'):
        capture(str(FIXTURES / 'broken_launch.py'), [])


def test_pgrep_probe_simulated_no_match():
    result = capture(str(FIXTURES / 'probe_launch.py'), [])
    assert node(result, '/bridge_node')['executable'] == 'bridge_exe'
    assert result['probes'] == [{'cmd': ['pgrep', '-f', 'some_pattern'], 'simulated': 'no match'}]


def test_other_subprocess_refused():
    with pytest.raises(CaptureError, match='launch-time subprocess not allowed'):
        capture(str(FIXTURES / 'forbidden_launch.py'), [])


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
