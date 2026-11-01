"""Keep Python calibration readers on the selected car's effective layers.

This is a conservative source guard, not YAML data-flow analysis: a file must
mention a shared config, perform file/YAML I/O, and index a calibration key.
Effective readers are permitted, but a separate raw-reading function in the
same module is still checked, so importing the helper cannot hide a regression.
"""

import ast
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racerbot_car.guards import CALIBRATION_KEYS  # noqa: E402
from racerbot_car.layers import SHARED_CONFIGS  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
EXCLUDED = (
    'src/racerbot_a', 'src/racerbot_b', 'src/web_dashboards',
    'src/realsense-ros', 'src/particle_filter', 'src/racerbot_car',
    # The guard finds the bridge/track tests' raw controller-limit checks.
    # Answer 4 and the piece-a ruling explicitly defer this simulator tree.
    'src/racerbot_sim',
)
WRITERS = {
    'src/odom_calibration/odom_calibration/vesc_yaml.py',
    'src/odom_calibration/odom_calibration/teleop_yaml.py',
}
ALLOW_LIST = {
    'src/odom_calibration/test/test_teleop_yaml.py':
        'Raw editor input: verify parsing, comments, backup and restore, not live calibration.',
    'src/odom_calibration/test/test_vesc_yaml.py':
        'Raw/frozen editor input: verify exact document edits and restoration (Task 6).',
    'tools/odom_calibration_e2e/run_e2e.py':
        'Writer integration harness: inspect mutable temporary apply/restart/undo documents.',
}


def _call_name(node):
    if isinstance(node.func, ast.Name):
        return node.func.id
    if isinstance(node.func, ast.Attribute):
        return node.func.attr
    return ''


def _has_effective_read(tree):
    return any(isinstance(node, ast.Call)
               and _call_name(node) in {'effective_config', 'effective_values'}
               for node in ast.walk(tree))


def _raw_read_and_index(tree, keys):
    reads = any(isinstance(node, ast.Call)
                and (_call_name(node) in {'open', 'read_text', 'safe_load', 'safe_load_all'}
                     or (isinstance(node.func, ast.Attribute)
                         and isinstance(node.func.value, ast.Name)
                         and node.func.value.id == 'yaml' and node.func.attr == 'load'))
                for node in ast.walk(tree))
    indexes = any(isinstance(node, ast.Subscript)
                  and isinstance(node.slice, ast.Constant)
                  and node.slice.value in keys
                  for node in ast.walk(tree))
    return reads and indexes


def test_no_direct_calibration_reads():
    findings = []
    checked = 0
    for directory in ('src', 'tools'):
        for path in sorted((ROOT / directory).rglob('*.py')):
            relative = path.relative_to(ROOT)
            name = relative.as_posix()
            if (any(name == prefix or name.startswith(prefix + '/') for prefix in EXCLUDED)
                    or 'launch' in relative.parts or name.endswith('_launch.py')
                    or name in WRITERS or name in ALLOW_LIST):
                continue
            checked += 1
            source = path.read_text(encoding='utf-8')
            configs = [config for config in SHARED_CONFIGS if config in source]
            if not configs:
                continue
            keys = {key for config in configs for block in CALIBRATION_KEYS[config].values()
                    for key in block}
            tree = ast.parse(source, filename=name)
            if not _raw_read_and_index(tree, keys):
                continue
            if not _has_effective_read(tree):
                findings.append(name)
                continue
            # A migrated module can contain an independent legacy raw reader.
            for scope in ast.walk(tree):
                if (isinstance(scope, (ast.FunctionDef, ast.AsyncFunctionDef))
                        and _raw_read_and_index(scope, keys) and not _has_effective_read(scope)):
                    findings.append(f'{name}:{scope.lineno} ({scope.name})')
    assert checked > 0, 'the source guard scanned no Python files'
    assert not findings, 'Direct calibration reads; use effective_config/effective_values:\n' + '\n'.join(findings)
