"""Capture launch behavior without executing processes.

Run with: python3 tools/dev/launch_params.py --help (inside ROS 2 Jazzy).
"""
import argparse
import builtins
import importlib.util
from collections import Counter
from contextlib import ExitStack, redirect_stdout
import io
import json
import logging
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import types
from unittest.mock import patch

import yaml
from ament_index_python.packages import PackageNotFoundError
import launch.logging
from launch import LaunchService, LaunchDescriptionSource
from launch.actions import ExecuteLocal, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch.utilities import perform_substitutions
from launch_ros.actions import LifecycleNode, Node

STUB_MODULES = ('range_libc',)


class CaptureError(RuntimeError):
    """Launch evaluation failed or attempted an unsupported operation."""


class ParameterLoader(yaml.SafeLoader):
    """Read Jazzy's inline tuple parameters as ROS array values."""


ParameterLoader.add_constructor(
    'tag:yaml.org,2002:python/tuple',
    lambda loader, node: loader.construct_sequence(node, deep=True))


def _dotted(values, prefix=''):
    result = {}
    for key, value in values.items():
        name = f'{prefix}.{key}' if prefix else key
        if isinstance(value, dict):
            result.update(_dotted(value, name))
        else:
            result[name] = value
    return result


def _blocks(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.load(stream, Loader=ParameterLoader) or {}
    return {key: _dotted(value.get('ros__parameters', {}))
            for key, value in data.items() if isinstance(value, dict)}


def _merged_blocks(entries):
    # rcl keeps exact block keys in first-appearance order across all files.
    blocks = {}
    for value, is_file in entries:
        if is_file:
            for key, params in _blocks(value).items():
                blocks.setdefault(key, {}).update(params)
    return blocks


def flatten_params(entries, node_fqn):
    result = {}
    basename = node_fqn.rsplit('/', 1)[-1] if node_fqn else None
    keys = {'/**'}
    if basename:
        keys.update((node_fqn, basename, '/**/' + basename))
    for block, params in _merged_blocks(entries).items():
        if block in keys:
            result.update(params)
    # Command-line -p overrides apply after every file, in their own order.
    for value, is_file in entries:
        if not is_file:
            key, value = value.split(':=', 1)
            result[key] = yaml.safe_load(value)
    return result


def normalize(obj, ws_root, sandbox=None, *, matrix_dir=None):
    if isinstance(obj, dict):
        return {normalize(k, ws_root, sandbox, matrix_dir=matrix_dir):
                normalize(v, ws_root, sandbox, matrix_dir=matrix_dir) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [normalize(v, ws_root, sandbox, matrix_dir=matrix_dir) for v in obj]
    if not isinstance(obj, str):
        return obj
    replacements = [(os.path.realpath(ws_root), '<WS>'), (str(ws_root), '<WS>'),
                    (os.path.expanduser('~'), '<HOME>')]
    if sandbox:
        replacements.append((os.path.realpath(sandbox), '<SANDBOX>'))
    if matrix_dir:
        replacements.append((os.path.realpath(matrix_dir), '<MATRIX>'))
    for path, replacement in sorted(replacements, key=lambda pair: len(pair[0]), reverse=True):
        obj = obj.replace(path, replacement)
    obj = re.sub(r'(?<![0-9])[0-9]{8}-[0-9]{6}(?![0-9])', '<TIMESTAMP>', obj)
    tmp = tempfile.gettempdir().rstrip('/')
    return re.sub(r'(?<!>)' + re.escape(tmp) + r'(?=/|$)', '<TMP>', obj)


def capture(launch_file, launch_args, ws_root=None, *, sandbox=None, matrix_dir=None):
    if sandbox is None:
        with tempfile.TemporaryDirectory(prefix='launch_params_') as directory:
            return capture(launch_file, launch_args, ws_root, sandbox=directory, matrix_dir=matrix_dir)
    ws_root = ws_root or os.environ.get('RB_WS') or os.getcwd()
    sandbox = os.path.realpath(sandbox)
    os.makedirs(sandbox, exist_ok=True)
    home, tmp = os.path.join(sandbox, 'home'), os.path.join(sandbox, 'tmp')
    os.makedirs(home, exist_ok=True)
    os.makedirs(tmp, exist_ok=True)
    result = {'nodes': [], 'processes': [], 'deferred_callables': [], 'errors': [],
              'probes': [], 'writes': [], 'missing_includes': []}
    deferred = set()
    violations = []

    def refuse(message):
        violations.append(message)
        raise CaptureError(message)

    def node_execute(self, context):
        self._perform_substitutions(context)
        entries = self._Node__expanded_parameter_arguments
        name = self._Node__final_node_name
        if not name or '<node_name_unspecified>' in name:
            name = None
        elif name.startswith('<node_namespace_unspecified>'):
            name = name.replace('<node_namespace_unspecified>', '', 1)
        namespace = self.expanded_node_namespace
        if namespace == '<node_namespace_unspecified>':
            namespace = '/'
        record = {'package': self.node_package, 'executable': self.node_executable,
                  'name': name, 'namespace': namespace,
                  'params': flatten_params(entries, name),
                  'param_files': [value for value, is_file in entries if is_file],
                  'remaps': self.expanded_remapping_rules or [], 'deferred': id(self) in deferred}
        if name is None:
            # A list keeps precedence visible even with JSON sort_keys=True.
            record['blocks'] = [
                {'key': key, 'params': params}
                for key, params in _merged_blocks(entries).items()]
        result['nodes'].append(record)

    def process_execute(self, context):
        executable = self._ExecuteLocal__process_description
        cmd = [perform_substitutions(context, part) for part in executable.cmd]
        result['processes'].append({'cmd': cmd, 'deferred': id(self) in deferred})

    def mark(action, entities):
        if id(action) in deferred and entities:
            deferred.update(id(item) for item in entities)
        return entities

    def timer_execute(self, context):
        return mark(self, list(self._TimerAction__actions))

    def register_execute(self, context):
        handler = self.event_handler
        actions = getattr(handler, '_OnActionEventBase__actions_on_event', None)
        if actions is None:
            actions = getattr(handler, 'entities', None)
        callback = getattr(handler, '_OnActionEventBase__on_event', None)
        if callback is None:
            callback = getattr(handler, '_OnShutdown__on_shutdown', None)
        if callback:
            result['deferred_callables'].append(f'{type(handler).__name__}:{callback.__qualname__}')
        if isinstance(actions, (list, tuple)):
            deferred.update(id(action) for action in actions)
            return list(actions)

    def guarded_argv(args):
        argv = shlex.split(args) if isinstance(args, str) else [str(part) for part in args]
        programs = {'pgrep': (1, 'no match'), 'git': (128, 'not a repository'),
                    'ps': (0, 'empty'), 'dpkg-query': (0, 'empty')}
        program = os.path.basename(argv[0]) if argv else ''
        if program not in programs:
            refuse(f'launch-time subprocess not allowed by the oracle: {argv}')
        code, description = programs[program]
        result['probes'].append({'cmd': argv, 'simulated': description})
        return code

    def empty(kwargs):
        return '' if kwargs.get('text') or kwargs.get('universal_newlines') or kwargs.get('encoding') else b''

    def run(args, *unused, **kwargs):
        code = guarded_argv(args)
        return subprocess.CompletedProcess(args, code, empty(kwargs), empty(kwargs))

    def check_output(args, *unused, **kwargs):
        code = guarded_argv(args)
        if code:
            raise subprocess.CalledProcessError(code, args, output=empty(kwargs))
        return empty(kwargs)

    def call(args, *unused, **kwargs):
        return guarded_argv(args)

    def forbid_process(name):
        def forbidden(*args, **kwargs):
            refuse(f'launch-time process creation not allowed by the oracle: {name}')
        return forbidden

    class Popen:
        def __init__(self, args, *unused, **kwargs):
            self.returncode = guarded_argv(args)
            self.output = empty(kwargs)

        def communicate(self, *args, **kwargs):
            return self.output, self.output

        def wait(self, *args, **kwargs):
            return self.returncode

        def poll(self):
            return self.returncode

    class ErrorHandler(logging.Handler):
        def emit(self, record):
            result['errors'].append(record.getMessage())

    def check_write(path, dir_fd=None):
        if isinstance(path, int) or dir_fd is not None:
            refuse(f'launch-time write outside the oracle sandbox: {path}')
        resolved = os.path.realpath(os.fsdecode(path))
        if os.path.commonpath([sandbox, resolved]) != sandbox:
            refuse(f'launch-time write outside the oracle sandbox: {path}')
        result['writes'].append(resolved)

    def check_device(path, dir_fd=None):
        if isinstance(path, int):
            return
        path = os.fsdecode(path)
        if dir_fd is not None and not os.path.isabs(path):
            path = os.path.join('/proc/self/fd', str(dir_fd), path)
        resolved = os.path.realpath(path)
        if (resolved.startswith('/dev/')
                and resolved not in ('/dev/null', '/dev/zero', '/dev/random', '/dev/urandom')
                and not resolved.startswith('/dev/shm/')):
            refuse(f'launch-time device open refused by the oracle: {resolved}')

    def guard_open(original):
        def guarded(file, mode='r', *args, **kwargs):
            check_device(file)
            if any(flag in mode for flag in 'wax+'):
                check_write(file)
            return original(file, mode, *args, **kwargs)
        return guarded

    def guard_os_open(original):
        def guarded(path, flags, *args, **kwargs):
            check_device(path, kwargs.get('dir_fd'))
            if flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_APPEND | os.O_TRUNC):
                check_write(path, kwargs.get('dir_fd'))
            return original(path, flags, *args, **kwargs)
        return guarded

    def guard_paths(original, indices):
        def guarded(*args, **kwargs):
            for index in indices:
                check_write(args[index], kwargs.get('dir_fd') or kwargs.get('src_dir_fd') or kwargs.get('dst_dir_fd'))
            return original(*args, **kwargs)
        return guarded

    def guard_truncate(original):
        def guarded(path, length):
            check_write(path)
            return original(path, length)
        return guarded

    logger = logging.getLogger('launch')
    handler = ErrorHandler(level=logging.ERROR)
    with ExitStack() as stack:
        for module_name in STUB_MODULES:
            if module_name not in sys.modules and importlib.util.find_spec(module_name) is None:
                stub = types.ModuleType(module_name)
                # The cars use the CUDA ray caster; no ray-casting code runs here.
                stub.SHOULD_USE_CUDA = True
                sys.modules[module_name] = stub
                def remove_stub(name=module_name, module=stub):
                    if sys.modules.get(name) is module:
                        del sys.modules[name]
                stack.callback(remove_stub)
                result['probes'].append({'import': module_name, 'simulated': 'stub'})
        stack.enter_context(patch.dict(os.environ, {'HOME': home, 'TMPDIR': tmp, 'ROS_LOG_DIR': os.path.join(sandbox, 'log')}))
        stack.enter_context(patch.object(tempfile, 'tempdir', tmp))
        stack.enter_context(patch.object(sys, 'dont_write_bytecode', True))
        config = launch.logging.launch_config
        stack.enter_context(patch.object(config, '_log_dir', sandbox))
        stack.enter_context(patch.object(config, 'file_handlers', {}))
        stack.enter_context(patch.object(config, 'get_log_file_handler', lambda *a, **kw: logging.NullHandler()))
        for existing in list(logging.root.manager.loggerDict.values()):
            if isinstance(existing, logging.Logger):
                for old_handler in list(existing.handlers):
                    if isinstance(old_handler, logging.FileHandler):
                        existing.removeHandler(old_handler)
                        stack.callback(existing.addHandler, old_handler)
        for owner, name in [(builtins, 'open'), (io, 'open')]:
            stack.enter_context(patch.object(owner, name, guard_open(getattr(owner, name))))
        stack.enter_context(patch.object(os, 'open', guard_os_open(os.open)))
        stack.enter_context(patch.object(os, 'truncate', guard_truncate(os.truncate)))
        # os.write needs a descriptor already vetted by guarded open/os.open.
        for owner, names, indices in [
                (os, ('mkdir', 'makedirs', 'remove', 'unlink', 'rmdir'), (0,)),
                (os, ('rename', 'replace'), (0, 1)),
                (os, ('symlink',), (1,)),
                (os, ('link',), (0, 1)),
                (shutil, ('rmtree',), (0,)),
                (shutil, ('copy', 'copy2', 'copyfile', 'copytree'), (1,)),
                (shutil, ('move',), (0, 1))]:
            for name in names:
                stack.enter_context(patch.object(owner, name, guard_paths(getattr(owner, name), indices)))
        for owner, attr, replacement in [(Node, 'execute', node_execute),
                (LifecycleNode, 'execute', node_execute),
                (ExecuteLocal, 'execute', process_execute), (TimerAction, 'execute', timer_execute),
                (RegisterEventHandler, 'execute', register_execute),
                (subprocess, 'run', run), (subprocess, 'Popen', Popen),
                (subprocess, 'check_output', check_output), (subprocess, 'call', call),
                (os, 'system', call)]:
            stack.enter_context(patch.object(owner, attr, replacement))
        for name in dir(os):
            if name in ('fork', 'forkpty', 'posix_spawn', 'posix_spawnp') or name.startswith(('spawn', 'exec')):
                stack.enter_context(patch.object(os, name, forbid_process(name)))
        for cls in (GroupAction, IncludeLaunchDescription, OpaqueFunction):
            original = cls.execute
            def propagate(self, context, original=original):
                try:
                    return mark(self, original(self, context))
                except Exception as exc:
                    match = re.search(r"package '([^']+)' not found", str(exc))
                    if isinstance(self, IncludeLaunchDescription) and (match or isinstance(exc, PackageNotFoundError)):
                        package = match.group(1) if match else str(exc.args[0])
                        result['missing_includes'].append(
                            {'package': package, 'deferred': id(self) in deferred})
                        return None
                    raise
            stack.enter_context(patch.object(cls, 'execute', propagate))
        logger.addHandler(handler)
        stack.callback(logger.removeHandler, handler)
        try:
            description = get_launch_description_from_python_launch_file(launch_file)
        except Exception as exc:
            raise CaptureError(f'{type(exc).__name__}: {exc}') from exc
        if violations:
            raise CaptureError(violations[0])
        arguments = [tuple(arg.split(':=', 1)) for arg in launch_args]
        service = LaunchService(noninteractive=True)
        service.include_launch_description(IncludeLaunchDescription(
            LaunchDescriptionSource(description), launch_arguments=arguments))
        rc = service.run(shutdown_when_idle=True)
        if violations:
            raise CaptureError(violations[0])
        if rc or result['errors']:
            raise CaptureError('\n'.join(result['errors']) or f'launch returned {rc}')
    result['nodes'].sort(key=lambda n: (n['namespace'] or '', n['name'] or '', n['package'], n['executable']))
    result['missing_includes'].sort(key=lambda entry: (entry['package'], entry['deferred']))
    return normalize(result, ws_root, sandbox, matrix_dir=matrix_dir)


def diff_captures(a, b):
    def indexed(nodes):
        counts = Counter()
        result = {}
        for n in nodes:
            if n['name']:
                identity = (n['namespace'], n['name'], n['package'], n['executable'])
            else:
                identity = (n['package'], n['executable'])
            key = (*identity, counts[identity])
            counts[identity] += 1
            result[key] = n
        return result
    def label(n, key):
        name = n['name'] or f"{n['package']}/{n['executable']}"
        return name + (f'#{key[-1] + 1}' if key[-1] else '')
    def value(v):
        return json.dumps(v, sort_keys=True)
    left, right = indexed(a['nodes']), indexed(b['nodes'])
    lines = []
    for key in left.keys() - right.keys():
        lines.append('- node ' + label(left[key], key))
    for key in right.keys() - left.keys():
        lines.append('+ node ' + label(right[key], key))
    for key in left.keys() & right.keys():
        x, y = left[key], right[key]
        name = label(x, key)
        for param in x['params'].keys() - y['params'].keys():
            lines.append(f"- {name} {param}={value(x['params'][param])}")
        for param in y['params'].keys() - x['params'].keys():
            lines.append(f"+ {name} {param}={value(y['params'][param])}")
        for param in x['params'].keys() & y['params'].keys():
            # JSON retains scalar types recursively (unlike True == 1 == 1.0).
            if value(x['params'][param]) != value(y['params'][param]):
                lines.append(f"~ {name} {param} {value(x['params'][param])} -> {value(y['params'][param])}")
        for field in ('remaps', 'deferred', 'blocks'):
            if value(x.get(field)) != value(y.get(field)):
                lines.append(f"~ {name} {field} {value(x.get(field))} -> {value(y.get(field))}")
    for sign, first, second in [('-', a, b), ('+', b, a)]:
        counts = Counter(json.dumps(p, sort_keys=True) for p in first['processes']) - Counter(json.dumps(p, sort_keys=True) for p in second['processes'])
        for item, count in counts.items():
            lines.extend([f"{sign} process {value(json.loads(item)['cmd'])}"] * count)
        missing = Counter(entry['package'] for entry in first.get('missing_includes', [])) - Counter(
            entry['package'] for entry in second.get('missing_includes', []))
        for package, count in missing.items():
            lines.extend([f'{sign} missing_include {package}'] * count)
    return sorted(lines)


def load_matrix(path):
    with open(path, encoding='utf-8') as stream:
        matrix = yaml.safe_load(stream)
    if not isinstance(matrix, dict) or not isinstance(matrix.get('entries'), list):
        raise ValueError('invalid matrix')
    ids = [entry['id'] for entry in matrix['entries']]
    if len(ids) != len(set(ids)) or any(not re.fullmatch('[a-z0-9][a-z0-9._-]*', item) for item in ids):
        raise ValueError('invalid or duplicate matrix id')
    return matrix


def resolve_args(entry, car, ws_root, *, matrix_dir=None):
    def replace(match):
        kind = match.group(1)
        from racerbot_car.paths import data_dir, workspace_root
        return str(Path(ws_root) / data_dir(kind, car).relative_to(workspace_root()))
    args = entry.get('args', []) + entry.get('args_by_car', {}).get(car, [])
    if matrix_dir is None and any('{matrix_dir}' in arg for arg in args):
        raise ValueError('matrix_dir is required for {matrix_dir} arguments')
    return [re.sub(r'\{data:([^}]+)\}', replace,
                   arg.replace('{ws}', ws_root).replace('{matrix_dir}', os.path.realpath(matrix_dir) if matrix_dir else ''))
            for arg in args]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    one = sub.add_parser('capture-one')
    one.add_argument('--file')
    one.add_argument('--package')
    one.add_argument('--launch')
    one.add_argument('--ws', default=os.environ.get('RB_WS', os.getcwd()))
    one.add_argument('--matrix-dir')
    one.add_argument('args', nargs=argparse.REMAINDER)
    batch = sub.add_parser('capture')
    batch.add_argument('--matrix', required=True)
    batch.add_argument('--car', choices=['rb1', 'rb2'], required=True)
    batch.add_argument('--out', required=True)
    batch.add_argument('--ws', default=os.environ.get('RB_WS', os.getcwd()))
    batch.add_argument('--only')
    diff = sub.add_parser('diff')
    diff.add_argument('a')
    diff.add_argument('b')
    options = parser.parse_args(argv)
    if options.command == 'capture-one':
        if bool(options.file) == bool(options.package) or bool(options.package) != bool(options.launch):
            parser.error('specify --file or --package with --launch')
        launch_output = io.StringIO()
        try:
            if options.file:
                path = options.file
            else:
                from ros2launch.api import get_share_file_path_from_package
                path = get_share_file_path_from_package(package_name=options.package, file_name=options.launch)
            args = options.args[1:] if options.args[:1] == ['--'] else options.args
            with redirect_stdout(launch_output):
                captured = capture(path, args, options.ws, matrix_dir=options.matrix_dir)
            print(json.dumps(captured, sort_keys=True, indent=1))
            print(launch_output.getvalue(), end='', file=sys.stderr)
            return 0
        except (CaptureError, OSError, ValueError) as exc:
            print(str(exc), file=sys.stderr)
            print(launch_output.getvalue(), end='', file=sys.stderr)
            return 2
    if options.command == 'capture':
        try:
            matrix = load_matrix(options.matrix)
            matrix_dir = os.path.realpath(os.path.dirname(options.matrix))
            entries = [entry for entry in matrix['entries'] if options.car in entry.get('cars', ['rb1', 'rb2']) and (not options.only or entry['id'] == options.only)]
            if options.only and not entries:
                raise ValueError('unknown or unavailable entry: ' + options.only)
            for entry in entries:
                entry['package'], entry['launch']
                resolve_args(entry, options.car, options.ws, matrix_dir=matrix_dir)
        except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
            print(str(exc), file=sys.stderr)
            return 1
        out = Path(options.out)
        out.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, RACERBOT_CAR=options.car)
        for entry in entries:
            name = entry['id']
            cmd = [sys.executable, __file__, 'capture-one', '--package', entry['package'], '--launch', entry['launch'], '--ws', options.ws, '--matrix-dir', matrix_dir, '--', *resolve_args(entry, options.car, options.ws, matrix_dir=matrix_dir)]
            try:
                proc = subprocess.run(cmd, text=True, capture_output=True, timeout=120, env=env)
                error = proc.stderr if proc.returncode else None
                if error is None:
                    try:
                        json.loads(proc.stdout)
                    except json.JSONDecodeError as exc:
                        error = f'invalid capture JSON: {exc}'
            except subprocess.TimeoutExpired:
                error = 'capture timed out after 120 seconds'
            if error is not None:
                (out / (name + '.error.txt')).write_text(error, encoding='utf-8')
                (out / (name + '.json')).unlink(missing_ok=True)
                print(f"FAIL {name}: {error.splitlines()[0] if error else 'capture failed'}")
            else:
                (out / (name + '.json')).write_text(proc.stdout, encoding='utf-8')
                (out / (name + '.error.txt')).unlink(missing_ok=True)
                print('ok ' + name)
        return 0
    a, b = Path(options.a), Path(options.b)
    names = {p.stem for folder in (a, b) for p in folder.glob('*.json')}
    names.update(p.name[:-10] for folder in (a, b) for p in folder.glob('*.error.txt'))
    changed = False
    for name in sorted(names):
        print('== ' + name)
        errors = [(side, folder / (name + '.error.txt')) for side, folder in [('A', a), ('B', b)] if (folder / (name + '.error.txt')).exists()]
        if errors:
            for side, path in errors:
                print(f'error in {side}: {path.read_text(encoding="utf-8").strip()}')
            changed = True
        elif not (a / (name + '.json')).exists() or not (b / (name + '.json')).exists():
            print('only in ' + ('A' if (a / (name + '.json')).exists() else 'B'))
            changed = True
        else:
            lines = diff_captures(json.loads((a / (name + '.json')).read_text()), json.loads((b / (name + '.json')).read_text()))
            print('\n'.join(lines)) if lines else None
            changed |= bool(lines)
    return int(changed)


if __name__ == '__main__':
    sys.exit(main())
