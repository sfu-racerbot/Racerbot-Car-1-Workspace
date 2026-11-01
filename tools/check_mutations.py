#!/usr/bin/env python3
"""Prove a feature's tests can fail (CLAUDE.md, Hard rule 1), from a spec file of deliberate breaks.

    python3 tools/check_mutations.py tools/mutations/slam_reliability.json            # every break
    python3 tools/check_mutations.py tools/mutations/slam_reliability.json --only map_localize

A spec is JSON: {"copy": [directories], "mutations": [{"file", "old", "new", "tests"}, ...]}. The directories
are copied into a temporary tree and each break is applied there, one at a time, so the real files are never
touched and nothing else running can import a broken file. Each break's tests must FAIL, and must pass on the
unbroken tree first, or the break proves nothing.

Tests run with ROS sourced from this checkout's install and on ROS_DOMAIN_ID 79, localhost only, as CLAUDE.md
requires; the copied packages come first on PYTHONPATH so the install's own copies are not what gets tested.
tools/neural_racer/check_mutations.py is the same idea, written for one feature before this one.

Exit 0 only if every break was caught.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_spec(path):
    spec = json.loads(Path(path).read_text())
    for index, mutation in enumerate(spec['mutations']):
        missing = {'file', 'old', 'new', 'tests'} - set(mutation)
        if missing:
            raise ValueError(f'mutation {index} has no {", ".join(sorted(missing))}')
        if mutation['old'] == mutation['new']:
            raise ValueError(f'mutation {index} changes nothing')
    return spec


def name_of(mutation):
    first = mutation['old'].strip().splitlines()[0][:50]
    return f"{Path(mutation['file']).stem}: {first}"


def apply(text, old, new):
    """text with old replaced by new. old must occur exactly once, or which line is broken is a guess."""
    count = text.count(old)
    if count != 1:
        raise LookupError(f'break text found {count} times, not once')
    return text.replace(old, new)


def build_tree(tmp, root, copy):
    for rel in copy:
        shutil.copytree(root / rel, tmp / rel, ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))


def environment(tmp, copy):
    # Packages under src/<pkg>/<pkg> import as <pkg>: their parent goes first on the path.
    packages = [str(tmp / rel) for rel in copy if rel.startswith('src/')]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', ROS_DOMAIN_ID='79',
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
    env['PYTHONPATH'] = os.pathsep.join(packages + [p for p in [env.get('PYTHONPATH', '')] if p])
    return env


def run_tests(tmp, tests, env, setup):
    # No bytecode caches: a break that keeps a file's size, written in the same second, would run the old .pyc.
    for cache in tmp.rglob('__pycache__'):
        shutil.rmtree(cache, ignore_errors=True)
    command = 'python3 -m pytest -x -q -p no:cacheprovider ' + ' '.join(tests)
    if setup:
        command = f'source /opt/ros/jazzy/setup.bash && source {setup} && export PYTHONPATH={env["PYTHONPATH"]}:$PYTHONPATH && {command}'
    return subprocess.run(['bash', '-c', command], cwd=tmp, env=env, capture_output=True, text=True,
                          timeout=1800).returncode


def check(spec, root=ROOT, only='', setup=None):
    """[(name, outcome)], outcome one of caught, MISSED, not-found, red-baseline."""
    chosen = [m for m in spec['mutations'] if only in name_of(m) or only in m['file']]
    results = []
    green = {}
    with tempfile.TemporaryDirectory() as directory:
        tmp = Path(directory)
        build_tree(tmp, root, spec['copy'])
        env = environment(tmp, spec['copy'])
        for mutation in chosen:
            name = name_of(mutation)
            path = tmp / mutation['file']
            original = path.read_text()
            try:
                broken = apply(original, mutation['old'], mutation['new'])
            except LookupError:
                results.append((name, 'not-found'))
                print(f'??      {name}: break text not found once', flush=True)
                continue
            key = tuple(mutation['tests'])
            if key not in green:
                green[key] = run_tests(tmp, mutation['tests'], env, setup) == 0
            if not green[key]:
                results.append((name, 'red-baseline'))
                print(f'!!      {name}: its tests fail on the unbroken tree', flush=True)
                continue
            path.write_text(broken)
            try:
                caught = run_tests(tmp, mutation['tests'], env, setup) != 0
            finally:
                path.write_text(original)
            results.append((name, 'caught' if caught else 'MISSED'))
            print(f"{'caught' if caught else 'MISSED'}  {name}", flush=True)
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('spec', type=Path)
    parser.add_argument('--only', default='', help='only breaks whose file or first line contains this')
    parser.add_argument('--no-ros', action='store_true', help="don't source ROS (pure-Python tests only)")
    args = parser.parse_args(argv)
    setup = None if args.no_ros else str(ROOT / 'install' / 'setup.bash')
    results = check(load_spec(args.spec), only=args.only, setup=setup)
    bad = [(name, outcome) for name, outcome in results if outcome != 'caught']
    print(f'\n{len(results) - len(bad)}/{len(results)} breaks caught')
    for name, outcome in bad:
        print(f'  - {outcome}: {name}')
    return 0 if results and not bad else 1


if __name__ == '__main__':
    sys.exit(main())
