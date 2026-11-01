"""Task 7 contract: executable code and config must name a car in data paths."""
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]
EXCLUDED = {'racerbot_a', 'racerbot_b', 'web_dashboards', 'realsense-ros',
            'particle_filter'}
BARE = re.compile(r'data/(?:racerbot_(?:runs|auto|sim)|imu_calibration)\b')


def find_bare_data_paths(root):
    hits = []
    for base in ('src', 'tools'):
        for path in (root / base).rglob('*'):
            rel = path.relative_to(root)
            if base == 'src' and rel.parts[1] in EXCLUDED:
                continue
            if any(p in {'__pycache__', '.git', '.pytest_cache'} for p in rel.parts):
                continue
            if not path.is_file():
                continue
            if path.suffix not in {'.py', '.yaml', '.sh'}:
                if path.suffix:
                    continue
                with path.open('rb') as stream:
                    if stream.read(2) != b'#!':
                        continue
            for number, line in enumerate(path.read_text().splitlines(), 1):
                if BARE.search(line):
                    hits.append(f'{rel}:{number}: {line.strip()}')
    return hits


def test_no_bare_data_paths():
    hits = find_bare_data_paths(ROOT)
    assert not hits, '\n'.join(hits)


def test_nonexecutable_extensionless_script_is_checked(tmp_path):
    script = tmp_path / 'tools' / 'nested' / 'record'
    script.parent.mkdir(parents=True)
    bare = 'data/' + 'racerbot_runs'
    script.write_text(f'#!/usr/bin/env python3\noutput = "{bare}"\n')
    script.chmod(0o644)
    assert find_bare_data_paths(tmp_path) == [f'tools/nested/record:2: output = "{bare}"']
