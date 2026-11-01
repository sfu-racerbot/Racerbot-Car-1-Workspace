"""Task 7 contract: executable code and config must name a car in data paths."""
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]
EXCLUDED = {'racerbot_a', 'racerbot_b', 'web_dashboards', 'realsense-ros',
            'particle_filter'}
BARE = re.compile(r'data/(?:racerbot_(?:runs|auto|sim)|imu_calibration)\b')


def test_no_bare_data_paths():
    hits = []
    for base in ('src', 'tools'):
        for path in (ROOT / base).rglob('*'):
            rel = path.relative_to(ROOT)
            if base == 'src' and rel.parts[1] in EXCLUDED:
                continue
            if any(p in {'__pycache__', '.git', '.pytest_cache'} for p in rel.parts):
                continue
            if not path.is_file():
                continue
            if path.suffix not in {'.py', '.yaml', '.sh'}:
                if path.suffix or path.parent.name != 'scripts':
                    continue
            for number, line in enumerate(path.read_text().splitlines(), 1):
                if BARE.search(line):
                    hits.append(f'{rel}:{number}: {line.strip()}')
    assert not hits, '\n'.join(hits)
