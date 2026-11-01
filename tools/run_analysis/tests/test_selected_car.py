"""Analysis entry points resolve the selected car before opening a run."""

import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('module,args', [
    ('fit_sim', ['missing-run']), ('analyze_run', ['missing-run']),
])
def test_entry_point_imports_without_car_then_refuses(module, args):
    env = dict(os.environ)
    env.pop('RACERBOT_CAR', None)
    code = (f'from run_analysis import {module} as m; '
            f'print("imported", flush=True); m.main({args!r})')
    env['PYTHONPATH'] = str(ROOT / 'tools')
    result = subprocess.run([sys.executable, '-c', code], cwd=ROOT, env=env,
                            capture_output=True, text=True, check=False)
    assert 'imported' in result.stdout
    assert result.returncode != 0
    assert 'RACERBOT_CAR' in result.stderr
