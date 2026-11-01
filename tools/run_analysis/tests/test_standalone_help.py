"""Exercise real unsourced entry points, without pytest's import-path setup."""
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('tool', ['grip_replay.py', 'slam_replay.py'])
def test_analysis_help_without_sourced_workspace(tmp_path, tool):
    result = subprocess.run(
        ['env', '-i', f'HOME={os.environ["HOME"]}', 'PATH=/usr/bin:/bin',
         'python3', str(ROOT / 'tools/run_analysis' / tool), '--help'],
        cwd=tmp_path, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert '--car' in result.stdout
