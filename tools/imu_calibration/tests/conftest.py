"""Keep unsourced simulation tests on the selected car's actual mount."""
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/racerbot_car'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture(autouse=True)
def selected_test_car(monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', os.environ.get('RACERBOT_CAR') or 'rb1')
