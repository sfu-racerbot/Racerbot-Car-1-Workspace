"""Make the source car-path package available to unsourced tests."""
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'racerbot_car'))


@pytest.fixture(autouse=True)
def selected_test_car(monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
