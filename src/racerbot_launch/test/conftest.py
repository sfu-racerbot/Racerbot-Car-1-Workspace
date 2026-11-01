import pytest


@pytest.fixture(autouse=True)
def select_test_car(monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
