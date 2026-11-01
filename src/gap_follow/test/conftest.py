import pytest
from racerbot_car.layers import layered


def car_config_args(path):
    return [arg for file in layered(path, 'gap_follow.yaml', 'rb1')
            for arg in ('--params-file', file)]


@pytest.fixture(autouse=True)
def select_test_car(monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
