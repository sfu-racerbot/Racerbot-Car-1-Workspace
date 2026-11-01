from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racerbot_car import guards, layers


@pytest.mark.parametrize('config', layers.SHARED_CONFIGS)
def test_shared_config_holds_no_calibration(config):
    blocks = guards.overlay_blocks(layers.shared_config_path(config))
    for block, keys in guards.CALIBRATION_KEYS[config].items():
        for key in keys:
            assert key not in blocks.get(block, {}), f'{config}: {block}.{key}'
