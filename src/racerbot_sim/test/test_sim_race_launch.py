import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def test_referee_publisher_is_result_only():
    path = os.path.join(os.path.dirname(__file__), '..', 'racerbot_sim', 'race_referee_node.py')
    source = open(path, encoding='utf-8').read()
    assert source.count('create_publisher') == 1
    assert "'/sim/race_result'" in source
    for forbidden in ('/drive', '/ackermann_cmd', '/joy', '/scan', '/odom'):
        assert forbidden not in source


def test_race_launch_overrides_car2_topics_after_yaml_and_isolates_deadman():
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'sim_race_launch.py')
    source = open(path, encoding='utf-8').read()
    assert "drive_topic='/car2/drive'" in source
    assert source.index("drive_topic='/car2/drive'") > source.index('car2 = _params')
    assert "'car2_hold_deadman': LaunchConfiguration('car2_hold_deadman')" in source
    assert "'hold_deadman': 'true'" in source
