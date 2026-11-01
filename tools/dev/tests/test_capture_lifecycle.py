"""Back-to-back real launch captures must not cross sandbox lifetimes."""
import gc
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from launch_params import capture


def test_inline_parameter_captures_back_to_back(tmp_path):
    launch_file = tmp_path / 'inline_launch.py'
    launch_file.write_text(
        'import gc\n'
        'from launch import LaunchDescription\n'
        'from launch_ros.actions import Node\n'
        'def generate_launch_description():\n'
        '    gc.collect()\n'
        '    node = Node(\n'
        '        package="demo", executable="not_started", name="inline",\n'
        '        parameters=[{"gain": 3}])\n'
        '    node.cycle = node\n'
        '    return LaunchDescription([node])\n')
    # Retain cycles until the next launch forces collection inside its guard.
    # capture() must finalize the preceding launch before installing that guard.
    enabled = gc.isenabled()
    gc.disable()
    try:
        first = capture(str(launch_file), [])
        second = capture(str(launch_file), [])
        assert first['nodes'][0]['params']['gain'] == 3
        assert second['nodes'][0]['params']['gain'] == 3
        assert second['errors'] == []
    finally:
        if enabled:
            gc.enable()
        gc.collect()
