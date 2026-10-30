"""Duration floor must fit the measured short course and physical turn rate."""
import math
from pathlib import Path

import yaml

from pure_pursuit.auto_map_race_node import AutoMapRaceNode


def test_duration_gate_fits_short_course_and_fastest_turn():
    src = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((src / 'pure_pursuit/config/auto_map_race.yaml').read_text())[
        'auto_map_race_node']['ros__parameters']
    speed = yaml.safe_load((src / 'gap_follow/config/gap_follow.yaml').read_text())[
        'gap_follow_node']['ros__parameters']['max_speed']
    duration = params['minimum_lap_duration_sec']
    # oracle: spec docs/hardware-reference.md: autonomy radius is 1.05 m.
    # Recorded run 20261026-194913: shortest mapped course is 28.7 m.
    fastest_turn_sec = params['minimum_lap_turn_deg'] / math.degrees(speed / 1.05)
    assert duration >= 0
    assert fastest_turn_sec < duration < 28.7 / speed


def test_declared_duration_default_matches_yaml():
    import ast
    import inspect

    params = yaml.safe_load((Path(__file__).resolve().parents[1]
                             / 'config/auto_map_race.yaml').read_text())[
        'auto_map_race_node']['ros__parameters']
    tree = ast.parse(inspect.getsource(AutoMapRaceNode))
    declarations = [call for call in ast.walk(tree)
                    if isinstance(call, ast.Call)
                    and isinstance(call.func, ast.Attribute)
                    and call.func.attr == 'declare_parameter'
                    and call.args and isinstance(call.args[0], ast.Constant)
                    and call.args[0].value == 'minimum_lap_duration_sec']
    # oracle: invariant, launching without YAML must use the same floor.
    assert [ast.literal_eval(call.args[1]) for call in declarations] == [
        params['minimum_lap_duration_sec']]
