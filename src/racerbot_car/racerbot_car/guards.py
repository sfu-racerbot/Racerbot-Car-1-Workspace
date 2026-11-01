"""Validate per-car hardware calibration overlays."""

from pathlib import Path

import yaml


CALIBRATION_KEYS = {
    'vesc.yaml': {
        '/**': {key: 'required' for key in (
            'speed_to_erpm_gain', 'steering_angle_to_servo_gain',
            'steering_angle_to_servo_offset', 'servo_min', 'servo_max')},
        'vesc_to_odom_node': {'speed_to_erpm_gain': 'required'},
    },
    'sensors.yaml': {'urg_node': {key: 'optional' for key in (
        'serial_port', 'ip_address', 'ip_port')}},
    'joy_teleop.yaml': {'joy_teleop': {
        'human_control.axis_mappings.drive-steering_angle.scale': 'required'}},
    'gap_follow.yaml': {'gap_follow_node': {
        'max_steering_angle': 'required', 'arc_guard_max_correction': 'required'}},
    'pure_pursuit.yaml': {'pure_pursuit_node': {'max_steering_angle': 'required'}},
    'neural_racer.yaml': {'neural_racer_node': {'max_steering_angle': 'required'}},
    'odom_calibration.yaml': {'odom_calibration_node': {key: 'required' for key in (
        'vehicle_model', 'planner_max_steering_rad', 'speed_to_erpm_gain',
        'steering_angle_to_servo_gain', 'steering_angle_to_servo_offset',
        'servo_min', 'servo_max')}},
}


class CalibrationOverlayError(ValueError):
    """An overlay is not a ROS parameter mapping."""


def _flatten(data, prefix=''):
    result = {}
    for key, value in data.items():
        name = f'{prefix}.{key}' if prefix else key
        if isinstance(value, dict):
            result.update(_flatten(value, name))
        else:
            result[name] = value
    return result


def overlay_blocks(path: Path) -> dict[str, dict]:
    try:
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise CalibrationOverlayError(f'{path}: invalid YAML: {exc}') from exc
    if not isinstance(data, dict):
        raise CalibrationOverlayError(f'{path}: expected a mapping')
    result = {}
    for block, value in data.items():
        if not isinstance(value, dict) or not isinstance(value.get('ros__parameters'), dict):
            raise CalibrationOverlayError(f'{path}: {block} needs a ros__parameters mapping')
        result[block] = _flatten(value['ros__parameters'])
    return result


def check_calibration_overlay(path: Path, config_name: str) -> list[str]:
    if config_name not in CALIBRATION_KEYS:
        return [f'{path}: {config_name} has no calibration allow-list.']
    allowed = CALIBRATION_KEYS[config_name]
    blocks = overlay_blocks(path)
    problems = []
    for block, params in blocks.items():
        if block not in allowed:
            problems.append(f'{path}: {block} is not a calibration block for {config_name}.')
            continue
        for key in params:
            if key not in allowed[block]:
                problems.append(f'{path}: {block}.{key} is not a calibration key for {config_name}.')
    for block, keys in allowed.items():
        for key, requirement in keys.items():
            if requirement == 'required' and key not in blocks.get(block, {}):
                problems.append(f'{path}: {block}.{key} is a missing required calibration key for {config_name}.')
    return problems
