import copy
import json
import math
from pathlib import Path
import numpy as np
import pytest
import yaml

from opponent_perception.configuration import load_config, parse_config
from opponent_perception.offline import read_json


def config():
    return {'schema_version': 1, 'calibrated': True, 'tracking_frame': 'odom',
            'tracking': {}, 'max_error_px': 2, 'max_range_m': 8,
            'tags': [{'id': 7, 'vehicle_id': 'rb1', 'size_m': .12,
                      'opponent_from_tag': np.eye(4).tolist()}]}


def test_measured_config_preserves_mount_and_filter_settings(tmp_path):
    # oracle: spec README.md: configured transform/gain/ID are the values used by the pipeline.
    raw = config()
    raw['tags'][0]['opponent_from_tag'][2][3] = .23
    raw['tracking'] = {'velocity_gain': .2}
    path = tmp_path / 'mount.yaml'
    path.write_text(yaml.safe_dump(raw), encoding='utf-8')
    parsed = load_config(path)
    assert parsed.tags[7].opponent_from_tag[2, 3] == .23
    assert parsed.tags[7].vehicle_id == 'rb1'
    assert parsed.tracking.velocity_gain == .2


@pytest.mark.parametrize('key,value', [
    ('calibrated', False), ('schema_version', 2),
    ('schema_version', True), ('tracking_frame', 'base_link'),
    ('tracking', {'invented': 1}), ('tags', []),
    ('max_error_px', math.nan), ('max_range_m', -1)])
def test_config_refuses_unsafe_or_unknown_values(key, value):
    # oracle: spec README.md: unmeasured, unversioned or malformed setup is refused.
    raw = config()
    raw[key] = value
    with pytest.raises(ValueError,
                       match='calibration|version|fixed|tracking|tags|positive|unknown'):
        parse_config(raw)


@pytest.mark.parametrize('field,value', [
    ('id', -1), ('id', 587), ('size_m', 0),
    ('size_m', math.inf), ('opponent_from_tag', None), ('vehicle_id', '')])
def test_bad_tag_configuration(field, value):
    # oracle: spec README.md: positive size, dictionary ID and measured rigid mount required.
    raw = config()
    raw['tags'][0][field] = value
    with pytest.raises(ValueError, match='tag|transform'):
        parse_config(raw)


def test_missing_unknown_and_duplicate_config_fields():
    # oracle: spec README.md: configuration is strict, one tag per vehicle in v1.
    unknown, missing, duplicate = config(), config(), config()
    unknown['typo'] = 1
    del missing['tracking']
    duplicate['tags'].append(copy.deepcopy(duplicate['tags'][0]))
    for raw in (unknown, missing, duplicate):
        with pytest.raises(ValueError, match='keys|duplicate'):
            parse_config(raw)


@pytest.mark.parametrize('content', [b'\xff', b'[truncated', b'', b'x'*65537])
def test_configuration_file_errors_are_explicit(tmp_path, content):
    # oracle: spec README.md: bounded UTF-8 YAML; no silent default calibration.
    path = tmp_path / 'config.yaml'
    path.write_bytes(content)
    with pytest.raises(ValueError, match='UTF-8|keys|64 KiB'):
        load_config(path)


def test_shipped_template_is_refused():
    # oracle: spec config/opponent_perception.yaml: calibration is deliberately incomplete.
    path = Path(__file__).parents[1] / 'config' / 'opponent_perception.yaml'
    with pytest.raises(ValueError, match='calibration is not confirmed'):
        load_config(path)


def manifest():
    return {'stamp_sec': 1, 'image': 'frame.png', 'tracking_from_camera': np.eye(4).tolist()}


def test_manifest_keeps_acquisition_stamp_and_transform():
    # oracle: spec README.md: image timestamp and transform are supplied per frame, never inferred.
    raw = manifest()
    raw['tracking_from_camera'][0][3] = -2
    parsed = read_json(json.dumps(raw))
    assert parsed['stamp_sec'] == 1
    assert parsed['tracking_from_camera'][0][3] == -2


@pytest.mark.parametrize('line', ['', '{', '[]', '{"unknown":1}', '\ufffd', ' '*65537,
                                  '{"stamp_sec":NaN,"image":"x","tracking_from_camera":[]}'])
def test_bad_manifest_records_are_refused(line):
    # oracle: spec README.md: strict finite bounded JSON object.
    with pytest.raises(ValueError, match='manifest|JSON'):
        read_json(line)


@pytest.mark.parametrize('field,value', [
    ('stamp_sec', -1), ('stamp_sec', True), ('image', ''), ('tracking_from_camera', [])])
def test_manifest_bad_fields(field, value):
    # oracle: spec README.md: reject wrong types and invalid timestamps/transforms.
    raw = manifest()
    raw[field] = value
    with pytest.raises(ValueError, match='manifest|transform'):
        read_json(json.dumps(raw))
