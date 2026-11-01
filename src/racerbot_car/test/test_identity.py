import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_car import identity


def test_serial_strips_nul(tmp_path):
    path = tmp_path / 'serial'
    path.write_bytes(b'1421723021900\x00')
    assert identity.read_serial(str(path)) == '1421723021900'


def test_serial_strips_whitespace_and_nul(tmp_path):
    path = tmp_path / 'serial'
    path.write_bytes(b' \n1423025035537\x00 \n')
    assert identity.read_serial(str(path)) == '1423025035537'


def test_serial_missing_file_is_none(tmp_path):
    assert identity.read_serial(str(tmp_path / 'missing')) is None


def test_serial_empty_is_none(tmp_path):
    path = tmp_path / 'serial'
    path.write_bytes(b'\x00\n')
    assert identity.read_serial(str(path)) is None


def test_serial_strips_embedded_nul(tmp_path):
    path = tmp_path / 'serial'
    path.write_bytes(b'12\x003')
    assert identity.read_serial(str(path)) == '123'


def test_known_car_ids_requires_pattern_and_car_yaml(tmp_path):
    for name in ('rb1', 'rb2', 'notacar'):
        (tmp_path / name).mkdir()
    for name in ('rb1', 'notacar'):
        (tmp_path / name / 'car.yaml').touch()
    assert identity.known_car_ids(tmp_path) == {'rb1'}


def machines(tmp_path, text):
    path = tmp_path / 'machines.yaml'
    path.write_text(text)
    return identity.load_machines(path, {'rb1', 'rb2'})


def test_machines_loads_quoted_serial(tmp_path):
    assert machines(tmp_path, '"001": rb2\n') == {'001': 'rb2'}


def test_machines_unquoted_serial_rejected(tmp_path):
    with pytest.raises(ValueError, match='quote'):
        machines(tmp_path, '123: rb1\n')


def test_machines_duplicate_serial_rejected(tmp_path):
    with pytest.raises(ValueError, match='123'):
        machines(tmp_path, '"123": rb1\n"123": rb2\n')


def test_machines_unknown_id_rejected(tmp_path):
    with pytest.raises(ValueError, match='rb9'):
        machines(tmp_path, '"123": rb9\n')


def test_machines_bad_id_pattern_rejected(tmp_path):
    with pytest.raises(ValueError, match='car1'):
        machines(tmp_path, '"123": car1\n')


def test_machines_empty_file_is_empty(tmp_path):
    assert machines(tmp_path, '') == {}


def test_machines_comments_only_is_empty(tmp_path):
    assert machines(tmp_path, '  # comment\n\n # another\n') == {}


@pytest.mark.parametrize('document', ['~\n', 'null\n'])
def test_machines_explicit_null_rejected(tmp_path, document):
    with pytest.raises(ValueError, match='mapping'):
        machines(tmp_path, document)


def test_machines_non_mapping_rejected(tmp_path):
    with pytest.raises(ValueError, match='mapping'):
        machines(tmp_path, '- rb1\n')


def resolve(env, serial):
    return identity.resolve_car_id(env, serial, {'123': 'rb1', '456': 'rb2'}, {'rb1', 'rb2'})


def test_known_serial_resolves():
    assert resolve({}, '456') == 'rb2'


def test_known_serial_with_matching_env_resolves():
    assert resolve({'RACERBOT_CAR': 'rb2'}, '456') == 'rb2'


def test_known_serial_with_other_env_refuses():
    with pytest.raises(identity.CarMismatchError, match='rb2') as exc:
        resolve({'RACERBOT_CAR': 'rb2'}, '123')
    assert all(text in str(exc.value) for text in ('rb1', '123', 'unset', 'RACERBOT_CAR'))


def test_known_serial_ignores_empty_env():
    assert resolve({'RACERBOT_CAR': ''}, '456') == 'rb2'


def test_unknown_serial_without_env_refuses_and_prints_line():
    with pytest.raises(identity.UnknownCarError, match=re.escape('"999": <car id>')) as exc:
        resolve({}, '999')
    assert 'cars/machines.yaml' in str(exc.value)


def test_unknown_serial_with_env_resolves():
    assert resolve({'RACERBOT_CAR': 'rb2'}, '999') == 'rb2'


def test_no_serial_with_env_resolves():
    assert resolve({'RACERBOT_CAR': 'rb2'}, None) == 'rb2'


def test_no_serial_without_env_refuses():
    with pytest.raises(identity.UnknownCarError, match='RACERBOT_CAR'):
        resolve({}, None)


def test_env_not_a_known_car_refuses():
    with pytest.raises(identity.UnknownCarError, match='rb9'):
        resolve({'RACERBOT_CAR': 'rb9'}, None)


def test_current_car_id_uses_env_serial_and_cars_dir(tmp_path, monkeypatch):
    (tmp_path / 'rb2').mkdir()
    (tmp_path / 'rb2' / 'car.yaml').touch()
    (tmp_path / 'machines.yaml').write_text('"456": rb2\n')
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(tmp_path))
    monkeypatch.setenv('RACERBOT_CAR', 'rb2')
    monkeypatch.setattr(identity, 'read_serial', lambda: '456')
    assert identity.current_car_id() == 'rb2'
