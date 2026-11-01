import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racerbot_car import hardware, profile

FIXTURES = Path(__file__).parent / 'fixtures'


def test_usb_ids_read_from_fixture():
    assert hardware.usb_ids_present(FIXTURES / 'usb') == {'15d1:0000', '8086:0b3a'}


def test_usb_ids_missing_root_empty(tmp_path):
    assert hardware.usb_ids_present(tmp_path / 'missing') == set()


def test_usb_ids_skips_dirs_without_ids(tmp_path):
    device = tmp_path / '1-1'
    device.mkdir()
    (device / 'idVendor').write_text('15d1')
    assert hardware.usb_ids_present(tmp_path) == set()


def profiles():
    return profile.all_profiles(FIXTURES / 'cars_ok')


def test_contradiction_rb1_with_realsense_present():
    cars = profiles()
    message = hardware.contradiction(cars[0], cars, {'8086:0b3a'})
    assert message and 'rb2' in message and '8086:0b3a' in message


def test_contradiction_rb2_with_hokuyo_present():
    cars = profiles()
    message = hardware.contradiction(cars[1], cars, {'15d1:0000'})
    assert message and 'rb1' in message and '15d1:0000' in message


def test_no_contradiction_when_sensor_unplugged():
    cars = profiles()
    assert hardware.contradiction(cars[0], cars, set()) is None


def test_no_contradiction_own_ids_present():
    cars = profiles()
    assert hardware.contradiction(cars[0], cars, {'15d1:0000'}) is None
