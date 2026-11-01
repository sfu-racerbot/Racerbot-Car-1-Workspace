"""The two hardware selections must not probe each other's devices."""
import os
import sys
import builtins

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'racerbot_car'))

from racerbot_car.profile import load_profile  # noqa: E402
from racerbot_doctor import car, checks, system  # noqa: E402
from test_checks import healthy_live, healthy_snap  # noqa: E402


def test_doctor_rb1_never_mentions_ethernet_or_realsense(tmp_path, monkeypatch):
    spec = car.spec_for(load_profile('rb1'))
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        raise AssertionError('rb1 must not run a network command')

    def connect(addr, timeout):
        calls.append(addr)
        raise AssertionError('rb1 must not open a network connection')

    def carrier(*args):
        raise AssertionError('rb1 must not read network carrier')

    monkeypatch.setattr(system, 'read_carrier', carrier)
    system.gather(spec=spec, sys_root=tmp_path, dev_root=tmp_path,
                  proc_root=tmp_path, run=run, connect=connect)
    snap = healthy_snap()
    reports = [checks.lidar_ladder(spec.lidar, snap, healthy_live()),
               checks.camera_ladder(spec.camera, snap, healthy_live())]
    text = ' '.join(f'{layer.name} {layer.detail} {layer.fix}'
                    for report in reports for layer in report.layers).lower()
    assert not any(word in text for word in ('ping', 'carrier', 'realsense', 'ethernet', 'host ip'))
    assert calls == []


def test_doctor_rb2_never_opens_tty_or_video(tmp_path, monkeypatch):
    spec = car.spec_for(load_profile('rb2'))
    (tmp_path / 'sensors').mkdir()
    (tmp_path / 'sensors/hokuyo').write_text('')
    (tmp_path / 'video0').write_text('')
    touched = []
    real_lexists = os.path.lexists
    real_exists = os.path.exists
    real_access = os.access
    real_stat = os.stat
    real_open = builtins.open

    def record(fn):
        def wrapper(path, *args, **kwargs):
            touched.append(os.fspath(path))
            return fn(path, *args, **kwargs)
        return wrapper

    monkeypatch.setattr(os.path, 'lexists', record(real_lexists))
    monkeypatch.setattr(os.path, 'exists', record(real_exists))
    monkeypatch.setattr(os, 'access', record(real_access))
    monkeypatch.setattr(os, 'stat', record(real_stat))
    monkeypatch.setattr(builtins, 'open', record(real_open))
    from test_rb2_network import _Run
    system.gather(spec=spec, sys_root=tmp_path, dev_root=tmp_path,
                  proc_root=tmp_path, run=_Run(),
                  probe_lidar_tcp=False)
    assert not any(path.endswith('/sensors/hokuyo') or path.startswith(str(tmp_path / 'video'))
                   for path in touched)
