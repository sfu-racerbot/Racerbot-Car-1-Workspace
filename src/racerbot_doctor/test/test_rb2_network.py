"""Network fixtures from car 2's doctor, recorded 2026-10-01."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'racerbot_car'))
from racerbot_doctor import system  # noqa: E402
from racerbot_doctor import car  # noqa: E402
from racerbot_car.profile import load_profile  # noqa: E402


# Captured from `ip -o -4 addr` on car 2, 2026-10-01 (trimmed to 3 lines).
IP_O_4 = (
    '1: lo    inet 127.0.0.1/8 scope host lo\\       valid_lft forever preferred_lft forever\n'
    '4: enP8p1s0    inet 192.168.0.15/24 brd 192.168.0.255 scope global noprefixroute enP8p1s0\\       valid_lft forever preferred_lft forever\n'
    '9: docker0    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0\\       valid_lft forever preferred_lft forever\n'
)


def test_parse_ip_addr_real_output():
    assert system.parse_ip_addr(IP_O_4) == {
        'lo': ['127.0.0.1/8'],
        'enP8p1s0': ['192.168.0.15/24'],
        'docker0': ['172.17.0.1/16'],
    }


def test_parse_ip_addr_garbage_and_empty():
    assert system.parse_ip_addr('') == {}
    assert system.parse_ip_addr('not ip output\n\n3: x\n') == {}


def test_read_carrier(tmp_path):
    net = tmp_path / 'sys/class/net'
    (net / 'up').mkdir(parents=True)
    (net / 'up/carrier').write_text('1\n')
    (net / 'unplugged').mkdir()
    (net / 'unplugged/carrier').write_text('0\n')
    (net / 'admin_down').mkdir()
    root = tmp_path / 'sys'
    assert system.read_carrier(root, 'up') is True
    assert system.read_carrier(root, 'unplugged') is False
    assert system.read_carrier(root, 'admin_down') is None
    assert system.read_carrier(root, 'missing') is None


class _Run:
    """Stand-in for subprocess.run at the process boundary."""
    def __init__(self, ip_out='', ping_rc=0):
        self.ip_out, self.ping_rc, self.calls = ip_out, ping_rc, []

    def __call__(self, argv, **kw):
        self.calls.append(argv)

        class R:
            pass
        r = R()
        r.stdout = self.ip_out if argv[0] == 'ip' else ''
        r.returncode = self.ping_rc if argv[0] == 'ping' else 0
        return r


def _gather(tmp_path, *, probe_lidar_tcp, connect_ok=True, run=None,
            extra_procs=(), vesc_link=None):
    from test_system import _proc, _usb
    sys_root = tmp_path / 'sys'
    _usb(sys_root, '1-2.3', '0483', '5740', 12, 'VESC',
         {'1.0': {'driver': 'cdc_acm', 'tty': ['ttyACM0']}})
    dev = tmp_path / 'dev'
    dev.mkdir()
    (dev / 'sensors').mkdir()
    (dev / 'input').mkdir()
    (dev / 'ttyACM0').write_text('')
    (dev / 'input/js0').write_text('')
    (dev / 'input/event5').write_text('')
    _usb(sys_root, '1-2.4', '046d', 'c21f', 12, 'Wireless Gamepad F710',
         {'1.0': {'driver': 'xpad', 'input': {'input5': 'js0'}}})
    if vesc_link:
        os.symlink(f'../{vesc_link}', dev / 'sensors/vesc')
    proc = tmp_path / 'proc'
    proc.mkdir()
    _proc(proc, 500, ['vesc_driver_node'], {3: str(dev / 'ttyACM0')})
    for pid, argv in extra_procs:
        _proc(proc, pid, argv)
    connects = []

    def connect(addr, timeout):
        connects.append(addr)
        if not connect_ok:
            raise ConnectionRefusedError(111, 'refused')

        class S:
            def close(self):
                pass
        return S()

    snap = system.gather(spec=car.spec_for(load_profile('rb2')),
                         sys_root=sys_root, dev_root=dev, proc_root=proc,
                         run=run or _Run(IP_O_4), connect=connect,
                         probe_lidar_tcp=probe_lidar_tcp, env={'ROS_DISTRO': 'jazzy'})
    return snap, connects


def test_gather_resolves_vesc_symlink_and_holders(tmp_path):
    snap, _ = _gather(tmp_path, probe_lidar_tcp=False, vesc_link='ttyACM0')
    assert snap.vesc_symlink == str(tmp_path / 'dev/ttyACM0')
    assert snap.vesc_symlink_rw is True
    assert snap.tty_holders == {str(tmp_path / 'dev/ttyACM0'): [500]}
    assert snap.input_readable == {'js0': True, 'event5': True}
    assert snap.ifaces['enP8p1s0'].ipv4 == ['192.168.0.15/24']
    assert snap.env == {'ROS_DISTRO': 'jazzy'}


def test_gather_missing_vesc_symlink_is_none(tmp_path):
    snap, _ = _gather(tmp_path, probe_lidar_tcp=False)
    assert snap.vesc_symlink is None
    assert snap.vesc_symlink_rw is False


def test_gather_dangling_vesc_symlink_reports_target(tmp_path):
    snap, _ = _gather(tmp_path, probe_lidar_tcp=False, vesc_link='ttyACM9')
    assert snap.vesc_symlink == str(tmp_path / 'dev/ttyACM9')
    assert snap.vesc_symlink_rw is False


def test_gather_does_not_touch_hokuyo_tcp_unless_asked(tmp_path):
    snap, connects = _gather(tmp_path, probe_lidar_tcp=False)
    assert connects == []
    assert snap.lidar_tcp is None


def test_gather_probes_hokuyo_tcp_when_asked(tmp_path):
    snap, connects = _gather(tmp_path, probe_lidar_tcp=True)
    assert connects == [('192.168.0.10', 10940)]
    assert snap.lidar_tcp is True


def test_gather_tcp_refused_is_false(tmp_path):
    snap, _ = _gather(tmp_path, probe_lidar_tcp=True, connect_ok=False)
    assert snap.lidar_tcp is False


def test_gather_ping_result(tmp_path):
    run = _Run(IP_O_4, ping_rc=1)
    snap, _ = _gather(tmp_path, probe_lidar_tcp=False, run=run)
    assert snap.lidar_ping is False
    assert ['ping', '-c', '1', '-W', '1', '192.168.0.10'] in run.calls


def test_gather_survives_missing_ip_and_ping_binaries(tmp_path):
    def run(argv, **kw):
        raise FileNotFoundError(argv[0])
    snap, _ = _gather(tmp_path, probe_lidar_tcp=False, run=run)
    assert snap.lidar_ping is None
    assert snap.ifaces['enP8p1s0'].ipv4 == []


def test_gather_auto_probes_tcp_when_urg_node_is_down(tmp_path):
    snap, connects = _gather(tmp_path, probe_lidar_tcp='auto')
    assert connects == [('192.168.0.10', 10940)]


def test_gather_auto_never_probes_tcp_while_urg_node_runs(tmp_path):
    snap, connects = _gather(
        tmp_path, probe_lidar_tcp='auto',
        extra_procs=[(600, ['/opt/ros/jazzy/lib/urg_node/urg_node_driver',
                            '--ros-args'])])
    assert connects == []
    assert snap.lidar_tcp is None
