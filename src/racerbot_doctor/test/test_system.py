"""
Tests for system.py -- reading the OS (sysfs, /proc, /dev, `ip`).

Pure filesystem + string handling, no rclpy:
    python3 -m pytest src/racerbot_doctor/test/ -v

Oracle: the fake sysfs tree below mirrors the real layout recorded on car 1
on 2026-10-20 (`ls /sys/bus/usb/devices`, `lsusb`, driver links): the
Hokuyo at 1-2.2 (15d1:0000, speed 12, cdc_acm on 1.0 and 1.1, ttyACM0), the
F710 at 1-2.3 (046d:c21f, speed 12, xpad, input/input5/{js0,event5}), the
VESC at 1-2.4 (0483:5740, cdc_acm, ttyACM1). The webcam was unplugged that
day; its entry (uvcvideo, video4linux/video0..1) follows the standard UVC
sysfs layout. Tests never touch the real /sys or /proc -- everything passes
an explicit tmp_path root.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_doctor.system import (  # noqa: E402
    gather, read_processes, read_usb_devices, tty_holders)


def _usb(sys_root, name, vid, pid, speed, product, interfaces):
    """interfaces: {'1.0': {'driver': 'cdc_acm', 'tty': ['ttyACM0'], ...}}"""
    dev = sys_root / 'bus/usb/devices' / name
    dev.mkdir(parents=True)
    (dev / 'idVendor').write_text(vid + '\n')
    (dev / 'idProduct').write_text(pid + '\n')
    if speed is not None:
        (dev / 'speed').write_text(f'{speed}\n')
    (dev / 'product').write_text(product + '\n')
    drivers_dir = sys_root / 'bus/usb/drivers'
    for intf, spec in interfaces.items():
        idir = sys_root / 'bus/usb/devices' / f'{name}:{intf}'
        idir.mkdir()
        if spec.get('driver'):
            (drivers_dir / spec['driver']).mkdir(parents=True, exist_ok=True)
            os.symlink(drivers_dir / spec['driver'], idir / 'driver')
        for tty in spec.get('tty', []):
            (idir / 'tty' / tty).mkdir(parents=True)
        for video in spec.get('video', []):
            (idir / 'video4linux' / video).mkdir(parents=True)
        for inp, js in spec.get('input', {}).items():
            (idir / 'input' / inp / js).mkdir(parents=True)
            (idir / 'input' / inp / f'event{inp[-1]}').mkdir()


def _car1_tree(tmp_path):
    root = tmp_path / 'sys'
    _usb(root, '1-2', '0bda', '5489', 480, '4-Port USB 2.0 Hub',
         {'1.0': {'driver': 'hub'}})
    _usb(root, '1-2.2', '15d1', '0000', 12, 'URG-Series USB Driver',
         {'1.0': {'driver': 'cdc_acm', 'tty': ['ttyACM0']},
          '1.1': {'driver': 'cdc_acm'}})
    _usb(root, '1-2.3', '046d', 'c21f', 12, 'Wireless Gamepad F710',
         {'1.0': {'driver': 'xpad', 'input': {'input5': 'js0'}}})
    _usb(root, '1-2.4', '0483', '5740', 12, 'ChibiOS/RT Virtual COM Port',
         {'1.0': {'driver': 'cdc_acm', 'tty': ['ttyACM1']},
          '1.1': {'driver': 'cdc_acm'}})
    _usb(root, '2-1.1', '046d', '0825', 480, 'Webcam C270',
         {'1.0': {'driver': 'uvcvideo', 'video': ['video0', 'video1']},
          '1.1': {'driver': 'uvcvideo'},
          '1.2': {'driver': 'snd-usb-audio'}})
    return root


def _by_vid(devs):
    return {(d.vid, d.pid): d for d in devs}


def test_reads_vid_pid_speed_product(tmp_path):
    devs = _by_vid(read_usb_devices(_car1_tree(tmp_path)))
    lidar = devs[('15d1', '0000')]
    assert lidar.sysname == '1-2.2'
    assert lidar.speed_mbps == 12.0
    assert lidar.product == 'URG-Series USB Driver'
    assert devs[('046d', '0825')].speed_mbps == 480.0


def test_collects_drivers_and_device_nodes_across_interfaces(tmp_path):
    devs = _by_vid(read_usb_devices(_car1_tree(tmp_path)))
    cam = devs[('046d', '0825')]
    assert sorted(cam.drivers) == ['snd-usb-audio', 'uvcvideo']
    assert sorted(cam.videos) == ['video0', 'video1']
    pad = devs[('046d', 'c21f')]
    assert pad.joysticks == ['js0']
    assert pad.events == ['event5']         # what joy_node (SDL2) opens
    assert pad.drivers == ['xpad']
    assert devs[('15d1', '0000')].ttys == ['ttyACM0']
    vesc = devs[('0483', '5740')]
    assert vesc.ttys == ['ttyACM1']
    assert vesc.drivers == ['cdc_acm']   # deduplicated across interfaces


def test_interface_dirs_are_not_reported_as_devices(tmp_path):
    names = sorted(d.sysname for d in read_usb_devices(_car1_tree(tmp_path)))
    assert names == ['1-2', '1-2.2', '1-2.3', '1-2.4', '2-1.1']


def test_device_without_driver_link_has_no_drivers(tmp_path):
    root = tmp_path / 'sys'
    _usb(root, '1-1', '0483', '5740', 12, 'VESC', {'1.0': {}})
    (dev,) = read_usb_devices(root)
    assert dev.drivers == []
    assert dev.ttys == []


def test_missing_speed_file_reads_as_zero(tmp_path):
    root = tmp_path / 'sys'
    _usb(root, '1-1', '0483', '5740', None, 'VESC', {})
    (dev,) = read_usb_devices(root)
    assert dev.speed_mbps == 0.0


def test_vid_pid_lowercased(tmp_path):
    root = tmp_path / 'sys'
    _usb(root, '1-1', '046D', 'C21F', 12, 'pad', {})
    (dev,) = read_usb_devices(root)
    assert (dev.vid, dev.pid) == ('046d', 'c21f')


def test_no_usb_tree_is_empty_not_a_crash(tmp_path):
    assert read_usb_devices(tmp_path / 'nothing') == []


def _proc(proc_root, pid, argv, fds=None):
    d = proc_root / str(pid)
    d.mkdir(parents=True)
    (d / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv) + b'\0')
    (d / 'fd').mkdir()
    for n, target in (fds or {}).items():
        os.symlink(target, d / 'fd' / str(n))


def test_read_processes_splits_nul_argv_and_ignores_non_pids(tmp_path):
    proc = tmp_path / 'proc'
    _proc(proc, 4242, ['/ws/install/vesc_driver/lib/vesc_driver/vesc_driver_node',
                       '--ros-args', '-r', '__node:=vesc_driver_node'])
    _proc(proc, 77, ['/usr/bin/python3', '/opt/ros/jazzy/lib/joy/joy_node'])
    (proc / 'self').mkdir()
    (proc / 'meminfo').write_text('x')
    procs = {p.pid: p for p in read_processes(proc)}
    assert sorted(procs) == [77, 4242]
    assert procs[4242].argv[0].endswith('vesc_driver_node')
    assert procs[4242].argv[1] == '--ros-args'
    assert procs[4242].runs('vesc_driver_node')
    assert procs[77].runs('joy_node')          # python interpreter argv[1]
    assert not procs[77].runs('joy')           # basename match, not substring


def test_kernel_threads_with_empty_cmdline_are_skipped(tmp_path):
    proc = tmp_path / 'proc'
    d = proc / '2'
    d.mkdir(parents=True)
    (d / 'cmdline').write_bytes(b'')
    assert read_processes(proc) == []


def test_tty_holders_finds_pids_with_the_tty_open(tmp_path):
    proc = tmp_path / 'proc'
    _proc(proc, 10, ['a'], {3: '/dev/ttyACM0', 4: '/dev/null'})
    _proc(proc, 11, ['b'], {3: '/dev/ttyACM1'})
    _proc(proc, 12, ['c'], {5: '/dev/ttyACM0'})
    assert tty_holders(proc, ['/dev/ttyACM0']) == {'/dev/ttyACM0': [10, 12]}


def test_tty_holders_unreadable_fd_dir_is_skipped(tmp_path):
    proc = tmp_path / 'proc'
    _proc(proc, 10, ['a'], {3: '/dev/ttyACM0'})
    (proc / '13').mkdir()                    # no fd dir: exited / no perms
    assert tty_holders(proc, ['/dev/ttyACM0']) == {'/dev/ttyACM0': [10]}


def _gather(tmp_path, *, vesc_link='ttyACM1', lidar_link='ttyACM0',
            input_modes=None, video0=0o660, extra_procs=()):
    sys_root = _car1_tree(tmp_path)
    dev = tmp_path / 'dev'
    (dev / 'sensors').mkdir(parents=True)
    (dev / 'input').mkdir()
    for tty in ('ttyACM0', 'ttyACM1'):
        (dev / tty).write_text('')
    for node, mode in (input_modes or {'js0': 0o644, 'event5': 0o640}).items():
        (dev / 'input' / node).write_text('')
        os.chmod(dev / 'input' / node, mode)
    if video0 is not None:
        (dev / 'video0').write_text('')
        os.chmod(dev / 'video0', video0)
    if vesc_link:
        os.symlink(f'../{vesc_link}', dev / 'sensors/vesc')
    if lidar_link:
        os.symlink(f'../{lidar_link}', dev / 'sensors/hokuyo')
    proc = tmp_path / 'proc'
    _proc(proc, 500, ['vesc_driver_node'], {3: str(dev / 'ttyACM1')})
    _proc(proc, 501, ['urg_node_driver'], {4: str(dev / 'ttyACM0')})
    for pid, argv in extra_procs:
        _proc(proc, pid, argv)
    snap = gather(sys_root=sys_root, dev_root=dev, proc_root=proc,
                  env={'ROS_DISTRO': 'jazzy'})
    return snap, dev


def test_gather_resolves_both_symlinks_and_their_holders(tmp_path):
    snap, dev = _gather(tmp_path)
    assert snap.vesc_symlink == str(dev / 'ttyACM1')
    assert snap.vesc_symlink_rw is True
    assert snap.lidar_symlink == str(dev / 'ttyACM0')
    assert snap.lidar_symlink_rw is True
    assert snap.tty_holders == {str(dev / 'ttyACM1'): [500],
                                str(dev / 'ttyACM0'): [501]}
    assert snap.env == {'ROS_DISTRO': 'jazzy'}


def test_gather_missing_symlinks_are_none(tmp_path):
    snap, _ = _gather(tmp_path, vesc_link=None, lidar_link=None)
    assert snap.vesc_symlink is None and snap.vesc_symlink_rw is False
    assert snap.lidar_symlink is None and snap.lidar_symlink_rw is False


def test_gather_dangling_symlink_reports_target(tmp_path):
    snap, dev = _gather(tmp_path, lidar_link='ttyACM9')
    assert snap.lidar_symlink == str(dev / 'ttyACM9')
    assert snap.lidar_symlink_rw is False


def test_gather_checks_js_and_event_nodes_separately(tmp_path):
    # Car 1, 2026-10-20: js0 0664, event5 0660 root:input, user not in
    # `input`. Here the test owns the files, so owner bits model "can I".
    snap, _ = _gather(tmp_path, input_modes={'js0': 0o400, 'event5': 0o000})
    assert snap.input_readable == {'js0': True, 'event5': False}
    snap, _ = _gather(tmp_path / 'b', input_modes={'js0': 0o444,
                                                   'event5': 0o444})
    assert snap.input_readable == {'js0': True, 'event5': True}


def test_gather_input_node_absent_from_dev_is_unreadable(tmp_path):
    snap, _ = _gather(tmp_path, input_modes={'js0': 0o644})
    assert snap.input_readable == {'js0': True, 'event5': False}


def test_gather_camera_device_states(tmp_path):
    assert _gather(tmp_path / 'a', video0=0o660)[0].camera_device is True
    assert _gather(tmp_path / 'b', video0=0o000)[0].camera_device is False
    assert _gather(tmp_path / 'c', video0=None)[0].camera_device is None
