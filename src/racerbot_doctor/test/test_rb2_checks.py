"""Car 2's Ethernet Hokuyo and RealSense ladder regression tests."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'racerbot_car'))

from racerbot_car.profile import load_profile  # noqa: E402
from racerbot_doctor import car, checks, snapshot  # noqa: E402
from test_checks import healthy_live, first_bad, layer, statuses, without  # noqa: E402
from test_checks import PIDS  # noqa: E402

P, W, F, I, S = (snapshot.Status.PASS, snapshot.Status.WARN, snapshot.Status.FAIL,
                  snapshot.Status.INFO, snapshot.Status.SKIP)


def spec():
    return car.spec_for(load_profile('rb2'))


def healthy_snap():
    return snapshot.Snapshot(
        usb=[snapshot.UsbDevice('1-2.3', '0483', '5740', drivers=['cdc_acm'],
                               ttys=['ttyACM0'], speed_mbps=12),
             snapshot.UsbDevice('1-2.4', '046d', 'c21f', drivers=['xpad'],
                                joysticks=['js0'], events=['event5'], speed_mbps=12),
             snapshot.UsbDevice('2-1.1', '8086', '0b3a',
             drivers=['uvcvideo', 'usbhid'], videos=['video0', 'video1'],
              speed_mbps=5000)],
        processes=[snapshot.Process(pid, [f'/ws/{exe}', '--ros-args'])
                   for exe, pid in PIDS.items() if exe != car.EXE_CAMERA]
                  + [snapshot.Process(108, ['/ws/realsense2_camera_node'])],
        ifaces={'enP8p1s0': snapshot.NetIface('enP8p1s0', True, ['192.168.0.15/24'])},
        vesc_symlink='/dev/ttyACM0', vesc_symlink_rw=True,
        tty_holders={'/dev/ttyACM0': [PIDS[car.EXE_VESC_DRIVER]]},
        input_readable={'js0': True, 'event5': True}, lidar_ping=True,
        env={'ROS_DISTRO': 'jazzy', 'ROS_DOMAIN_ID': '0',
             'AMENT_PREFIX_PATH': '/ws/install/vesc_driver:/ws/install/'
             'ackermann_mux:/ws/install/f1tenth_stack:/opt/ros/jazzy'})


def live():
    sample = healthy_live()
    sample.topics['/camera/camera/color/image_raw'] = snapshot.TopicStats(45, 3.0)
    return sample


def lidar(snap, sample):
    return checks.lidar_ladder(spec().lidar, snap, sample)


def camera(snap, sample):
    return checks.camera_ladder(spec().camera, snap, sample)


def camera_device(snap):
    return next(d for d in snap.usb if (d.vid, d.pid) == ('8086', '0b3a'))


def test_lidar_no_carrier_blames_cable_or_power():
    snap = healthy_snap()
    snap.ifaces['enP8p1s0'].carrier = False
    r = lidar(snap, live())
    assert r.layers[0].status is F and 'cable' in r.layers[0].detail
    assert set(statuses(r)[1:]) == {S}


def test_lidar_iface_missing():
    snap = healthy_snap()
    snap.ifaces = {}
    assert lidar(snap, live()).layers[0].status is F


def test_lidar_wrong_host_ip_suggests_nmcli():
    snap = healthy_snap()
    snap.ifaces['enP8p1s0'].ipv4 = ['169.254.3.3/16']
    bad = first_bad(lidar(snap, live()))
    assert bad.status is F and 'nmcli' in bad.fix and '169.254.3.3' in bad.detail


def test_lidar_no_ping():
    snap = healthy_snap()
    snap.lidar_ping = False
    bad = first_bad(lidar(snap, live()))
    assert bad.status is F and '192.168.0.10' in bad.detail


def test_lidar_ping_unavailable_warns_and_continues():
    snap = healthy_snap()
    snap.lidar_ping = None
    r = lidar(snap, live())
    assert first_bad(r).status is W
    assert F not in statuses(r)


def test_lidar_tcp_refused_with_driver_down():
    snap = without(healthy_snap(), car.EXE_URG)
    snap.lidar_tcp = False
    bad = first_bad(lidar(snap, None))
    assert bad.status is F and '10940' in bad.detail


def test_lidar_tcp_not_probed_while_urg_node_runs_is_not_a_failure():
    r = lidar(healthy_snap(), live())
    tcp = layer(r, 'TCP')
    assert tcp.status is S and 'urg_node' in tcp.detail
    assert r.layers[-1].status is P


def test_lidar_urg_crashed_while_bringup_up():
    snap = without(healthy_snap(), car.EXE_URG)
    snap.lidar_tcp = True
    bad = first_bad(lidar(snap, live()))
    assert bad.status is F and 'urg_node' in bad.detail


def test_lidar_urg_running_but_no_scans_blames_power():
    sample = live()
    sample.topics['/scan'] = snapshot.TopicStats(0, 3.0)
    bad = first_bad(lidar(healthy_snap(), sample))
    assert bad.name == '/scan rate' and bad.status is F
    assert 'power' in bad.detail


def test_camera_missing_is_warn_not_fail():
    snap = healthy_snap()
    snap.usb = []
    r = camera(snap, live())
    assert r.layers[0].status is W and 'not plugged in' in r.layers[0].detail
    assert set(statuses(r)[1:]) == {S}
    assert r.worst() is W


def test_camera_on_usb2_warns_and_continues():
    snap = healthy_snap()
    camera_device(snap).speed_mbps = 480
    r = camera(snap, live())
    assert layer(r, 'USB speed').status is W
    assert 'USB 2' in layer(r, 'USB speed').detail
    assert r.layers[-1].status is P


def test_camera_speed_boundary():
    snap = healthy_snap()
    camera_device(snap).speed_mbps = 5000
    assert layer(camera(snap, None), 'USB speed').status is P
    camera_device(snap).speed_mbps = 4999
    assert layer(camera(snap, None), 'USB speed').status is W


def test_camera_no_uvc_is_warn():
    snap = healthy_snap()
    cam = camera_device(snap)
    cam.drivers, cam.videos = ['usbhid'], []
    bad = first_bad(camera(snap, live()))
    assert bad.status is W and 'uvcvideo' in bad.detail


def test_camera_not_launched_is_info_even_with_bringup_up():
    snap = without(healthy_snap(), 'realsense2_camera_node')
    bad = first_bad(camera(snap, live()))
    assert bad.status is I and 'realsense_camera_launch.py' in bad.fix


def test_camera_launched_but_no_frames_is_warn():
    sample = live()
    sample.topics['/camera/camera/color/image_raw'] = snapshot.TopicStats(0, 3.0)
    r = camera(healthy_snap(), sample)
    assert r.layers[-1].status is W and r.worst() is W


def test_gather_is_asked_to_decide_tcp_probe_itself():
    from test_cli import _run
    _, _, calls = _run(['--car', 'rb2'], healthy_snap(), live())
    assert calls['gather'][0]['probe_lidar_tcp'] == 'auto'
