"""Hardware specifications for both cars, and shared health thresholds.

Car 1's constants remain the USB-serial Hokuyo and UVC webcam defaults.
Car 2's Ethernet Hokuyo and RealSense defaults live in RB2_LIDAR/RB2_CAMERA.
spec_for takes addresses from the profile and selects by hardware kind.

Every number cites where it comes from, so a hardware change has an
obvious place to update. Keep in step with the files named.
"""

from dataclasses import dataclass, replace

from racerbot_car.profile import (
    CameraRealSense, CameraUVC, LidarEthernet, LidarSerial)

# --- VESC: /etc/udev/rules.d/99-vesc.rules, f1tenth_stack config/vesc.yaml
VESC_USB = ('0483', '5740')
VESC_SYMLINK = 'sensors/vesc'               # under /dev
VESC_KERNEL_DRIVER = 'cdc_acm'
VESC_CORE_MIN_HZ = 25.0                     # driver polls at 50 Hz, vesc_driver.cpp:107

# --- Battery: 3S LiPo (team, 2026-10-01). Per-cell: 3.4 V fail, 3.6 V warn,
# 4.25 V is above a full 3S charge.
BATTERY_CELLS = 3
BATTERY_ABSENT_V = 5.0                      # below this: VESC alive on USB only
BATTERY_FAIL_V = 3.4 * BATTERY_CELLS        # 10.2
BATTERY_WARN_V = 3.6 * BATTERY_CELLS        # 10.8
BATTERY_HIGH_V = 4.25 * BATTERY_CELLS       # 12.75

# --- LiDAR: Hokuyo UST-30LX over USB, /etc/udev/rules.d/99-hokuyo.rules
# (15d1:0000, SYMLINK sensors/hokuyo) and f1tenth_stack config/sensors.yaml
# (serial_port: /dev/sensors/hokuyo), docs/hardware-reference.md.
LIDAR_USB = ('15d1', '0000')
LIDAR_SYMLINK = 'sensors/hokuyo'            # under /dev
LIDAR_KERNEL_DRIVER = 'cdc_acm'
SCAN_MIN_HZ = 30.0                          # UST-30LX scans at 40 Hz (25 ms)
SCAN_MIN_FINITE_FRACTION = 0.5

# --- Joystick: Logitech F710, joy_teleop.yaml. XInput mode is c21f; the
# D (DirectInput) position on the back switch enumerates as c219, and this
# Jetson then creates no /dev/input/js* at all (docs/troubleshooting.md).
JOY_VID = '046d'
JOY_XINPUT_PID = 'c21f'
JOY_DIRECTINPUT_PID = 'c219'
JOY_KERNEL_DRIVER = 'xpad'
JOY_MIN_HZ = 5.0                            # autorepeat_rate: 20.0
JOY_LB_BUTTON = 4                           # human_control deadman_buttons: [4]

# --- Camera: UVC USB webcam, any make -- matched by its uvcvideo driver, not
# a vid:pid. Device path: usb_cam_stream_node `device` in racerbot_launch
# config/web_dashboard_rb1.yaml. Started by webcam_launch.py; it serves MJPEG
# on port 9090 and publishes no ROS topic, so there is no rate layer.
# Optional: never fails the run (team, 2026-10-01).
CAMERA_KERNEL_DRIVER = 'uvcvideo'
CAMERA_DEVICE = 'video0'                    # under /dev

# --- Bringup: f1tenth_stack bringup_launch.py
ODOM_MIN_HZ = 25.0                          # vesc_to_odom publishes per /sensors/core
# Drive path /joy -> joy_teleop -> /teleop -> ackermann_mux -> /ackermann_cmd
# -> ackermann_to_vesc -> /commands/*: each hop republishes once per input
# message (joy_teleop.py joy_callback, ackermann_mux input callback,
# ackermann_to_vesc.cpp), so every hop must keep up with /joy.
DRIVE_MIN_HZ = JOY_MIN_HZ
LASER_TF = (0.26, 0.0, 0.11)                # base_link->laser, docs/hardware-reference.md
LASER_TF_TOL_M = 0.001

# Executable basenames, as they appear in /proc/<pid>/cmdline.
EXE_VESC_DRIVER = 'vesc_driver_node'
EXE_URG = 'urg_node_driver'
EXE_JOY = 'joy_node'
EXE_MUX = 'ackermann_mux'
EXE_ACK_TO_VESC = 'ackermann_to_vesc_node'
EXE_VESC_TO_ODOM = 'vesc_to_odom_node'
EXE_JOY_TELEOP = 'joy_teleop'                # teleop_launch.py
EXE_CAMERA = 'camera_stream_node'           # usb_cam_stream, webcam_launch.py
BRINGUP_EXES = (EXE_VESC_DRIVER, EXE_URG, EXE_JOY, EXE_MUX,
                EXE_ACK_TO_VESC, EXE_VESC_TO_ODOM)
# racerbot_sim forges /joy and publishes a second /scan and /odom.
SIM_EXES = ('gym_bridge_node', 'sim_joy_node')

TEST_DOMAIN_ID = '79'                       # CLAUDE.md: tests only


@dataclass(frozen=True)
class SerialLidarSpec:
    usb: tuple[str, str]
    symlink: str
    kernel_driver: str


@dataclass(frozen=True)
class EthernetLidarSpec:
    iface: str
    host_ip: str
    ip: str
    port: int


@dataclass(frozen=True)
class UvcCameraSpec:
    device: str
    kernel_driver: str
    exe: str


@dataclass(frozen=True)
class RealSenseSpec:
    usb: tuple[str, str]
    kernel_driver: str
    min_speed_mbps: float
    topic: str
    min_hz: float
    exe: str


@dataclass(frozen=True)
class DoctorSpec:
    lidar: SerialLidarSpec | EthernetLidarSpec
    camera: UvcCameraSpec | RealSenseSpec | None


RB1_LIDAR = SerialLidarSpec(LIDAR_USB, LIDAR_SYMLINK, LIDAR_KERNEL_DRIVER)
RB1_CAMERA = UvcCameraSpec(CAMERA_DEVICE, CAMERA_KERNEL_DRIVER, EXE_CAMERA)

# Car 2's car.py: Hokuyo UST-10LX over Ethernet, f1tenth_stack config/sensors.yaml
# (ip_address/ip_port) and the `hokuyo` NetworkManager profile on enP8p1s0.
RB2_LIDAR = EthernetLidarSpec(
    iface='enP8p1s0',          # `hokuyo` NetworkManager profile on enP8p1s0
    host_ip='192.168.0.15/24',  # `hokuyo` NetworkManager profile on enP8p1s0
    ip='192.168.0.10',        # f1tenth_stack config/sensors.yaml (ip_address)
    port=10940,              # f1tenth_stack config/sensors.yaml (ip_port)
)
# Car 2's car.py: RealSense D435i, racerbot_launch realsense_camera_launch.py
# (424x240 @ 15 fps). Optional: never fails the run (team, 2026-10-01).
RB2_CAMERA = RealSenseSpec(
    usb=('8086', '0b3a'),     # RealSense D435i, realsense_camera_launch.py
    kernel_driver='uvcvideo',  # RealSense D435i, realsense_camera_launch.py
    min_speed_mbps=5000.0,   # USB 3; 480 = USB 2
    topic='/camera/camera/color/image_raw',  # realsense_camera_launch.py
    min_hz=8.0,              # realsense_camera_launch.py (424x240 @ 15 fps)
    exe='realsense2_camera_node',  # Executable basename in /proc/<pid>/cmdline
)

RB1 = DoctorSpec(RB1_LIDAR, RB1_CAMERA)


def spec_for(profile) -> DoctorSpec:
    """Use profile hardware kinds and values, with each driver's fixed defaults."""
    if isinstance(profile.lidar, LidarSerial):
        lidar = replace(RB1_LIDAR, usb=tuple(profile.lidar.usb_id.split(':')),
                        symlink=profile.lidar.port.removeprefix('/dev/'))
    elif isinstance(profile.lidar, LidarEthernet):
        lidar = replace(RB2_LIDAR, iface=profile.lidar.iface,
                        host_ip=profile.lidar.host_ip, ip=profile.lidar.ip,
                        port=profile.lidar.port)
    else:
        raise ValueError(f'unsupported lidar: {profile.lidar!r}')
    if isinstance(profile.camera, CameraUVC):
        camera = replace(RB1_CAMERA, device=profile.camera.device.removeprefix('/dev/'))
    elif isinstance(profile.camera, CameraRealSense):
        camera = replace(RB2_CAMERA, usb=tuple(profile.camera.usb_id.split(':')),
                         topic=profile.camera.topic)
    elif profile.camera is None:
        camera = None
    else:
        raise ValueError(f'unsupported camera: {profile.camera!r}')
    return DoctorSpec(lidar, camera)
