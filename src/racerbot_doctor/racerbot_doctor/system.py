"""Read the OS into a Snapshot. The only module that touches the machine.

Read-only by design, and stricter than that: it never open()s a tty, a
video node or a joystick node (opening a tty can toggle DTR; opening a
video node can take the camera from its driver) -- access is checked with
os.access only. That covers car 1's USB Hokuyo too: it is a tty like the
VESC, so the doctor never opens it, whether or not urg_node is running.

Every root path is a parameter, so tests run against a fake tree in
tmp_path.
"""
import os

from . import car
from .snapshot import Process, Snapshot, UsbDevice


def _read(path, default=''):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def _listdir(path):
    try:
        return sorted(os.listdir(path))
    except OSError:
        return []


def read_usb_devices(sys_root):
    base = os.path.join(sys_root, 'bus/usb/devices')
    devices = []
    names = _listdir(base)
    for name in names:
        if ':' in name:
            continue                    # an interface, not a device
        dev_dir = os.path.join(base, name)
        vid = _read(os.path.join(dev_dir, 'idVendor')).lower()
        if not vid:
            continue
        try:
            speed = float(_read(os.path.join(dev_dir, 'speed'), '0'))
        except ValueError:
            speed = 0.0
        dev = UsbDevice(
            sysname=name, vid=vid,
            pid=_read(os.path.join(dev_dir, 'idProduct')).lower(),
            product=_read(os.path.join(dev_dir, 'product')),
            speed_mbps=speed)
        for intf in names:
            if not intf.startswith(name + ':'):
                continue
            idir = os.path.join(base, intf)
            link = os.path.join(idir, 'driver')
            if os.path.islink(link):
                drv = os.path.basename(os.readlink(link))
                if drv not in dev.drivers:
                    dev.drivers.append(drv)
            dev.ttys += _listdir(os.path.join(idir, 'tty'))
            dev.videos += _listdir(os.path.join(idir, 'video4linux'))
            for inp in _listdir(os.path.join(idir, 'input')):
                nodes = _listdir(os.path.join(idir, 'input', inp))
                dev.joysticks += [n for n in nodes if n.startswith('js')]
                dev.events += [n for n in nodes if n.startswith('event')]
        devices.append(dev)
    return devices


def read_processes(proc_root):
    procs = []
    for name in _listdir(proc_root):
        if not name.isdigit():
            continue
        try:
            with open(os.path.join(proc_root, name, 'cmdline'), 'rb') as f:
                raw = f.read()
        except OSError:
            continue                    # exited between listdir and open
        argv = [a.decode(errors='replace') for a in raw.split(b'\0') if a]
        if argv:
            procs.append(Process(int(name), argv))
    return sorted(procs, key=lambda p: p.pid)


def tty_holders(proc_root, ttys):
    """{tty: [pid, ...]} for the given device paths. Other users' processes
    are invisible without root, so this can under-report, never over-report."""
    wanted = set(ttys)
    found = {}
    for name in _listdir(proc_root):
        if not name.isdigit():
            continue
        fd_dir = os.path.join(proc_root, name, 'fd')
        for fd in _listdir(fd_dir):
            try:
                target = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            if target in wanted:
                pids = found.setdefault(target, [])
                if int(name) not in pids:
                    pids.append(int(name))
    return {k: sorted(v) for k, v in found.items()}


def _symlink(dev_root, rel):
    """(resolved target, readable+writable) for /dev/<rel>, (None, False)
    if the link is absent. A dangling link still reports its target."""
    link = os.path.join(dev_root, rel)
    if not os.path.lexists(link):
        return None, False
    return os.path.realpath(link), os.access(link, os.R_OK | os.W_OK)


def gather(*, sys_root='/sys', dev_root='/dev', proc_root='/proc', env=None):
    snap = Snapshot(env=dict(os.environ if env is None else env))
    snap.usb = read_usb_devices(sys_root)
    snap.processes = read_processes(proc_root)

    snap.vesc_symlink, snap.vesc_symlink_rw = _symlink(dev_root, car.VESC_SYMLINK)
    snap.lidar_symlink, snap.lidar_symlink_rw = _symlink(dev_root, car.LIDAR_SYMLINK)

    ttys = [os.path.join(dev_root, t) for d in snap.usb for t in d.ttys]
    ttys += [t for t in (snap.vesc_symlink, snap.lidar_symlink) if t]
    snap.tty_holders = tty_holders(proc_root, ttys)

    for d in snap.usb:
        for node in d.joysticks + d.events:
            snap.input_readable[node] = os.access(
                os.path.join(dev_root, 'input', node), os.R_OK)

    cam = os.path.join(dev_root, car.CAMERA_DEVICE)
    if os.path.exists(cam):
        snap.camera_device = os.access(cam, os.R_OK | os.W_OK)
    return snap
