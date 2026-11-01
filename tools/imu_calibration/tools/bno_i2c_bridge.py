#!/usr/bin/env python3
"""BNO085 on Jetson I2C -> the same `$B,...` lines firmware/bno085_stream prints, so racer_imu runs unchanged.

Speaks SHTP / SH-2 directly over /dev/i2c-N (no Blinka, no MCU). Enables the same three reports as the
firmware (accelerometer, calibrated gyro, game rotation vector) and emits one line per accel report,
carrying the latest gyro + quaternion, exactly like the sketch.

    python3 bno_i2c_bridge.py --print                 # human-readable readings
    python3 bno_i2c_bridge.py --pty /tmp/bno085       # symlink a virtual serial port there for racer_imu
"""
import argparse
import fcntl
import math
import os
import pty
import struct
import sys
import termios
import time
import tty

I2C_SLAVE = 0x0703
CH_EXEC, CH_CONTROL, CH_REPORTS = 1, 2, 3
ACCEL, GYRO_CAL, GAME_RV = 0x01, 0x02, 0x08
# SH-2 reference manual: report id -> total length in bytes, and Q point of its int16 fields
REPORT_LEN = {0x01: 10, 0x02: 10, 0x03: 10, 0x04: 10, 0x05: 14, 0x06: 10, 0x07: 16, 0x08: 12, 0x09: 14,
              0x0F: 16, 0x14: 16, 0x15: 10, 0x2A: 14}
Q_ACCEL, Q_GYRO, Q_QUAT = 8, 9, 14


class Shtp:
    def __init__(self, bus: int, addr: int):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)
        self.seq = [0] * 6

    def write(self, channel: int, payload: bytes):
        n = len(payload) + 4
        pkt = struct.pack("<HBB", n, channel, self.seq[channel]) + payload
        self.seq[channel] = (self.seq[channel] + 1) & 0xFF
        for _ in range(5):
            try:
                os.write(self.fd, pkt)
                return
            except OSError:
                time.sleep(0.002)
        raise OSError("BNO085 did not accept write")

    def read(self):
        """Returns (channel, payload) or None if nothing is pending."""
        try:
            hdr = os.read(self.fd, 4)
        except OSError:          # NACK while the hub is busy
            return None
        n = struct.unpack_from("<H", hdr)[0] & 0x7FFF
        if n <= 4 or n == 0x7FFF:
            return None
        try:
            data = os.read(self.fd, n)   # the hub re-sends the header in this read
        except OSError:
            return None
        return data[2], data[4:n]

    def drain(self, seconds: float):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            if self.read() is None:
                time.sleep(0.005)

    def soft_reset(self):
        self.write(CH_EXEC, b"\x01")
        time.sleep(0.3)
        self.drain(0.5)

    def enable(self, report_id: int, interval_us: int):
        # Set Feature Command (0xFD): id, flags, change sensitivity, report interval, batch interval, config
        self.write(CH_CONTROL, struct.pack("<BBBHIII", 0xFD, report_id, 0, 0, interval_us, 0, 0))

    def command(self, cmd: int, params, timeout: float = 1.0):
        """Command Request (0xF2) on the control channel; returns the matching response's R0..R10 or None."""
        self.cmd_seq = (getattr(self, "cmd_seq", -1) + 1) & 0xFF
        p = (list(params) + [0] * 9)[:9]
        self.write(CH_CONTROL, bytes([0xF2, self.cmd_seq, cmd, *p]))
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            r = self.read()
            if r is None:
                time.sleep(0.002)
                continue
            ch, pl = r
            if ch == CH_CONTROL and len(pl) >= 16 and pl[0] == 0xF1 and pl[2] == cmd:
                return list(pl[5:16])
        return None

    def me_cal(self):
        """Dynamic (ME) calibration enables: dict accel/gyro/mag/planar/on_table, or None."""
        r = self.command(0x07, [0, 0, 0, 0x01])          # subcommand 0x01 = get
        if r is None or r[0] != 0:
            return None
        return dict(zip(("accel", "gyro", "mag", "planar", "on_table"), r[1:6]))

    def set_me_cal(self, accel, gyro, mag, planar=0, on_table=0) -> bool:
        r = self.command(0x07, [accel, gyro, mag, 0x00, planar, on_table])   # subcommand 0x00 = configure
        return r is not None and r[0] == 0


def checksum(body: str) -> int:
    c = 0
    for ch in body.encode():
        c ^= ch
    return c


def fmt(v, dec):
    return "nan" if v != v else f"{v:.{dec}f}"


def run(args):
    s = Shtp(args.bus, args.addr)
    s.soft_reset()

    def freeze_accel_cal():
        """Stop the hub re-tuning its own accel calibration while racer_imu calibrates it (volatile: a
        reset restores the default). Gyro and mag dynamic calibration are left as they were."""
        before = s.me_cal()
        if before is None:
            sys.exit("# could not read the BNO085's dynamic calibration settings")
        ok = s.set_me_cal(0, before["gyro"], before["mag"], before["planar"], before["on_table"])
        after = s.me_cal()
        if not ok or after is None or after["accel"] != 0:
            sys.exit(f"# could not turn off accel dynamic calibration (before {before}, after {after})")
        print(f"# dynamic calibration before {before}\n# dynamic calibration now    {after}", flush=True)

    if args.no_dyncal:
        freeze_accel_cal()
    interval = int(1e6 / args.rate)
    for rid in (ACCEL, GYRO_CAL, GAME_RV):
        s.enable(rid, interval)
    out_fd = None
    if args.pty:
        master, slave = pty.openpty()
        tty.setraw(slave)
        name = os.ttyname(slave)
        if os.path.islink(args.pty) or os.path.exists(args.pty):
            os.remove(args.pty)
        os.symlink(name, args.pty)
        out_fd = master
        print(f"# virtual serial port {args.pty} -> {name}", flush=True)
    gyr = [math.nan] * 3
    quat = [math.nan] * 4
    n_lines, t_rate, last_print = 0, time.monotonic(), 0.0
    if out_fd is not None:
        os.write(out_fd, b"# bno085_stream ready (i2c bridge)\r\n")
    while True:
        r = s.read()
        if r is None:
            time.sleep(0.0005)
            continue
        ch, p = r
        if ch == CH_EXEC and p[:1] == b"\x01":          # hub reset itself: re-enable, like the sketch
            print("# sensor was reset, re-enabling reports", flush=True)
            if args.no_dyncal:
                freeze_accel_cal()
            for rid in (ACCEL, GYRO_CAL, GAME_RV):
                s.enable(rid, interval)
            if out_fd is not None:
                os.write(out_fd, b"# sensor was reset, re-enabling reports\r\n")
            continue
        if ch != CH_REPORTS or not p or p[0] != 0xFB:
            continue
        t_read = time.monotonic()
        base_delay = struct.unpack_from("<I", p, 1)[0] * 100e-6     # 100 us units
        i = 5
        while i < len(p):
            rid = p[i]
            if rid == 0xFA:                                           # timestamp rebase
                i += 5
                continue
            L = REPORT_LEN.get(rid)
            if L is None or i + L > len(p):
                break
            status = p[i + 2] & 0x03
            delay = (((p[i + 2] >> 2) << 8) | p[i + 3]) * 100e-6
            if rid == GYRO_CAL:
                gyr = [v / (1 << Q_GYRO) for v in struct.unpack_from("<hhh", p, i + 4)]
            elif rid == GAME_RV:
                qi, qj, qk, qr = (v / (1 << Q_QUAT) for v in struct.unpack_from("<hhhh", p, i + 4))
                quat = [qr, qi, qj, qk]
            elif rid == ACCEL:
                acc = [v / (1 << Q_ACCEL) for v in struct.unpack_from("<hhh", p, i + 4)]
                t_us = int((t_read - base_delay + delay) * 1e6) & 0xFFFFFFFF
                body = "B,%d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d" % (
                    t_us, *(fmt(v, 4) for v in acc), *(fmt(v, 5) for v in gyr), *(fmt(v, 5) for v in quat), status)
                line = "$%s*%02X\r\n" % (body, checksum(body))
                n_lines += 1
                if out_fd is not None:
                    try:
                        os.write(out_fd, line.encode())
                    except BlockingIOError:
                        pass
                if args.print and t_read - last_print > 0.25:
                    last_print = t_read
                    rate = n_lines / max(t_read - t_rate, 1e-6)
                    an = math.sqrt(sum(a * a for a in acc))
                    print(f"{rate:5.0f} Hz  acc {acc[0]:+7.3f} {acc[1]:+7.3f} {acc[2]:+7.3f} |a|={an:6.3f} m/s^2"
                          f"  gyro {fmt(gyr[0],4)} {fmt(gyr[1],4)} {fmt(gyr[2],4)} rad/s"
                          f"  quat {' '.join(fmt(q,3) for q in quat)}  acc_status {status}", flush=True)
            i += L
        if args.seconds and time.monotonic() - t_rate > args.seconds:
            print(f"# {n_lines} accel lines in {args.seconds:.0f} s = {n_lines / args.seconds:.0f} Hz")
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--bus", type=int, default=7, help="7 = Jetson header pins 3/5")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x4A)
    ap.add_argument("--rate", type=float, default=200.0, help="report rate [Hz], same as the firmware")
    ap.add_argument("--print", action="store_true", help="print readings 4x per second")
    ap.add_argument("--pty", default=None, help="create a virtual serial port symlinked here")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = run forever)")
    ap.add_argument("--no-dyncal", action="store_true",
                    help="turn off the BNO085's own accel dynamic calibration (use while calibrating)")
    a = ap.parse_args()
    if not a.print and not a.pty:
        a.print = True
    try:
        run(a)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
