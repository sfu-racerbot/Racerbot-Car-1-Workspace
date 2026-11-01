#!/usr/bin/env python3
"""Run the real map_course launch on a recorded run's sensors, with the car parked.

    python3 tools/run_analysis/replay_map_course.py data/racerbot_runs/<time> --out /tmp/replays

slam_replay.py tests SLAM alone. This tests what the operator actually runs: the launch, the IMU
stack, SLAM, gap_follow and the lap counter, all unmodified, and says whether the laps would have
closed and whether the map would have passed. It stands in for bringup and nothing else: it
publishes the recorded /scan, /odom, odom -> base_link, /tf_static, /sensors/imu/raw and /joy,
re-stamped to now, at the recorded pace. The recording's own SLAM, IMU stack and commands are not
played.

/joy is what the driver really pressed, so this forges the LB deadman. That is safe only away from
the car, so it refuses while any driver runs, and everything is on a localhost-only ROS domain of
its own. Nothing it starts can reach the car's ackermann_mux.

The BNO is left out (bno:=false) unless the recording has one, because a parked BNO beside a
recorded VESC gyro would look like two gyros disagreeing. No run has one yet, so this exercises the
VESC gyro path. Launch arguments can be added after --.

A recording where the car was carried between LB holds cannot pass from its start: SLAM cannot follow
what the wheels and gyro did not see. --from-seconds starts after that.

Exit: 0 the laps closed and the map passed, 1 anything less, 2 refused or could not run.
"""
import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROS_DOMAIN = '78'  # not the car's 0, and not 79, which the test suites use
STILL_SPEED = 0.02  # m/s: what /odom reads with the car parked
MIN_STILL_S = 1.0
PREROLL_S = 12.0  # what the operator waits: the stack starts, and the startup gyro bias settles
TOPICS = ('/scan', '/odom', '/tf', '/tf_static', '/sensors/imu/raw', '/joy')
DRIVER_MARKERS = ('vesc_driver_node', 'urg_node_driver', 'gym_bridge_node', 'async_slam_toolbox_node')
LAP_CLOSED = re.compile(r'Closed mapping lap (\d+)/(\d+) detected \(([\d.]+)m')
RECORDING = re.compile(r'Recording run to: (\S+)')


class Refused(Exception):
    """Not safe, or not possible, to run."""


# --- the parts with no ROS in them --------------------------------------------------------------

def still_window(odom_rows, min_seconds=MIN_STILL_S, still_speed=STILL_SPEED):
    """(start, end) of the first stretch with the car parked, or None.

    odom_rows are (time [s], speed [m/s]) in time order. The stack's startup needs a parked car, and
    a recording may start with it moving, so playback starts from the first time it was parked.
    """
    start = None
    last = None
    for t, speed in odom_rows:
        if abs(speed) > still_speed or speed != speed:
            if start is not None and last - start >= min_seconds:
                return start, last
            start = None
        elif start is None:
            start = t
        last = t
    if start is not None and last - start >= min_seconds:
        return start, last
    return None


def wheel_transforms(transforms):
    """Only vesc_to_odom's odom -> base_link.

    The recording's map -> odom is its own SLAM's answer, and odom_imu -> odom its own IMU stack's.
    Playing either would give the frames two parents.
    """
    return [t for t in transforms
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link']


def laps_from_log(text):
    """({lap number: metres}, laps wanted) from what map_course printed. Empty if none closed."""
    laps = {}
    wanted = None
    for match in LAP_CLOSED.finditer(text):
        laps[int(match.group(1))] = float(match.group(3))
        wanted = int(match.group(2))
    return laps, wanted


def recording_from_log(text):
    """The folder the launch recorded into, or None."""
    match = RECORDING.search(text)
    return Path(match.group(1)) if match else None


def verdict_of(rebuild_json):
    """rebuild.json, as (go, why, whole laps). A missing file is a failure, not a pass."""
    if not rebuild_json.is_file():
        return False, 'the map check wrote no rebuild.json', 0
    data = json.loads(rebuild_json.read_text())
    verdict = data['verdict']
    # GO needs a good map and a line to race on it, as map_course's own summary does.
    return (bool(verdict['ok']) and bool(data['raceline_ok']), str(data.get('message') or verdict['reason']),
            int(data['laps']))


def refusal(cmdlines, markers=DRIVER_MARKERS):
    """Why this must not run, given every process's command line, or None."""
    found = sorted({m for m in markers for line in cmdlines if m in line})
    if found:
        return ('refusing: ' + ', '.join(found) + ' running. This publishes a held LB, '
                'so it never runs beside drivers or another SLAM.')
    return None


def process_cmdlines(proc_root=Path('/proc')):
    lines = []
    for entry in Path(proc_root).iterdir():
        if entry.name.isdigit():
            try:
                lines.append((entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode('utf-8', 'replace'))
            except OSError:
                continue
    return lines


# --- the replay: needs ROS ------------------------------------------------------------------------

def _reader(bag, topics):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def _shift(stamp, offset_ns):
    total = stamp.sec * 1_000_000_000 + stamp.nanosec + offset_ns
    stamp.sec, stamp.nanosec = divmod(total, 1_000_000_000)


class _Feeder:
    """Publishes recorded messages at their recorded pace, stamped as if they had just happened."""

    def __init__(self, bag):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rosidl_runtime_py.utilities import get_message
        self.rclpy = rclpy
        self.bag = bag
        reader = _reader(bag, TOPICS)
        self.types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
                      if t.name in TOPICS}
        missing = [t for t in ('/scan', '/odom', '/tf', '/tf_static', '/joy') if t not in self.types]
        if missing:
            raise Refused(f'{bag} has no {", ".join(missing)}')
        if '/sensors/imu/raw' not in self.types:
            raise Refused(f'{bag} has no /sensors/imu/raw, so there is no gyro to map on')
        rclpy.init()
        self.node = rclpy.create_node('recorded_sensors')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs = {name: self.node.create_publisher(self.types[name], name,
                                                      latched if name == '/tf_static' else 50)
                     for name in self.types}

    def read(self, start_s=None, end_s=None, topics=TOPICS):
        """(time [s], topic, message), in recorded order."""
        from rclpy.serialization import deserialize_message
        reader = _reader(self.bag, topics)
        if start_s is not None:
            reader.seek(int(start_s * 1e9))
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            t = t_ns * 1e-9
            if end_s is not None and t > end_s:
                return
            yield t, topic, deserialize_message(data, self.types[topic])

    def find_start(self, from_seconds=0.0):
        rows = [(t, m.twist.twist.linear.x) for t, _, m in self.read(topics=('/odom',))]
        rows = [row for row in rows if rows and row[0] >= rows[0][0] + from_seconds]
        window = still_window(rows)
        if window is None:
            raise Refused(f'{self.bag}: the car is never parked for {MIN_STILL_S} s, '
                          'and the IMU stack needs a parked start')
        return window

    def publish_statics(self):
        for _, topic, message in self.read(topics=('/tf_static',)):
            now = self.node.get_clock().now().to_msg()
            for transform in message.transforms:
                transform.header.stamp = now
            self.pubs[topic].publish(message)
            return
        raise Refused(f'{self.bag} has an empty /tf_static')

    def _send(self, topic, message, offset_ns):
        if topic == '/tf_static':
            return
        if topic == '/tf':
            message.transforms = wheel_transforms(message.transforms)
            if not message.transforms:
                return
            for transform in message.transforms:
                _shift(transform.header.stamp, offset_ns)
        else:
            _shift(message.header.stamp, offset_ns)
        self.pubs[topic].publish(message)

    def play(self, messages, should_stop):
        """Publish at the recorded pace. True if should_stop() ended it."""
        offset_ns = None
        for t, topic, message in messages:
            if should_stop():
                return True
            if offset_ns is None:
                offset_ns = time.time_ns() - int(t * 1e9)
            wait = t + offset_ns * 1e-9 - time.time()
            if wait > 0.0:
                time.sleep(wait)
            self._send(topic, message, offset_ns)
        return False

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def _environment(home, workspace):
    env = dict(os.environ, HOME=str(home), ROS_HOME=str(home / 'ros'), RACERBOT_WS=str(workspace),
               PYTHONUSERBASE=str(Path.home() / '.local'), ROS_DOMAIN_ID=ROS_DOMAIN,
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
    return env


def replay(run_dir, out_root, launch_args, workspace, max_seconds, from_seconds=0.0):
    """One replay. Returns the result dict that is also saved as result.json."""
    run_dir = Path(run_dir).resolve()
    reason = refusal(process_cmdlines())
    if reason:
        raise Refused(reason)
    out = Path(tempfile.mkdtemp(prefix=f'{run_dir.name}-', dir=Path(out_root).expanduser()))
    home = out / 'home'
    (home / 'racerbot-ws' / 'data').mkdir(parents=True)
    (home / 'imu-calibration').symlink_to(Path.home() / 'imu-calibration')
    auto = out / 'auto'
    auto.mkdir()
    env = _environment(home, workspace)
    # rclpy reads these once, at init, for the feeder in this process.
    os.environ.update(ROS_DOMAIN_ID=ROS_DOMAIN, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')

    feeder = _Feeder(run_dir / 'bag')
    still_start, still_end = feeder.find_start(from_seconds)
    feeder.publish_statics()
    argv = ['ros2', 'launch', 'racerbot_launch', 'map_course_launch.py', 'bno:=false',
            'imu_hardware:=true', f'output_directory:={auto}', *launch_args]
    log_path = out / 'launch.log'
    log = log_path.open('wb')
    launch = subprocess.Popen(argv, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    began = time.monotonic()
    ended = launch.poll
    try:
        # Read afresh each time round: publishing re-stamps a message in place, so a kept list
        # would move a day further into the future on every pass.
        while time.monotonic() - began < PREROLL_S and ended() is None:
            feeder.play(feeder.read(still_start, still_end),
                        lambda: ended() is not None or time.monotonic() - began >= PREROLL_S)

        def stop():
            return ended() is not None or time.monotonic() - began >= max_seconds

        stopped = feeder.play(feeder.read(still_end), stop)
        finished_alone = ended() is not None
        how = 'finished by itself' if finished_alone else (
            'time limit' if stopped else 'recording ran out, stopped with Ctrl+C')
    finally:
        feeder.close()
        for sig, wait in ((signal.SIGINT, 45), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            if launch.poll() is not None:
                break
            try:
                os.killpg(launch.pid, sig)
                launch.wait(timeout=wait)
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                break
        log.close()

    text = log_path.read_text(errors='replace')
    laps, wanted = laps_from_log(text)
    recording = recording_from_log(text)
    rebuilt = None
    go, why, whole = False, 'no recording', 0
    if recording is not None and recording.is_dir():
        rebuild = subprocess.run(
            [sys.executable, str(Path(workspace) / 'tools/run_analysis/rebuild_run.py'), str(recording),
             '--out-root', str(auto)], env=env, capture_output=True, text=True, timeout=600)
        (out / 'rebuild.log').write_text(rebuild.stdout + rebuild.stderr)
        rebuilt = auto / f'{recording.name}-rebuilt'
        go, why, whole = verdict_of(rebuilt / 'rebuild.json')
    result = {
        'run': run_dir.name, 'args': launch_args, 'how': how, 'seconds': round(time.monotonic() - began, 1),
        'laps_closed': len(laps), 'laps_wanted': wanted, 'lap_metres': [laps[k] for k in sorted(laps)],
        'map_go': go, 'map_reason': why, 'whole_laps_in_map': whole, 'out': str(out),
        'rebuilt': str(rebuilt) if rebuilt else None,
    }
    (out / 'result.json').write_text(json.dumps(result, indent=2))
    # The bag is a copy of what went in; only the small results are worth keeping.
    shutil.rmtree(home / 'ros', ignore_errors=True)
    return result


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--out', type=Path, required=True, help='parent of the output folder')
    parser.add_argument('--max-seconds', type=float, default=420.0)
    parser.add_argument('--from-seconds', type=float, default=0.0,
                        help='start from the first parked stretch after this far into the recording. Use it to '
                             'skip a car that was carried: SLAM cannot follow what the wheels and gyro did not see')
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2],
                        help='where tools/ is; the launch comes from whichever install is sourced')
    parser.add_argument('launch_args', nargs='*', help='name:=value, after --')
    # Intermixed: plain parsing gives launch_args its turn at run_dir, before any name:=value after an option.
    return parser.parse_intermixed_args([a for a in (sys.argv[1:] if argv is None else argv) if a != '--'])


def main(argv=None):
    args = parse_arguments(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    try:
        result = replay(args.run_dir, args.out, args.launch_args, args.workspace, args.max_seconds, args.from_seconds)
    except Refused as exc:
        print(exc, file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2))
    closed = result['laps_wanted'] and result['laps_closed'] >= result['laps_wanted']
    return 0 if closed and result['map_go'] else 1


if __name__ == '__main__':
    sys.exit(main())
