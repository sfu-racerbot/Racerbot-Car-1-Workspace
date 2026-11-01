"""`ros2 run racerbot_doctor doctor` -- read-only self-check of the car.

Exit code: 0 = no FAIL, 1 = at least one FAIL, 2 = selection error or crash.
See src/racerbot_doctor/README.md.
"""
import argparse
import sys
import traceback

from racerbot_car import identity, paths
from racerbot_car.profile import ProfileError, load_profile

from . import car
from .checks import run_all
from .report import render


def _default_sample(window_s, *, spec):
    from .live import sample      # imports rclpy; may raise ImportError
    return sample(window_s, spec=spec)


def main(argv=None, *, gather_fn=None, sample_fn=None, out=None):
    out = out or sys.stdout
    parser = argparse.ArgumentParser(
        prog='doctor', description='Read-only self-check: is each device '
        'plugged in, is its kernel driver bound, is its ROS driver running, '
        'and is it producing sane data. Never publishes or moves the car.')
    parser.add_argument('--window', type=float, default=3.0,
                        help='seconds to sample each topic (default 3)')
    parser.add_argument('--no-live', action='store_true',
                        help='hardware/OS checks only; do not join the ROS graph')
    parser.add_argument('--no-color', action='store_true')
    parser.add_argument('--car', help='car profile to inspect (rb1 or rb2); overrides identity')
    args = parser.parse_args(argv)
    color = not args.no_color and out.isatty()

    try:
        known = identity.known_car_ids(paths.cars_dir())
        car_id = args.car if args.car is not None else identity.current_car_id()
        if car_id not in known:
            raise identity.UnknownCarError(f'unknown car {car_id!r}; known ids: {sorted(known)}')
        spec = car.spec_for(load_profile(car_id))
    except (identity.UnknownCarError, identity.CarMismatchError, ProfileError) as exc:
        out.write(f'{exc}\nSelect a known car with RACERBOT_CAR or --car rb1|rb2.\n')
        return 2

    try:
        if gather_fn is None:
            from .system import gather as gather_fn
        if sample_fn is None:
            sample_fn = _default_sample
        gather_options = {'spec': spec}
        if isinstance(spec.lidar, car.EthernetLidarSpec):
            gather_options['probe_lidar_tcp'] = 'auto'
        snap = gather_fn(**gather_options)

        notes = []
        live = None
        exes = car.BRINGUP_EXES + (car.EXE_JOY_TELEOP,)
        if isinstance(spec.camera, car.RealSenseSpec):
            exes += (spec.camera.exe,)
        drivers_up = any(p.runs(exe) for p in snap.processes for exe in exes)
        if args.no_live:
            notes.append('live stage off (--no-live): topic layers skipped')
        elif drivers_up:
            try:
                live = sample_fn(args.window, spec=spec)
            except ImportError as e:
                notes.append(f'live stage skipped, ROS is not importable ({e}): '
                             'source /opt/ros/jazzy/setup.bash && source '
                             '~/racerbot-ws/install/setup.bash')
            except Exception as e:      # noqa: BLE001 -- report, don't crash
                notes.append(f'live stage failed: {type(e).__name__}: {e}')

        reports = run_all(snap, live, spec=spec)
        out.write(render(reports, color=color, notes=notes))
        failed = any(l.status.value == 'FAIL' for r in reports for l in r.layers)
        return 1 if failed else 0
    except Exception:                   # noqa: BLE001
        out.write('racerbot_doctor crashed -- this is a bug in the doctor, '
                  'not a finding about the car:\n' + traceback.format_exc())
        return 2


if __name__ == '__main__':
    sys.exit(main())
