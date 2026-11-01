#!/usr/bin/env bash
# Install udev rules: stable /dev/sensors/vesc and, for serial LiDAR,
# /dev/sensors/hokuyo symlinks, and Wiimote LED access. Copies of rules on
# car 1 (taken 2026-10-31). What each rule matches: docs/hardware-reference.md
# and docs/wiimote.md.
#
#   sudo tools/udev/install.sh
#
# Working when `ls -l /dev/sensors/` lists hokuyo and vesc with the devices plugged in.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
ws="$(cd "$here/../.." && pwd)"
selection="$(python3 - "$ws" <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, str(Path(sys.argv[1]) / 'src/racerbot_car'))
try:
    from racerbot_car.identity import current_car_id
    from racerbot_car.profile import LidarSerial, load_profile
    car = current_car_id()
    print(car, 'serial' if isinstance(load_profile(car).lidar, LidarSerial) else 'ethernet')
except Exception as error:
    print(f'Cannot resolve car identity or profile (RACERBOT_CAR): {error}', file=sys.stderr)
    sys.exit(1)
PY
)" || exit 1
read -r car lidar <<< "$selection"
rules=(99-vesc.rules 99-wiimote-leds.rules)
if [ "$lidar" = serial ]; then rules+=(99-hokuyo.rules); fi
if [ "${1:-}" = --dry-run ] && [ "$#" -eq 1 ]; then
    printf 'Car: %s\nRules:' "$car"
    printf ' %s' "${rules[@]}"
    printf '\n'
    exit 0
fi
if [ "$#" -ne 0 ]; then echo "usage: $0 [--dry-run]" >&2; exit 1; fi
if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo $0" >&2
    exit 1
fi
for rule in "${rules[@]}"; do install -m 0644 "$here/$rule" /etc/udev/rules.d/; done
udevadm control --reload-rules
udevadm trigger
printf 'Installed: %s\n' "${rules[*]}"
