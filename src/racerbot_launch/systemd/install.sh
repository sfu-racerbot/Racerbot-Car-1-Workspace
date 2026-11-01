#!/usr/bin/env bash
# Start car 2's web dashboard server at boot.
#
#   sudo src/racerbot_launch/systemd/install.sh <user> <workspace>
#   e.g. sudo src/racerbot_launch/systemd/install.sh "$USER" "$HOME/racerbot-ws"
#
# <user> runs the service (the account that owns the ROS 2 workspace);
# <workspace> is the colcon workspace with racerbot_launch and web_dashboard
# built into it.
#
# Installs /etc/systemd/system/racerbot-dashboard.service, enables it, and
# (re)starts it so a changed unit takes effect now. The unit runs the
# workspace's install space, not this checkout, so it keeps working from
# wherever this script was run.
#
# The Lichtblick bridge has its own unit in the submodule
# (src/web_dashboards/car/systemd/install.sh); the camera is still started
# by hand (docs/web-dashboard.md).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws="$(cd "$here/../../.." && pwd)"
car="$(python3 - "$ws" <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, str(Path(sys.argv[1]) / 'src/racerbot_car'))
try:
    from racerbot_car.identity import current_car_id
    print(current_car_id())
except Exception as error:
    print(f'Cannot resolve car identity (RACERBOT_CAR): {error}', file=sys.stderr)
    sys.exit(1)
PY
)" || exit 1
if [ "$car" != rb2 ]; then
  echo "Car $car cannot use car 2's installer; car 1's installer is tools/systemd/install.sh" >&2
  exit 1
fi
dry_run=0
if [ "${1:-}" = --dry-run ]; then dry_run=1; shift; fi
user="${1:-}"; workspace="${2:-}"
if [ "$#" -ne 2 ] || [ -z "$user" ] || [ -z "$workspace" ]; then
  echo "usage: $0 [--dry-run] <user> <workspace>" >&2; exit 1
fi
if (( dry_run )); then
  printf 'Car: %s\nUser: %s\nWorkspace: %s\nUnits: racerbot-dashboard.service\n' "$car" "$user" "$workspace"
  exit 0
fi
if [ "$(id -u)" -ne 0 ]; then echo "run with sudo" >&2; exit 1; fi
for pkg in racerbot_launch web_dashboard; do
  if [ ! -d "$workspace/install/$pkg" ]; then
    echo "$workspace/install/$pkg not found -- build the workspace first (colcon build --symlink-install)" >&2
    exit 1
  fi
done

sed -e "s|@USER@|$user|g" -e "s|@WORKSPACE@|$workspace|g" \
  "$here/racerbot-dashboard.service" > /etc/systemd/system/racerbot-dashboard.service
systemctl daemon-reload
systemctl enable racerbot-dashboard
systemctl restart racerbot-dashboard
systemctl --no-pager --lines=5 status racerbot-dashboard || true
