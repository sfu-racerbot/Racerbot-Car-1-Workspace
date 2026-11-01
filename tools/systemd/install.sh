#!/usr/bin/env bash
# Car 1's own boot services: the web dashboard server and the remote desktop.
#
#   sudo tools/systemd/install.sh <user> <workspace>
#   e.g. sudo tools/systemd/install.sh "$USER" "$HOME/racerbot-ws"
#
# Installs, enables and (re)starts:
#   web-dashboard.service  dashboard_node on :8080, all interfaces like by hand (runs as <user>,
#                          from <workspace>'s install/, so keep it built)
#   x11vnc.service         the live X display on 127.0.0.1:5900 (root; runs
#                          /usr/local/sbin/racerbot-x11vnc-follow, a root-owned
#                          copy of x11vnc-follow.sh -- re-run this script after
#                          editing that file)
#   novnc.service          noVNC on 127.0.0.1:6080 (runs as <user>)
# and makes sure sshd is enabled, for ssh-rb1.sfuracerbot.ca.
#
# Not installed here: foxglove-bridge.service (src/web_dashboards/car/systemd/
# install.sh) and cloudflared (src/web_dashboards/car/cloudflared/install.sh).
# The camera is started by hand. See docs/web-dashboard.md.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws="$(cd "$here/../.." && pwd)"
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
if [ "$car" != rb1 ]; then
  echo "Car $car cannot use car 1's installer; car 2's installer is src/racerbot_launch/systemd/install.sh" >&2
  exit 1
fi
dry_run=0
if [ "${1:-}" = --dry-run ]; then dry_run=1; shift; fi
user="${1:-}"; workspace="${2:-}"
if [ "$#" -ne 2 ] || [ -z "$user" ] || [ -z "$workspace" ]; then
  echo "usage: $0 [--dry-run] <user> <workspace>" >&2; exit 1
fi
if (( dry_run )); then
  printf 'Car: %s\nUser: %s\nWorkspace: %s\nUnits: web-dashboard.service x11vnc.service novnc.service\n' "$car" "$user" "$workspace"
  exit 0
fi
if [ "$(id -u)" -ne 0 ]; then echo "run with sudo" >&2; exit 1; fi
if [ ! -d "$workspace/install/web_dashboard" ]; then
  echo "$workspace/install/web_dashboard not found -- build the workspace first (colcon build --symlink-install)" >&2
  exit 1
fi
missing=""
for bin in x11vnc websockify; do command -v "$bin" >/dev/null || missing="$missing $bin"; done
[ -d /usr/share/novnc ] || missing="$missing novnc"
if [ -n "$missing" ]; then
  echo "missing:$missing -- sudo apt install x11vnc novnc websockify" >&2; exit 1
fi

install -m 0755 -o root -g root "$here/x11vnc-follow.sh" /usr/local/sbin/racerbot-x11vnc-follow
install -m 0644 -o root -g root "$here/x11vnc.service" /etc/systemd/system/x11vnc.service
for unit in novnc web-dashboard; do
  sed -e "s|@USER@|$user|g" -e "s|@WORKSPACE@|$workspace|g" \
    "$here/$unit.service" > "/etc/systemd/system/$unit.service"
done

systemctl daemon-reload
systemctl enable ssh x11vnc novnc web-dashboard
systemctl restart x11vnc novnc web-dashboard
systemctl --no-pager --lines=3 status ssh x11vnc novnc web-dashboard || true
