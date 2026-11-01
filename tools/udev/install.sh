#!/usr/bin/env bash
# Install car 1's udev rules: stable /dev/sensors/hokuyo and /dev/sensors/vesc
# symlinks, and Wiimote LED access. Copies of what is in /etc/udev/rules.d on
# car 1 (taken 2026-10-31). What each rule matches: docs/hardware-reference.md
# and docs/wiimote.md.
#
#   sudo tools/udev/install.sh
#
# Working when `ls -l /dev/sensors/` lists hokuyo and vesc with the devices plugged in.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo $0" >&2
    exit 1
fi
install -m 0644 "$here"/99-*.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
echo "Installed: $(cd "$here" && ls 99-*.rules | tr '\n' ' ')"
