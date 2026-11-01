#!/bin/bash
# Serve the car's live X display on 127.0.0.1:5900, for noVNC (novnc.service)
# and so for the remote desktop at rd-rb1.sfuracerbot.ca.
#
# Runs as root (x11vnc.service), because the display that is live changes:
# with nobody logged in it is GDM's login screen (:0, whose X authority only
# root and gdm can read); after someone logs in on the car's screen or
# through the remote desktop it is their GNOME session (:1). Car 2's unit
# hard-codes :1 and crash-loops on a car where nobody is logged in.
#
# The display served is the highest-numbered one with a live X server; the
# X authority file is read from that server's own command line (`-auth
# guess` cannot find GDM's). x11vnc is restarted whenever that changes.
#
# No VNC password: x11vnc listens on loopback only, and the only way in from
# off the car is the tunnel route behind a Cloudflare Access application
# that admits the team's emails (docs/web-dashboard.md, "Remote desktop and
# SSH"). The login screen still asks for the car's password.

set -u
PORT="${X11VNC_PORT:-5900}"

# Prints "<display> <authority file>" for the X server to serve, or nothing.
pick() {
  local sock n pid auth best=""
  for sock in /tmp/.X11-unix/X*; do
    [ -S "$sock" ] || continue
    n="${sock##*/X}"
    pid=$(ss -xlpH src "$sock" 2>/dev/null | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
    [ -n "$pid" ] || continue
    auth=$(tr '\0' '\n' < "/proc/$pid/cmdline" 2>/dev/null | sed -n '/^-auth$/{n;p;q}')
    [ -n "$auth" ] && [ -r "$auth" ] || continue
    if [ -z "$best" ] || [ "$n" -gt "${best%% *}" ]; then best="$n $auth"; fi
  done
  echo "$best"
}

trap 'kill "${vnc:-}" 2>/dev/null; exit 0' TERM INT

while true; do
  target=$(pick)
  if [ -z "$target" ]; then sleep 3; continue; fi
  echo "serving display :${target%% *}"
  x11vnc -display ":${target%% *}" -auth "${target#* }" -rfbport "$PORT" -localhost \
    -nopw -shared -forever -noxdamage -quiet &
  vnc=$!
  while kill -0 "$vnc" 2>/dev/null && [ "$(pick)" = "$target" ]; do sleep 3; done
  kill "$vnc" 2>/dev/null
  wait "$vnc" 2>/dev/null
  sleep 1
done
