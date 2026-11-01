# Web dashboard on car 1

> **Who this is for:** anyone who wants to watch or tune car 1 from a browser, or change how its dashboard, camera stream or Lichtblick bridge are set up.
> **Read first:** nothing. The full manual — every panel, tuning, measuring, clearing maps, stopping processes, the security note — is **[web-dashboards/car/docs/web-dashboard.md](../src/web_dashboards/car/docs/web-dashboard.md)**.
> **You'll be able to:** open car 1 at https://dashboard.sfuracerbot.ca, start its camera, reach its desktop and a shell from anywhere, and know which file holds car 1's settings.
> **Time:** 5 minutes; 10 more for remote desktop and SSH.

The dashboard moved out of this workspace on 2026-09-27. Its code, its docs and the site now live in one repo, **[sfu-racerbot/web-dashboards](https://github.com/sfu-racerbot/web-dashboards)**, which this workspace includes as a git submodule at `src/web_dashboards`. The package names did not change: `web_dashboard` and `usb_cam_stream` build and run exactly as before.

This page is only what is specific to car 1. Car 1's id on the site is **`rb1`** (car 2 is `rb2`).

## Highlights

- **One address, behind a login.** Open https://dashboard.sfuracerbot.ca, sign in with your team email, pick **Car 1**. The car serves no pages itself any more.
- **Car 1's settings are one file here.** [`src/racerbot_launch/config/web_dashboard_rb1.yaml`](../src/racerbot_launch/config/web_dashboard_rb1.yaml) holds everything that makes the generic packages car 1's: the site, the LiDAR offset, which nodes are tunable and stoppable, where saved runs live, and the USB webcam. A test checks each value against the file it has to agree with.
- **Always reachable.** On car 1 the dashboard's server starts at boot (`web-dashboard.service`), alongside `foxglove-bridge` (Lichtblick). Only the camera is started by hand. This differs from the web-dashboards repo, which starts the dashboard by hand too.
- **Remote desktop and a shell from anywhere**, through the same Cloudflare tunnel and team login: `rd-rb1.sfuracerbot.ca` (the car's screen, in a browser) and `ssh-rb1.sfuracerbot.ca` ([below](#remote-desktop-and-ssh)).

## Start it

**No driving layer is needed, and none of this can move the car**: the dashboard and the camera publish to no topic ([why](../src/web_dashboards/car/docs/web-dashboard.md#why-its-safe-to-start-at-any-time)).

**The dashboard's server is already running** — it starts at boot ([Car 1's boot services](#car-1s-boot-services)). Check it, **Terminal 1, on the car:**

```bash
systemctl status web-dashboard --no-pager
```

**Working when:** it says `active (running)`, and `journalctl -u web-dashboard -n 20` says it is serving the WebSocket on port 8080. The site shows Car 1 as `CAR ONLINE`.

Don't also run `ros2 launch racerbot_launch dashboard_launch.py` by hand while the service is up: the second copy can't get port 8080. To run it by hand (to watch its log in a terminal, say), `sudo systemctl stop web-dashboard` first.

**Terminal 2, from `~/racerbot-ws`** — the USB webcam's MJPEG stream, for the camera inset (car 1 has no RealSense):

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_launch webcam_launch.py
```

**Working when:** the camera inset on the site fills in within a few seconds. With no webcam plugged in, the stream keeps running and logs that it is waiting for `/dev/video0`.

**Everything but the driving stack, in one command** (LiDAR, webcam stream, dashboard — for bench testing): `ros2 launch racerbot_launch dashboard_test_launch.py`.

**With the simulator:** `ros2 launch racerbot_sim sim_auto_map_race_launch.py track:=indoor_wide dashboard:=true` ([sim-validation.md](sim-validation.md)).

`http://<car-ip>:8080/` now answers 404 — correct, not a fault. The old page that used to be served there (`serve_static`) was removed; use the site.

## If the site can't reach the car

**Terminal 1, on the car:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 run web_dashboard remote_check --site https://dashboard.sfuracerbot.ca --car rb1
```

**Working when:** it ends with `Everything this car can check is fine`. Then open https://dashboard.sfuracerbot.ca/rb1/check for the site's side. Together they name the hop that fails.

Car 1's tunnel is the `rb1` tunnel in the team's Cloudflare account. A **tunnel** is `cloudflared`, a system service on the car that holds one outgoing connection to Cloudflare; each **route** maps a public hostname to a port on the car. Car 1's routes:

| Hostname | Service on the car | What it is | Who gets in (Cloudflare Access) |
|---|---|---|---|
| `rb1-dash-origin.sfuracerbot.ca` | `http://localhost:8080` | `dashboard_node` | the site's service token only |
| `rb1-bridge-origin.sfuracerbot.ca` | `http://localhost:8765` | `foxglove_bridge` | the site's service token only |
| `rb1-cam-origin.sfuracerbot.ca` | `http://localhost:9090` | camera stream | the site's service token only |
| `rd-rb1.sfuracerbot.ca` | `http://localhost:6080` | noVNC remote desktop | the team's emails |
| `ssh-rb1.sfuracerbot.ca` | `ssh://localhost:22` | SSH | the team's emails |

The same five as car 2's, with `rb2` changed to `rb1`. The routes and Access applications live in the Cloudflare dashboard, not on the car: [web-dashboards docs/car-1-setup.md](https://github.com/sfu-racerbot/web-dashboards/blob/main/docs/car-1-setup.md). Setting one up from scratch: [web-dashboards/car/README.md](../src/web_dashboards/car/README.md).

**The tunnel's token** (its password to Cloudflare) is in `/etc/cloudflared/token`, readable only by root. The unit passes it with `--token-file`. That is what Cloudflare's own `cloudflared service install` writes, from cloudflared 2026.9 on; the token was refreshed and installed that way on 2026-10-24. The web-dashboards installer (`car/cloudflared/install.sh`) writes `/etc/cloudflared/tunnel.env` instead; either is fine.

It is not on `cloudflared`'s command line any more. There, any user on the car could read it with `ps` or `systemctl cat cloudflared`. Older cloudflared versions' `cloudflared service install <token>` put it there, so re-check after any reinstall:

```bash
ps -o args= -C cloudflared          # Terminal 1, on the car
```

**Working when:** it prints `/usr/bin/cloudflared --no-autoupdate tunnel run`, followed by nothing or by `--token-file /etc/cloudflared/token`. A long `eyJ…` string after `--token` means the token is exposed again.

## Changing car 1's settings

Edit [`web_dashboard_rb1.yaml`](../src/racerbot_launch/config/web_dashboard_rb1.yaml), not the package's own YAML in the submodule: that one holds the generic defaults every team shares. Only the keys you change go in the rb1 file; it is loaded on top.

Then check it before trusting it — **Terminal 1, from `~/racerbot-ws`:**

```bash
python3 -m pytest src/racerbot_launch/test/ -v
```

**Working when:** every test passes. They catch a misspelt key (which ROS would otherwise silently ignore), a LiDAR offset that disagrees with the bringup launches' transform, a site origin that disagrees with the site's own, and a stoppable process or map folder the dashboard would silently drop.

## Live tuning and the contract with the driving nodes

The tuning panel reaches `pure_pursuit_node` and `gap_follow_node` because each advertises a `live_tunable_spec`, and the rb1 YAML names them in `tuning_nodes`.

That agreement — the spec format, who enforces bounds, how "save" rewrites `config/*.yaml` — now spans two repos. It is written down in [web-dashboards/car/docs/web-dashboard.md, "The live_tunable_spec contract"](../src/web_dashboards/car/docs/web-dashboard.md#the-live_tunable_spec-contract).

This workspace's half is tested in `src/gap_follow/test/test_gap_follow_live_tuning.py` and `src/pure_pursuit/test/test_pure_pursuit_live_tuning.py`: each node's real spec must parse with the dashboard's `tuning.parse_spec`, and each node's live config must round-trip through the dashboard's YAML writer. **Change a spec or a config's shape and run those.**

## The Lichtblick bridge's boot service

`foxglove-bridge.service` starts foxglove_bridge at boot. Its unit and wrapper now live in the submodule (`src/web_dashboards/car/systemd/`), and the bridge's launch file in `web_dashboard`. **Browsers can publish to no topic through it** — see [foxglove-bridge.md](foxglove-bridge.md).

It was installed on car 1 from the submodule in October 2026. To reinstall it — after moving the workspace, say — **Terminal 1, on the car** (needs sudo):

```bash
sudo ~/racerbot-ws/src/web_dashboards/car/systemd/install.sh racerbotcar-1 /home/racerbotcar-1/racerbot-ws
```

**Working when:** it prints `active (running)`, `systemctl cat foxglove-bridge` shows `ExecStart=/home/racerbotcar-1/racerbot-ws/src/web_dashboards/car/systemd/foxglove-bridge.sh`, and `journalctl -u foxglove-bridge -n 20` shows `Server listening on port 8765`.

(Before the move the unit ran `tools/systemd/foxglove-bridge.sh` → `ros2 launch racerbot_launch foxglove_bridge_launch.py`. Both were removed once the unit was reinstalled; if `systemctl cat foxglove-bridge` still names `tools/systemd/foxglove-bridge.sh`, run the command above. Today's `tools/systemd/` holds car 1's other services, next section.)

## Car 1's boot services

Three more services start at boot on car 1, all from [`tools/systemd/`](../tools/systemd/):

| Service | Listens on | Runs as | What it is |
|---|---|---|---|
| `web-dashboard` | `:8080`, every interface (as when started by hand) | `racerbotcar-1` | `ros2 launch racerbot_launch dashboard_launch.py` |
| `x11vnc` | `127.0.0.1:5900` | root | The car's screen, as VNC |
| `novnc` | `127.0.0.1:6080` | `racerbotcar-1` | A web page that shows that VNC in a browser |

`sshd` is enabled too; it comes with Ubuntu.

**Why the dashboard at boot:** so the site can always reach car 1 without someone first opening a terminal on it (decided 2026-10-24). It is support tooling: it publishes to no topic. Its write paths — live tuning, stopping a driving process, resetting SLAM, deleting a saved run — are in the [full manual's security note](../src/web_dashboards/car/docs/web-dashboard.md). The camera stays by hand.

**Why x11vnc runs as root:** what's on the car's screen changes. With nobody logged in it is GDM's login screen (display `:0`), whose X authority file (the key to that screen) only root can read. After someone logs in it is their GNOME session (`:1`).

`x11vnc-follow.sh` serves the highest-numbered live display, and restarts x11vnc when that changes. Car 2's unit hard-codes `:1` instead. On car 1, where nobody stays logged in, that crash-looped every 5 seconds from the day it was copied over (17,889 restarts by 2026-10-24).

The installer copies `x11vnc-follow.sh` to `/usr/local/sbin/racerbot-x11vnc-follow`, owned by root. The service runs that copy, never the workspace file: anyone who can edit the workspace could otherwise run code as root. **Re-run the installer after editing `x11vnc-follow.sh`.**

To install or reinstall all three — **Terminal 1, on the car** (needs sudo; the workspace must be built):

```bash
sudo ~/racerbot-ws/tools/systemd/install.sh racerbotcar-1 /home/racerbotcar-1/racerbot-ws
```

**Working when:** it prints `active (running)` for `ssh`, `x11vnc`, `novnc` and `web-dashboard`. Then:

- `~/racerbot-ws/src/web_dashboards/car/check.sh` says `ok` for ports 8080 and 8765. Port 9090 is `MISSING` until you start the camera; that's expected.
- `journalctl -u x11vnc -n 5` shows `serving display :0`, or `:1` while someone is logged in.

## Remote desktop and SSH

Both go through the tunnel behind the team login, the same way car 2's do. Nothing on the car is open to the internet: x11vnc and noVNC listen on loopback only.

**Remote desktop:** open **https://rd-rb1.sfuracerbot.ca/vnc.html**, sign in with your team email, choose **Connect**. There is no VNC password; the Cloudflare login is the gate. With nobody logged in you see the car's login screen, and log in with the car's own password.

**Working when:** you see the car's screen and can move its mouse. "Failed to connect to server" means x11vnc isn't up: `systemctl status x11vnc` on the car.

**SSH, from your laptop.** Install `cloudflared` on the laptop ([Cloudflare's downloads](https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/downloads/)), then add this to the laptop's `~/.ssh/config`:

```
Host rb1
    HostName ssh-rb1.sfuracerbot.ca
    User racerbotcar-1
    ProxyCommand cloudflared access ssh --hostname %h
```

**Terminal 1, on your laptop:**

```bash
ssh rb1
```

**Working when:** a browser tab opens for the team login the first time, then you get the car's password prompt and a shell. `websocket: bad handshake` means the `ssh-rb1` route or its Access application is missing in Cloudflare.

## Tests

`web_dashboard` and `usb_cam_stream` are still built and tested by this workspace's colcon, from the submodule. **Run ROS tests on an isolated domain** — node tests start real driving nodes that join whatever graph they can see. They publish their drive commands on `/test_only/drive`, never the real `/drive`, but still put latched topics such as `/racing_line` on the graph, and the running dashboard shows them:

**Terminal 1, from `~/racerbot-ws`:**

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  colcon test --packages-select web_dashboard usb_cam_stream racerbot_launch
colcon test-result --verbose
```

**Working when:** 0 errors, 0 failures. Which runner runs which of the packages' tests: [web-dashboards/car/README.md, "Running the tests"](../src/web_dashboards/car/README.md#running-the-tests).

## Updating the submodule

The submodule is pinned to a commit. To take a newer web-dashboards:

**Terminal 1, from `~/racerbot-ws`:**

```bash
git -C src/web_dashboards fetch origin && git -C src/web_dashboards checkout origin/main
colcon build --symlink-install --packages-select web_dashboard usb_cam_stream
```

**Working when:** the build finishes, the tests above pass, and the site still shows Car 1 online. Then commit the new pointer (`git add src/web_dashboards`). A site whose protocol version moved past the car's shows a red banner — rebuilding and restarting the dashboard fixes it.
