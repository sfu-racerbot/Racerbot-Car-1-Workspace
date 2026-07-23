# Git setup / version control

How this workspace is versioned, what's a real git submodule vs. a plain vendored copy, and what to check before pulling upstream updates. Read this before touching `.gitmodules`, running any `git submodule` command, or updating `f1tenth_system`/`particle_filter`/`range_libc`/`transport_drivers` from upstream.

## Remote

- GitHub, private: `https://github.com/sfu-racerbot/Racerbot-Car-1-Workspace.git`
- Default/only branch: `main`
- `build/`, `install/`, `log/` (colcon artifacts), `__pycache__/`/`*.pyc`, and `.claude/` (local Claude Code settings, machine-specific) are gitignored — never expected in a commit.
- Car 2 is a related but independent sister repository, not a submodule or configured upstream. Compare against it and manually port selected changes only when explicitly requested; never wholesale-merge or overwrite Car 1 hardware configuration. See the main [README.md](../README.md).

## Submodules vs. vendored code — know which is which

| `src/` package | Tracking | Upstream | Branch/ref |
|---|---|---|---|
| `particle_filter` | **real git submodule** | `f1tenth/particle_filter` | `foxy-devel` |
| `range_libc` | **real git submodule** | `f1tenth/range_libc` | `humble-devel` |
| `transport_drivers` | **real git submodule** | `ros-drivers/transport_drivers` | `1.2.0` |
| `f1tenth_system` | **vendored (plain tracked files, NOT a submodule)** | was `f1tenth/f1tenth_system` | was pinned at `v2.0.0-beta-22-g94cb8d7` |

`f1tenth_system` is vendored to preserve this car's own hardware and launch differences: its working tree carried **three** uncommitted local patches that a plain submodule can't preserve across a fresh clone or a submodule update:

- `src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml` — the `human_control` profile's `drive-steering_angle` axis, changed from upstream's `axis: 2` to `axis: 3` (this car's F710 right stick in XInput mode; axis 2 is the left trigger — see [hardware-reference.md](hardware-reference.md#joystick--logitech-f710)). Verified against this car's own controller.
- `src/f1tenth_system/f1tenth_stack/config/sensors.yaml` — `urg_node` switched from upstream's default Ethernet params (`ip_address`/`ip_port`) to `serial_port: "/dev/sensors/hokuyo"`, because **this car's Hokuyo UST-30LX is USB, not Ethernet** — see [hardware-reference.md](hardware-reference.md#lidar--hokuyo-ust-30lx).
- `src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py` / `launch/teleop_launch.py` — upstream bundles `joy_teleop` into `bringup_launch.py` itself; here it's split out into its own `teleop_launch.py` so manual driving and autonomy are independent control layers you switch between with `Ctrl+C`, not a `pkill` dance (see [architecture.md](architecture.md#the-node-graph)). `bringup_launch.py` still starts `joy_node` (every control layer's deadman check needs it), just not `joy_teleop` anymore. This workspace also applies the split consistently to the `no_lidar_bringup_launch.py` and `sick_bringup_launch.py` variants.

A git submodule can only ever point at a commit in *someone else's* repo — there's no way to carry an uncommitted local edit through it into this repo's history. Vendoring was the simplest way to keep all three fixes without standing up a fork. **Practical effect: `f1tenth_system` will never move on its own.** There's no `git submodule update --remote` for it — updating it means manually pulling upstream changes and re-applying/re-checking all three patches (see below).

## Cloning this repo fresh

```bash
git clone --recurse-submodules https://github.com/sfu-racerbot/Racerbot-Car-1-Workspace.git
```
Forgot `--recurse-submodules`, or the clone predates a submodule being added? `particle_filter/`, `range_libc/`, `transport_drivers/` will exist but be empty:
```bash
git submodule update --init --recursive
```
`f1tenth_system` needs no such step — it's a plain part of the repo and comes with a normal clone.

## Checking for upstream updates — do this periodically, not just on breakage

None of the official `f1tenth`/roboracer/ros-drivers repos below have a `jazzy` branch yet (per the main [README.md](../README.md#notes)); everything here is `humble`-branch source built against ROS2 Jazzy. Before any dependency bump or if a `rosdep`/build error looks upstream-related, check each repo for a newer ROS2-distro branch rather than patching locally first:

- `particle_filter` / `range_libc` — https://github.com/f1tenth/particle_filter, https://github.com/f1tenth/range_libc
- `transport_drivers` — https://github.com/ros-drivers/transport_drivers
- `f1tenth_system` (vendored, no longer linked) — https://github.com/f1tenth/f1tenth_system

**Updating an actual submodule** (`particle_filter`, `range_libc`, `transport_drivers`):
```bash
cd src/<package>
git fetch origin
git checkout origin/<branch>   # e.g. humble-devel, or a newer distro branch if one now exists
cd ../..
git add src/<package>
git commit -m "Bump <package> submodule"
```
Rebuild just that package afterwards (`colcon build --symlink-install --packages-select <package>`) and re-test before trusting it.

**"Updating" `f1tenth_system`** (no submodule machinery to help you — do this manually):
1. Diff your vendored copy against a fresh clone/checkout of upstream `f1tenth/f1tenth_system` (whatever branch/commit you want to pull in) to see what actually changed upstream.
2. Apply the parts you want into `src/f1tenth_system/` by hand (copy files over, or `git diff`/`git apply` between the two trees).
3. **Before committing, re-check all three local patches**:
   - `f1tenth_stack/config/joy_teleop.yaml` — confirm `drive-steering_angle` under `human_control` is still `axis: 3`, not reverted to upstream's `axis: 2`.
   - `f1tenth_stack/config/sensors.yaml` — confirm `urg_node` still uses `serial_port: "/dev/sensors/hokuyo"`, not reverted to upstream's `ip_address`/`ip_port` Ethernet defaults.
   - `f1tenth_stack/launch/bringup_launch.py` — confirm it still does *not* start a `joy_teleop` node, with `teleop_launch.py` still present as the separate control layer (upstream will have them bundled back together).
   These are the three things guaranteed to get silently clobbered by a naive overwrite.
4. `git add src/f1tenth_system && git commit` as normal — there's no submodule pointer to bump, the files themselves are the commit.

## Other repo docs
- [hardware-reference.md](hardware-reference.md) — both local patches from the hardware side, and other exact config values for this car.
- [troubleshooting.md](troubleshooting.md) — the axis mixup as a symptom ("one axis doesn't do what you expect").
- [README.md](../README.md) — doc index and `src/` package table.
