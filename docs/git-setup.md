# Git setup / version control

> **Who this is for:** anyone cloning this repo fresh, or about to pull upstream changes into a vendored package.
> **Read first:** nothing, though [concepts.md](concepts.md) explains what the `src/` packages are.
> **You'll be able to:** tell submodules from vendored code, update either safely, and avoid clobbering this workspace's local fixes.

How this [workspace](glossary.md#workspace) — the whole `~/racerbot-ws` tree that colcon builds — is versioned. Which [packages](glossary.md#package) are real git submodules, which are plain vendored copies, and what to check before pulling upstream updates.

**Read this before touching `.gitmodules`, running any `git submodule` command, or updating any [package](glossary.md#package) listed below.**

---

## Why this page exists

Most of `src/` is other people's code. Some of it is linked to its original repository; one package deliberately isn't, because this workspace has patched it in ways a link can't carry.

Get that distinction wrong and you can silently revert a fix.

One of those fixes, if reverted, makes the car's [odometry](glossary.md#odometry--odom) — its own estimate of how far it has travelled — report the wrong sign. With no error message. Every speed-aware safety layer degrades along with it.

That's the whole reason this page is written down.

---

## First, a word on submodules

If you haven't used them: a **git submodule** is a pointer from this repository to a specific commit in *another* repository. This repo stores the URL and the commit ID, not the files.

That's useful: you get someone else's code at a known version without copying it in.

But it has one hard limitation, and that limitation is the reason for everything below.

> **A submodule can only ever point at a commit that exists in the other repo.** There is no way to carry a local edit through a submodule into this repository's history.

So if you need to patch third-party code and keep the patch, a submodule can't help you. You either fork upstream, or you **vendor** it — copy the files in and track them like your own.

This workspace vendors exactly one package, for exactly that reason.

---

## Remote

- GitHub: `sfu-racerbot/Racerbot-Car-1-Workspace` today. At cutover (plan Task 20), it is renamed to `sfu-racerbot/Racerbot-Car-Workspace`; GitHub redirects the old URL.
- Default and only branch: `main`
- Both cars use this workspace. Car 2's former repository, [Racerbot-Car-2-Workspace](https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace), is archived read-only only after both cars have driven on the merged repo.

**Before changing car-specific behavior, read [car-profiles.md](car-profiles.md)**: it explains how the shared launch files select each car's calibration.

## How car 2's history came in

Car 2's history entered as a merge on branch `unify/merge-car2`. Commit `a205f29`, “Merge car 2's workspace (Racerbot-Car-2-Workspace 9c2c2ed), keeping the best of both”, has parents `560da2c` (car 1's line) and `9c2c2ed` (car 2's main). As a result, `git log` shows the full history from both cars. Car 2's main is also kept at `refs/car2/main`.

Every change from that merge to what either car runs is listed in [unify-behaviour-changes.md](superpowers/plans/unify-behaviour-changes.md).

**Gitignored, never expected in a commit:** `build/`, `install/`, `log/` (colcon artifacts), `__pycache__/` and `*.pyc`, `.sim`, `.venv-f1tenth/`, `.pytest_cache/`.

`.claude/` is ignored **except** `.claude/skills/`, which is tracked deliberately — it holds the [documentation standard](../.claude/skills/novice-docs/SKILL.md) and its checker, which belong to the team rather than to one machine.

---

## Submodules vs. vendored code — know which is which

| `src/` package | Tracking | Upstream | Branch |
|---|---|---|---|
| `racerbot_a` | **team-developed git submodule** | `sfu-racerbot/racerbot_a` | `main` |
| `racerbot_b` | **team-developed git submodule** | `sfu-racerbot/racerbot_b` | `main` |
| `particle_filter` | **real git submodule** | `f1tenth/particle_filter` | `humble-devel` |
| `range_libc` | **real git submodule** | `f1tenth/range_libc` | `humble-devel` |
| `transport_drivers` | **real git submodule** | `ros-drivers/transport_drivers` | `humble` |
| `web_dashboards` | **team git submodule** | `sfu-racerbot/web-dashboards` | `main` (pinned commit; holds the `web_dashboard` and `usb_cam_stream` packages under `car/ros/`) |
| `f1tenth_system` | **vendored (plain tracked files, NOT a submodule)** | was `f1tenth/f1tenth_system` | was `humble-devel` |
| `tools/imu_calibration` (not under `src/`) | **merged in with its history (git subtree, 2026-10-31), plain tracked files** | was `sfu-racerbot/imu-calibration` | was `main` (`0d99cd2`) |

**`tools/imu_calibration` is now the copy that counts.** It is the club's IMU calibration stack (`racer_imu`, the BNO085 firmware and bridge, `config/car.yaml`, `calib/*.json`). Its three commits are in this repo's history (`git log -- tools/imu_calibration`), and car 1's calibration, which was never committed upstream, was committed here on top. Edit it here; the old GitHub repo no longer receives changes. Its raw calibration recordings are in `data/imu_calibration/`.

**Team-developed vs. workspace code.** `racerbot_a` and `racerbot_b` are the codebases the Racerbot teams are actively developing.

The workspace-specific integration code and documentation outside those two repositories were produced through vibe coding (AI-assisted development). The remaining submodules and vendored packages are third-party upstream code, not team-authored.

---

## The one vendored package, its fixes and additions

`f1tenth_system` used to be a submodule. It was **deliberately disconnected from upstream** and converted to a normal tracked directory.

Concretely: its `.git` gitlink and its `.gitmodules` / `.git/config` entries were removed, then the files were kept and `git add`-ed like any other package.

It carries local launch and shared-configuration changes that have to live in this repo's history. Car-specific settings are loaded from each car's calibration overlay.

> **All five are guaranteed to be silently clobbered by a naive upstream overwrite.** Check every one of them after any sync.

### Fix 1 — the steering axis

`src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`

The `human_control` profile's `drive-steering_angle` axis is **`axis: 3`**. Its per-car joystick axes and scales are in `src/racerbot_car/cars/<id>/calibration/joy_teleop.yaml`; the vendored file keeps shared settings and the `# per car:` pointer.

Upstream's axis 2 is this F710's *left trigger*. Axis 3 is the right stick, in XInput mode — see [hardware-reference.md](hardware-reference.md#joystick--logitech-f710).

**How it fails if reverted:** obviously and harmlessly. Steering moves to the trigger, you notice immediately.

### Fix 2 — teleop split into its own launch file

[Teleop](glossary.md#teleop) means driving the car by hand with the gamepad. A [launch file](glossary.md#launch-file) is the script that starts a set of nodes together.

`src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py` and `launch/teleop_launch.py`

Upstream bundles `joy_teleop` into `bringup_launch.py` itself. Here it's split out into its own [launch file](glossary.md#launch-file), `teleop_launch.py`.

That makes manual driving ([teleop](glossary.md#teleop)) and autonomy independent control layers you switch between with `Ctrl+C` — not a `pkill` dance. See [architecture.md](architecture.md#the-node-graph).

`bringup_launch.py` still starts `joy_node`, because every control layer's deadman check needs it. It just doesn't start `joy_teleop` any more.

The two alternate bringups, `no_lidar_bringup_launch.py` and `sick_bringup_launch.py`, got the same split on 2026-09-26 (they had been missed, and still started `joy_teleop`).

**How it fails if reverted:** confusingly. Autonomy silently stops working, because a bundled `joy_teleop` masks `/drive` permanently — see [troubleshooting.md](troubleshooting.md#autonomy-node-publishes-to-drive-car-doesnt-move-no-errors-anywhere).

### Fix 3 — the odometry sign, which fails silently

`src/f1tenth_system/f1tenth_stack/config/vesc.yaml`

The sign correction remains per car, with its gain in `src/racerbot_car/cars/<id>/calibration/vesc.yaml`. The vendored file retains shared VESC settings and a `# per car:` pointer. The [VESC](glossary.md#vesc) is the motor controller.

> **This is the dangerous one.** Nothing errors if it's lost. Odometry just reports the wrong sign, and every speed-aware safety layer degrades along with it.

**Verify both nodes after any upstream sync:**

```bash
ros2 param get /vesc_to_odom_node speed_to_erpm_gain      # must be negative for this car
ros2 param get /ackermann_to_vesc_node speed_to_erpm_gain # must be positive for this car
```

**Working when:** the two values are opposite in sign, exactly as shown. If they match, Fix 3 has been lost.

### Fix 4 — this car's measured geometry, which also fails silently

`src/f1tenth_system/f1tenth_stack/config/vesc.yaml` and
`src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py`

The per-car wheelbase is in `cars/<id>/calibration/vesc.yaml`. The shared geometry remains in the vendored launch and config files where it is common to both cars.

- Each car's calibration overlay provides its own wheelbase value; `vesc_to_odom_node` uses it when integrating heading.
- The `base_link`→`laser` transform is shared by the current profiles and remains in the vendored launch files.

See [hardware-reference.md](hardware-reference.md#physical-dimensions-used-in-config) for the measurements and the full list of files each one has to stay in step with.

> **This one fails as quietly as Fix 3.** Nothing errors. The map just builds slightly wrong, the car's modelled nose moves 0.09 m, and every clearance decision shifts with it.

**Verify after any upstream sync:**

**Terminal 1** — the driver stack, which the two checks read from.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** it settles into a steady stream of logs and stays up. It starts no [node](glossary.md#node) that can move the car on its own. Leave it running.

**Terminal 2** — the checks themselves.

```bash
ros2 param get /vesc_to_odom_node wheelbase        # must be 0.36
ros2 run tf2_ros tf2_echo base_link laser          # translation x must be 0.260
```

**Working when:** the parameter reads `0.36` and the transform's x translation reads `0.260`. Ctrl+C the `tf2_echo` once you've seen one message.

### Fix 5 — each car's LiDAR connection

`src/f1tenth_system/f1tenth_stack/config/sensors.yaml`

Each car's LiDAR connection is in `src/racerbot_car/cars/<id>/calibration/sensors.yaml`. Car 1 uses `/dev/sensors/hokuyo`; the vendored `sensors.yaml` keeps shared settings and its `# per car:` pointer.

**How it fails if reverted:** the driver tries the wrong connection and `/scan` does not publish.

### Addition 7 — bringups load each car's calibration overlay

All three `*_bringup_launch.py` files and `teleop_launch.py` load shared configuration first, then the selected car's overlay using `racerbot_car.layers.car_params(...)` and `launch_car_id()`. `f1tenth_stack/package.xml` adds `<exec_depend>racerbot_car</exec_depend>`.

The change stays small: calibration values live in `src/racerbot_car/cars/rb1/calibration/` and `cars/rb2/calibration/`, while vendored configuration retains shared values and `# per car:` pointers. This keeps one launch implementation for both cars.

If reverted, the shared defaults are used without the selected car's calibration, so steering, speed conversion, or sensor setup can be wrong. `src/racerbot_launch/test/test_car_profiles.py` checks the car hardware profiles; `src/racerbot_car/test/test_launch_layering.py` checks overlay layering.

### Addition 6 — bringup starts the automatic run recorder

`src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py`, `no_lidar_bringup_launch.py`, `sick_bringup_launch.py`, and `f1tenth_stack/package.xml`

Not a fix but an addition (2026-10-26): each bringup declares a `record` launch argument (default `true`) and includes `race_diagnostics`' `auto_record_launch.py`, which records every run of an algorithm. See [run-diagnostics.md](run-diagnostics.md#automatic-recording).

It is kept as small as it can be. About 20 lines at the end of each bringup; the recorder itself lives in `race_diagnostics`, which this repo owns. The lookup is wrapped so a missing `race_diagnostics` only logs a warning: bringup still starts. `package.xml` gains `<exec_depend>race_diagnostics</exec_depend>`.

The layered-override pattern below doesn't fit here, because there is no config file to layer: the change is which processes bringup starts. And the whole point is that the usual `ros2 launch f1tenth_stack bringup_launch.py` command records, without anyone having to remember a second one.

**How it fails if reverted:** quietly but harmlessly. Nothing drives differently; runs just stop being recorded. `python3 -m pytest src/race_diagnostics/test/test_bringup_auto_record.py src/racerbot_launch/test/test_car_profiles.py` fails if any bringup loses it.

### Not a fix: the SLAM tuning lives outside this package

[SLAM](glossary.md#slam) is the software that builds a map and locates the car in it at the same time.

`src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml` is **unmodified**, and should stay that way.

This workspace does change how `slam_toolbox` is tuned: how often it corrects the car's position, and how far it looks when trying to close a loop. Those changes live in a file this repo owns:

`src/racerbot_launch/config/slam_tracking.yaml`

`slam_launch.py` loads the vendored config first and then layers ours on top, so later values win. The vendored file stays clean, an upstream sync cannot take our tuning with it, and there is one place to read what this workspace changed about SLAM.

**Follow the same pattern for anything else you want to change about a vendored node.** A sixth entry on the list above is a sixth thing to check after every sync; a layered override file is none.

Why those particular values moved is in [localization.md](localization.md).

<details>
<summary><b>Why the two nodes need opposite signs</b> — the upstream hardcoded negation this cancels. Read before "tidying up" what looks like an inconsistency.</summary>

It looks like a typo. It isn't, and the asymmetry is load-bearing.

`vesc_ackermann`'s `vesc_to_odom.cpp` hardcodes:

```
speed = (-state.speed - offset) / gain
```

That negates the eRPM the VESC reports. Upstream assumes a VESC that reports eRPM with the *opposite* sign to the commanded eRPM.

**This car's VESC doesn't.** It reports eRPM with the same sign as the command. So without the override, `/odom` publishes **negative** `twist.linear.x` while the car drives forward — and integrates the `odom`→`base_link` TF backwards along with it.

The negative override cancels that hardcoded negation, for odometry only. [TF](glossary.md#tf--transform--frame) — ROS2's record of where things sit relative to each other — is built from that odometry, which is why a sign error there propagates into everything positional.

`ackermann_to_vesc_node` — which sets the actual motor command — **must keep `+3650.0`**. Flip that one and a forward command drives the car backwards.

So: same parameter name, two nodes, opposite signs, both correct.

Added 2026-07-27, after a `gap_follow` collision. See [hardware-reference.md](hardware-reference.md).

</details>

### What vendoring costs you

**`f1tenth_system` will never move on its own.** There's no `git submodule update --remote` for it any more.

Updating it means manually pulling upstream changes and re-applying or re-checking the local modifications — [the procedure is below](#updating-f1tenth_system--manual-no-submodule-machinery).

---

## Cloning this repo fresh

```bash
git clone --recurse-submodules https://github.com/sfu-racerbot/Racerbot-Car-1-Workspace.git
```

**Working when:** `src/particle_filter/` and the other submodule directories contain files, not just empty folders.

**Forgot `--recurse-submodules`?** Or does your clone predate a submodule being added? The submodule paths — `racerbot_a/`, `racerbot_b/`, `particle_filter/`, `range_libc/`, `transport_drivers/`, `web_dashboards/` — will exist but be empty. Fix it with:

```bash
git submodule update --init --recursive
```

`f1tenth_system` needs no such step. It's a plain part of the repo and arrives with a normal clone — which is the upside of vendoring.

---

## Working on Racerbot A or Racerbot B

**Order matters here.** Team code must be committed and pushed from inside the submodule *first*, then the pointer committed in this workspace.

**Terminal 1, from `~/racerbot-ws`:**

```bash
cd src/racerbot_b                 # or src/racerbot_a
git add .
git commit -m "Describe the team code change"
git push origin main

cd ../..
git add src/racerbot_b            # or src/racerbot_a
git commit -m "Bump Racerbot B submodule"
git push origin main
```

> **Never leave a team-code commit only on one machine.** The workspace repository records only the referenced commit ID, not the submodule's files.
>
> Push the submodule commit before pushing the workspace pointer. Do it the other way round and everyone else gets a pointer to a commit that doesn't exist anywhere they can reach — their clone breaks, and the fix has to come from your laptop.

---

## Checking for upstream updates

Do this periodically, not just when something breaks.

**The context:** none of the official `f1tenth` / roboracer / ros-drivers repos below have a `jazzy` branch yet (per the main [README.md](../README.md#notes)). Everything here is `humble`-branch source built against ROS2 Jazzy.

So before any dependency bump — or if a `rosdep` or build error looks upstream-related — check each repo for a newer ROS2-distro branch *before* patching locally.

| Package | Upstream |
|---|---|
| `particle_filter` | https://github.com/f1tenth/particle_filter |
| `range_libc` | https://github.com/f1tenth/range_libc |
| `transport_drivers` | https://github.com/ros-drivers/transport_drivers |
| `f1tenth_system` (vendored, no longer linked) | https://github.com/f1tenth/f1tenth_system |

### Updating an actual submodule

For `particle_filter`, `range_libc`, `transport_drivers`:

**Terminal 1, from `~/racerbot-ws`:**

```bash
cd src/<package>
git fetch origin
git checkout origin/<branch>   # e.g. humble-devel, or a newer distro branch if one now exists
cd ../..
git add src/<package>
git commit -m "Bump <package> submodule"
```

Then rebuild just that package and re-test before trusting it:

```bash
colcon build --symlink-install --packages-select <package>
```

**Working when:** `Summary: 1 package finished`, and whatever that package does still works on the car.

### Updating `f1tenth_system` — manual, no submodule machinery

1. **Diff your vendored copy against upstream.** Take a fresh clone or checkout of `f1tenth/f1tenth_system` at whatever branch or commit you want to pull in, and see what actually changed.
2. **Apply the parts you want by hand** into `src/f1tenth_system/` — copy files over, or use `git diff` / `git apply` between the two trees.
3. **Before committing, re-check all five local fixes and the one addition.** Every one of them:

   | Check | Should still be |
   |---|---|
   | `f1tenth_stack/config/joy_teleop.yaml` | shared configuration plus the `# per car:` pointer; axes and scales come from `cars/<id>/calibration/joy_teleop.yaml` |
   | `f1tenth_stack/launch/bringup_launch.py`, `no_lidar_bringup_launch.py`, `sick_bringup_launch.py` | still do **not** start a `joy_teleop` node, with `teleop_launch.py` still present separately (upstream will have bundled them back together) |
   | `f1tenth_stack/config/vesc.yaml` and `sensors.yaml` | shared settings and `# per car:` pointers remain; VESC values and LiDAR connection come from each car's calibration overlay |
   | all three `*_bringup_launch.py` files, and `f1tenth_stack/package.xml` | still declare `record` (default `true`) and include `race_diagnostics`' `auto_record_launch.py` (Addition 6); `package.xml` still has `<exec_depend>race_diagnostics</exec_depend>`. `python3 -m pytest src/race_diagnostics/test/test_bringup_auto_record.py` checks the launch files |

4. **Commit normally:** `git add src/f1tenth_system && git commit`. There's no submodule pointer to bump — the files themselves are the commit.

> **Do step 3 even when you're confident.** A naive overwrite can remove the calibration loading and make the car use the wrong hardware values.

One more difference from upstream is **not** a fix and is safe to lose. Upstream's three bringups remap the [mux](glossary.md#mux--multiplexer)'s `ackermann_cmd_out` → `ackermann_drive`. The mux never uses the name `ackermann_cmd_out`, so the remap did nothing, and it was deleted here in October 2026. If a sync brings it back, `python3 -m pytest src/racerbot_launch/test/` fails in `test_drive_path_wiring.py`; delete the line again.

---

## Related docs

- [hardware-reference.md](hardware-reference.md) — the axis-3 fix from the hardware and joystick side, plus other exact config values for this car.
- [troubleshooting.md](troubleshooting.md) — the axis mixup as a symptom you'd actually notice ("one axis doesn't do what you expect").
- [README.md](../README.md) — doc index and `src/` package table.
