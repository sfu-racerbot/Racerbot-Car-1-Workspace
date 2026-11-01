# Developer tools: running tests safely, proving they bite, checking a command starts

> **Who this is for:** anyone writing or changing code or tests in this repository, human or agent.
> **Read first:** the "Build, test, run" and "Test Quality" sections of [../CLAUDE.md](../CLAUDE.md), which set the rules these tools follow.
> **You'll be able to:** run any test suite the safe way with one command, prove a new test can fail, and check a floor-test command starts before anyone runs it on the car.
> **Time:** 5 minutes.

Three small tools in `tools/dev/` that do, in one command each, what this repository's rules otherwise ask you to do by hand.

## Highlights

- **`rostest.sh` runs pytest the safe way.** It sources ROS, puts the tests on their own ROS domain (79, localhost only), and refuses domain 0, the car's own. Tests that start driving programs then can't reach the real car.
- **It tests the code in front of you.** From a git worktree it puts that worktree's `src/` ahead of the installed packages, so you never test last week's build by mistake.
- **`mutation_check.py` automates Hard rule 1.** It breaks the code under each test, one break at a time, runs the test, and restores the file whatever happens, Ctrl+C included. On 2026-10-31 it ran 92 breaks across the new tools; two were missed at first, and each miss led to a missing test.
- **A syntax error is not a catch.** It only counts a break as caught when a test fails. If the break stops the tests from loading at all, it says so.
- **`node_start_check.sh` catches a bad command before the floor does.** It starts a [node](glossary.md#node) (one running program) for a few seconds on the isolated domain and shows how it ended. It refuses hardware drivers, because a ROS domain does not separate a serial port.

## Running tests

**Any terminal:**

```bash
cd ~/racerbot-ws
tools/dev/rostest.sh src/gap_follow/test/ -q
tools/dev/rostest.sh tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ tools/dev/tests/ -q
```

**Working when:** pytest ends with `N passed`. Compare N with the count you expect: a test file that fails to import is a loud error here, never a silent skip.

`ROSTEST_DOMAIN=80 tools/dev/rostest.sh ...` picks another isolated domain, for two suites at once.

## Proving a test can fail

Write the breaks in a YAML file. Each break swaps one exact piece of source for a broken version:

```yaml
- name: lag sign flipped
  file: tools/run_analysis/car_measure.py
  old: "np.interp(np.asarray(t) - lag, cmd_t, cmd)"
  new: "np.interp(np.asarray(t) + lag, cmd_t, cmd)"
  test: tools/run_analysis/tests/test_car_measure.py
```

**Any terminal:**

```bash
python3 tools/dev/mutation_check.py my_breaks.yaml
```

**Working when:** every line reads `caught`, and the last line is `N of N caught`.

**If it doesn't:** `MISSED` means every test still passed with the code broken, so the test is not checking that. Fix the test, not the break. `NOT APPLIED` means the `old` text is not in the file exactly once. `ERROR` means the break stopped the tests loading; pick a break that still runs.

Use the break kinds CLAUDE.md lists: return the expected constant, flip a comparison, negate a sign, change a constant by 10%, delete a clamp. The specs used for the tools of 2026-10-31 are in `tools/dev/breaks/`. Rerun them after changing those tools.

<details>
<summary>What it guarantees about your files (safe to skip)</summary>

Before a file is broken, a copy goes to `.mutation_backup/` in the repository root. The original is written back in a `finally` block, and its SHA-256 is compared with the one taken before. If they differ, the tool stops and names the backup.

Never run two of these, or a test suite, against the same files at once. A test run that starts while a file is broken tests the broken file.

</details>

## Checking a command starts

**Any terminal:**

```bash
tools/dev/node_start_check.sh 6 ros2 run gap_follow gap_follow_node --ros-args \
    --params-file src/gap_follow/config/gap_follow.yaml -p max_steering_angle:=0.436
```

**Working when:** it prints the node's last log lines, then `still running after 6 s (started fine)`. A `gap_follow` node shows `ready: ... deadman ENABLED (LB must be held)`.

**If it doesn't:** `exited by itself` with the node's own reason above it, such as `aim_depth_bias must be within [0, 1]`.

It cannot start bringup, the VESC (motor controller), the [LiDAR](glossary.md#lidar) (laser scanner), the joystick or the camera, and refuses if you try. Those open real devices, which the car's own stack is using.

## Safety

These tools change nothing on the car and never join its ROS graph. A driving node started by `node_start_check.sh` sits on domain 79 with no `/joy`, so its LB deadman holds it at zero. Never set `ROSTEST_DOMAIN=0`: the tools refuse it, because a test on the car's graph once published to the real `/drive` (2026-09-27).
