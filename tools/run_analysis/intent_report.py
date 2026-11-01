#!/usr/bin/env python3
"""Why the car drove the speed it did, from what the driving node said it was doing.

    python3 tools/run_analysis/intent_report.py data/rb2/racerbot_runs/20261030-175728
    python3 tools/run_analysis/intent_report.py RUN --window 61.0-64.5 --every 0.1
    python3 tools/run_analysis/intent_report.py RUN --line MAP/raceline_raw.csv     # by course position

Reads /drive_intent (docs/drive-intent.md), which gap_follow and pure_pursuit publish
each tick: the state, the commanded and desired speed and steering, and every speed
factor with the one that bound. While LB was held it reports:

- which factor bound, as a share of ticks;
- every stall: under 0.4 m/s and commanding under 0.4 m/s for 0.3 s or more, with what
  bound and how it ended (by itself, or by the driver releasing LB);
- every LB release and press, with the follow cap at the release and how long it stayed
  low after the press, which is how the false-mover stalls of 2026-10-30 were found;
- with --window, every decision in a time range; with --line, the same by course section
  (needs loc.py's track of the run).

Times are seconds from the start of the bag. Reads the bag; writes nothing.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path

import numpy as np

STALL_SPEED = 0.4       # m/s, both measured and commanded
STALL_MIN = 0.3         # s
LOW_CAP = 0.4           # m/s; a follow cap under this has the car effectively stopped


def parse(data: str) -> dict | None:
    """One /drive_intent message as a flat row, or None if it is not a drive intent."""
    try:
        d = json.loads(data)
    except (TypeError, ValueError):
        return None
    if not isinstance(d, dict) or "state" not in d:
        return None
    factors = d.get("factors") or []
    return dict(state=d["state"], severity=d.get("severity"),
                speed=float(d.get("commanded_speed", math.nan)), desired_speed=float(d.get("desired_speed", math.nan)),
                steering=float(d.get("commanded_steering", math.nan)),
                desired_steering=float(d.get("desired_steering", math.nan)),
                factors={f["name"]: f.get("value") for f in factors if "name" in f},
                binding=[f["name"] for f in factors if f.get("binding")], targets=d.get("targets"))


def limiter(row: dict) -> str:
    """What held the speed down this tick: the first binding factor, else the state."""
    return row["binding"][0] if row["binding"] else row["state"]


def shares(rows) -> dict:
    """Share of rows each limiter accounts for, largest first."""
    counts = collections.Counter(limiter(r) for r in rows)
    total = sum(counts.values())
    return {name: n / total for name, n in counts.most_common()} if total else {}


def stalls(events, speed: float = STALL_SPEED, shortest: float = STALL_MIN) -> list:
    """Stall episodes from time-ordered (kind, t, value) events.

    kinds: "odom" (measured speed), "joy" (LB held, bool), "intent" (a parse() row).
    An episode starts on an intent tick with LB held, measured and commanded speed under
    ``speed``. It ends by itself when the car measures ``speed`` again, or when LB is
    released. Episodes shorter than ``shortest`` are dropped.
    """
    out, current, odom, held = [], None, 0.0, False

    def close(how):
        if current and current["end"] - current["start"] >= shortest:
            current["ended"] = how
            out.append(current)

    for kind, t, value in events:
        if kind == "odom":
            odom = value
            if current is not None and odom >= speed:
                close("by itself")
                current = None
        elif kind == "joy":
            if held and not value and current is not None:
                close("LB released")
                current = None
            held = bool(value)
        elif kind == "intent" and held:
            if abs(odom) < speed and value["speed"] < speed:
                if current is None:
                    current = dict(start=t, end=t, limiters=collections.Counter(), states=collections.Counter())
                current["end"] = t
                current["limiters"][limiter(value)] += 1
                current["states"][value["state"]] += 1
    close("run end")
    return out


def lb_toggles(intent_t, follow_cap, joy_edges, low: float = LOW_CAP) -> list:
    """For each LB release followed by a press: the follow cap at the release, when it
    went low, and how long after the press it stayed low (nan if it never lifted)."""
    t, cap = np.asarray(intent_t, float), np.asarray(follow_cap, float)
    out = []
    for (released, r_held), (pressed, p_held) in zip(joy_edges, joy_edges[1:]):
        if r_held or not p_held:
            continue
        j = int(np.searchsorted(t, released)) - 1
        if j < 0:
            continue
        start = j
        while start > 0 and cap[start - 1] < low:
            start -= 1
        after = int(np.searchsorted(t, pressed))
        lifted = after
        while lifted < len(t) and cap[lifted] < low:
            lifted += 1
        out.append(dict(released=released, pressed=pressed, cap_at_release=float(cap[j]),
                        low_since=float(t[start]) if cap[j] < low else math.nan,
                        low_after_press=float(t[lifted] - pressed) if lifted < len(t) else math.nan))
    return out


def read_events(run):
    """(kind, t, value) events and the intent rows with their times."""
    from run_analysis.bagread import messages
    events, rows, t0 = [], [], None
    for topic, stamp, m in messages(run, ["/joy", "/odom", "/drive_intent"]):
        t0 = stamp if t0 is None else t0
        t = stamp - t0
        if topic == "/odom":
            events.append(("odom", t, m.twist.twist.linear.x))
        elif topic == "/joy":
            events.append(("joy", t, bool(len(m.buttons) > 4 and m.buttons[4])))
        else:
            row = parse(m.data)
            if row is not None:
                events.append(("intent", t, row))
                rows.append((t, stamp, row))
    return events, rows


def held_rows(events, rows):
    """Intent rows while LB was held."""
    held, out = False, []
    for kind, t, value in events:
        if kind == "joy":
            held = value
        elif kind == "intent" and held:
            out.append(value)
    return out


def by_section(run, rows, line, loc_dir, section: float = 2.5) -> list:
    """Per course section: true speed, commanded and desired speed, |steering| and limiters."""
    from run_analysis.laps import Course
    from run_analysis.loc import COL, load_line, load_track
    from run_analysis.sectors import true_speed
    track = load_track(run, loc_dir)
    course = Course(load_line(line))
    tt = track[:, COL["t"]]
    tv = true_speed(tt, track[:, COL["x"]], track[:, COL["y"]])
    drive = [(stamp, r) for _t, stamp, r in rows if r["severity"] in (None, "drive")]
    if not drive:
        return []
    stamps = np.array([s for s, _ in drive])
    x, y = np.interp(stamps, tt, track[:, COL["x"]]), np.interp(stamps, tt, track[:, COL["y"]])
    arc, _ = course.project(x, y)
    speed = np.interp(stamps, tt, tv)
    out = []
    for q in range(int(course.length // section) + 1):
        k = np.flatnonzero((arc >= q * section) & (arc < (q + 1) * section))
        if len(k) < 3:
            continue
        rs = [drive[i][1] for i in k]
        out.append(dict(at=q * section, n=len(k), true_speed=float(np.median(speed[k])),
                        speed=float(np.median([r["speed"] for r in rs])),
                        desired=float(np.median([r["desired_speed"] for r in rs])),
                        steering=float(np.median([abs(r["steering"]) for r in rs])),
                        limiters=shares(rs)))
    return out


def main(argv=None) -> int:
    from run_analysis.loc import DEFAULT_OUT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--window", action="append", default=[], help="start-end seconds to print every decision in")
    ap.add_argument("--every", type=float, default=0.0, help="with --window: at most one line per this many s")
    ap.add_argument("--line", help="course line CSV: also report by section (needs loc.py)")
    ap.add_argument("--loc", default=str(DEFAULT_OUT))
    args = ap.parse_args(argv)
    for run in args.runs:
        events, rows = read_events(run)
        print(f"== {Path(run).name}: {len(rows)} intents")
        if not rows:
            print("   no /drive_intent in this run")
            continue
        held = held_rows(events, rows)
        print("   held speed down: " + ", ".join(f"{k} {v:.0%}" for k, v in list(shares(held).items())[:8]))
        for s in stalls(events):
            top = ", ".join(f"{k} {n}" for k, n in s["limiters"].most_common(3))
            print(f"   stall {s['start']:7.2f}-{s['end']:7.2f} ({s['end'] - s['start']:.2f} s), ended {s['ended']}: {top}")
        edges, last = [], None
        for kind, t, value in events:
            if kind == "joy" and value != last:
                if last is not None:
                    edges.append((t, value))
                last = value
        cap = [r["factors"].get("follow cap", math.inf) for _t, _s, r in rows]
        cap = [math.inf if c is None else float(c) for c in cap]
        for g in lb_toggles([t for t, _s, _r in rows], cap, edges):
            print(f"   LB off {g['released']:7.2f}, on {g['pressed'] - g['released']:.2f} s later; follow cap at release "
                  f"{g['cap_at_release']:.2f}, low since {g['low_since']:.2f}, still low {g['low_after_press']:.2f} s after the press")
        for window in args.window:
            a, b = (float(v) for v in window.split("-"))
            last = -math.inf
            for t, _stamp, r in rows:
                if a <= t <= b and t - last >= args.every:
                    last = t
                    print(f"   {t:7.2f} {r['state']:16s} cmd {r['speed']:4.2f} want {r['desired_speed']:4.2f} "
                          f"steer {r['steering']:+.3f} bound {','.join(r['binding']) or '-'} "
                          + " ".join(f"{k}={v:.2f}" for k, v in r["factors"].items() if isinstance(v, (int, float))))
        if args.line:
            for s in by_section(run, rows, args.line, args.loc):
                lim = " ".join(f"{k}:{v:.0%}" for k, v in list(s["limiters"].items())[:3])
                print(f"   {s['at']:5.1f} m  true {s['true_speed']:.2f}  cmd {s['speed']:.2f}  want {s['desired']:.2f}  "
                      f"|steer| {s['steering']:.3f}  {lim}")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
