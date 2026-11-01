#!/usr/bin/env python3
"""Print a sweep2.py results file: per track, then totals.

    python3 tools/f1tenth_sim/summ2.py results.jsonl

A run that finishes without contact is a pass. A run that does not finish may still
have touched nothing (it stopped), so read the contact column as well as the passes.
"""

from __future__ import annotations

import argparse
import json


def read_rows(path) -> list:
    with open(path) as source:
        return [json.loads(line) for line in source if line.strip()]


def in_order(values) -> list:
    """Distinct values, first seen first."""
    return list(dict.fromkeys(values))


def totals(rows: list) -> list:
    """(candidate, friction, passed, total, contact runs), one per candidate and friction."""
    keys = in_order((r["cand"], r["mu"]) for r in rows)
    out = []
    for cand, mu in keys:
        chosen = [r for r in rows if (r["cand"], r["mu"]) == (cand, mu)]
        out.append((cand, mu, sum(r["passed"] for r in chosen), sum(r["total"] for r in chosen),
                    sum(r["contact"] for r in chosen)))
    return out


def format_row(row: dict) -> str:
    lap = "    -" if row["lap"] is None else f"{row['lap']:6.2f}"
    return (f"   {row['cand']:12} friction {row['mu']}  {row['passed']:3d}/{row['total']:<3d} lap {lap}  "
            f"contact {row['contact']:2d}  clearance {row['minclear']}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results", help="the file sweep2.py wrote")
    args = ap.parse_args(argv)
    rows = read_rows(args.results)
    for track in in_order(r["track"] for r in rows):
        print(track)
        for row in rows:
            if row["track"] == track:
                print(format_row(row))
    print("TOTAL")
    for cand, mu, passed, total, contact in totals(rows):
        print(f"   {cand:12} friction {mu}  {passed:3d}/{total:<3d} runs with contact {contact}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
