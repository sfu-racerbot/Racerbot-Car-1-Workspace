#!/usr/bin/env python3
"""Hard rule 1 of CLAUDE.md, automated: break the code under each test and see it fail.

    python3 tools/dev/mutation_check.py breaks.yaml
    python3 tools/dev/mutation_check.py breaks.yaml --only "sign flipped"

breaks.yaml is a list of breaks, each replacing one exact piece of source:

    - name: lag sign flipped
      file: tools/run_analysis/car_measure.py
      old: "np.interp(np.asarray(t) - lag, cmd_t, cmd)"
      new: "np.interp(np.asarray(t) + lag, cmd_t, cmd)"
      test: tools/run_analysis/tests/test_car_measure.py

For each, ``old`` must occur exactly once. It is swapped in, the test is run, and the
file is put back whatever happens, including Ctrl+C. A break counts as caught only when
pytest reports a failing test (exit 1). A break that stops the tests being collected at
all (a syntax error, exit 2) is reported as an error: that proves nothing about the test.

Writes nothing else. Each original is also copied to --backup before it is touched, so a
killed process can never lose a file; the copies are deleted at the end.
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUNNER = f"bash {ROOT / 'tools' / 'dev' / 'rostest.sh'} -x -q"


@dataclass
class Break:
    name: str
    file: str
    old: str
    new: str
    test: str


@dataclass
class Outcome:
    name: str
    verdict: str        # caught, MISSED, NOT APPLIED, ERROR
    detail: str = ""


def load(path) -> list:
    """Breaks from a YAML list; every field is required."""
    import yaml
    rows = yaml.safe_load(Path(path).read_text()) or []
    out = []
    for i, row in enumerate(rows):
        missing = [k for k in ("name", "file", "old", "new", "test") if k not in (row or {})]
        if missing:
            raise ValueError(f"break {i} is missing {', '.join(missing)}")
        out.append(Break(**{k: str(row[k]) for k in ("name", "file", "old", "new", "test")}))
    return out


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_one(item: Break, runner: str, root: Path, backup: Path) -> Outcome:
    """Apply one break, run its test, restore the file, and say what happened."""
    target = root / item.file
    original = target.read_bytes()
    count = original.decode().count(item.old)
    if count != 1:
        return Outcome(item.name, "NOT APPLIED", f"old text found {count} times")
    before = _digest(target)
    backup.mkdir(parents=True, exist_ok=True)
    saved = backup / (item.file.replace("/", "__"))
    shutil.copy2(target, saved)
    try:
        target.write_text(original.decode().replace(item.old, item.new, 1))
        done = subprocess.run(f"{runner} {item.test}", shell=True, cwd=root, capture_output=True, text=True)
    finally:
        target.write_bytes(original)
    if _digest(target) != before:
        raise RuntimeError(f"{item.file} did not restore; the original is at {saved}")
    saved.unlink()
    if done.returncode == 1:
        return Outcome(item.name, "caught")
    if done.returncode == 0:
        return Outcome(item.name, "MISSED", "every test passed with the break in place")
    return Outcome(item.name, "ERROR", f"pytest exit {done.returncode}: {(done.stdout + done.stderr).strip()[-200:]}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("spec", help="YAML list of breaks")
    ap.add_argument("--only", help="run only breaks whose name contains this")
    ap.add_argument("--runner", default=DEFAULT_RUNNER, help="command the test path is appended to")
    ap.add_argument("--root", default=str(ROOT), help="paths in the spec are relative to this")
    ap.add_argument("--backup", default=None, help="where originals are copied while broken")
    args = ap.parse_args(argv)
    root = Path(args.root)
    backup = Path(args.backup) if args.backup else root / ".mutation_backup"
    breaks = [b for b in load(args.spec) if not args.only or args.only in b.name]
    problems = 0
    for item in breaks:
        outcome = run_one(item, args.runner, root, backup)
        problems += outcome.verdict != "caught"
        print(f"{outcome.verdict:12s} {outcome.name}" + (f"  ({outcome.detail})" if outcome.detail else ""), flush=True)
    if backup.exists() and not any(backup.iterdir()):
        backup.rmdir()
    print(f"{len(breaks) - problems} of {len(breaks)} caught")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
