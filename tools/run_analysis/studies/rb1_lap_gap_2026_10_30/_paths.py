"""Where the study's inputs and caches live. Nothing is written into data/.

LAPGAP_LOC   loc.py's results (default ~/.cache/racerbot/loc, loc.py's own default)
LAPGAP_CACHE this study's bag extracts and per-run arrays (default ~/.cache/racerbot/lapgap)
"""
import os
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[4]
LOC = Path(os.environ.get("LAPGAP_LOC", "~/.cache/racerbot/loc")).expanduser()
CACHE = Path(os.environ.get("LAPGAP_CACHE", "~/.cache/racerbot/lapgap")).expanduser()
LINE = WS / "data/rb2/racerbot_auto/20261030-185036-fixed/raceline_raw.csv"
CACHE.mkdir(parents=True, exist_ok=True)
for path in (WS / "tools", WS / "tools/f1tenth_sim"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
