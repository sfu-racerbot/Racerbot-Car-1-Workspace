#!/usr/bin/env bash
# Commit and push this car's runs in data/<car>/ (see data/README.md).
#
#   tools/save_runs.sh                 # commit + push this car's data
#   tools/save_runs.sh --no-push       # commit only
#   tools/save_runs.sh --skip-idle     # leave runs that never moved uncommitted
#
# Runs where the car never moved, and stubs replaced at once by another
# recorder (both flagged in auto_record.yaml), are listed before committing.
# They are committed unless --skip-idle is given. Nothing is ever deleted.
#
# Commits only data/<car>/ and data/README.md -- other workspace changes are left
# alone. Refuses, without committing anything, if:
#   * a recorder or mapping run is still writing (a half-written bag has no
#     metadata.yaml and cannot be replayed), or a finished run is still being
#     summarized into its auto_record.yaml, or
#   * any file in data/ is over GitHub's 100 MB limit, which would make the
#     push fail after the commit is already made.
set -euo pipefail

LIMIT_BYTES=$((100 * 1000 * 1000))
push=1
skip_idle=0
for arg in "$@"; do
    case "$arg" in
        --no-push) push=0 ;;
        --skip-idle) skip_idle=1 ;;
        *) echo "Unknown option: $arg (use --no-push, --skip-idle)" >&2; exit 2 ;;
    esac
done

ws="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ws"

car="$(python3 - "$ws" <<'PY'
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(sys.argv[1]) / 'src/racerbot_car'))
try:
    from racerbot_car.identity import current_car_id
    print(current_car_id())
except Exception as error:
    print(f"Cannot resolve car identity for RACERBOT_CAR={os.environ.get('RACERBOT_CAR', '')!r}: "
          f"{error}", file=sys.stderr)
    sys.exit(1)
PY
)" || exit 1
car_data="data/$car"
# Include tracked deletions even if the entire car folder has been removed.
paths=()
if [[ -d "$car_data" ]] || [[ -n "$(git ls-files -- "$car_data")" ]]; then
    paths+=("$car_data")
fi
if [[ -f data/README.md ]] || git ls-files --error-unmatch data/README.md >/dev/null 2>&1; then
    paths+=(data/README.md)
fi

if pgrep -f 'ros2 bag record' >/dev/null || pgrep -f auto_map_race_node >/dev/null; then
    echo "A run is still recording (ros2 bag record / auto_map_race_node)." >&2
    echo "Stop it (Ctrl+C in its terminal) and run this again." >&2
    exit 1
fi
if pgrep -f race_diagnostics.run_activity >/dev/null; then
    echo "A finished run is still being summarized (run_activity). Run this again in a moment." >&2
    exit 1
fi

too_big="$(find data -type f -size +"${LIMIT_BYTES}"c -printf '%s\t%p\n')"
if [[ -n "$too_big" ]]; then
    echo "These files are over GitHub's 100 MB limit and would make the push fail:" >&2
    echo "$too_big" >&2
    echo "Move them out of data/ (or set up Git LFS), then run this again." >&2
    exit 1
fi

# Only runs not committed yet. `moved: false` and `superseded: true` are exactly
# what auto_record.yaml says; a run with no such line is never treated as idle.
idle=()
while IFS= read -r info; do
    if grep -qE '^(  moved: false|superseded: true)$' "$info"; then
        idle+=("$(dirname "$info")")
    fi
done < <(git ls-files --others --exclude-standard -- "$car_data" | grep '/auto_record\.yaml$' || true)

if (( ! ${#paths[@]} )); then
    echo "No new runs in $car_data/ -- nothing to commit."
    exit 0
fi
git add -- "${paths[@]}"
if (( ${#idle[@]} )); then
    echo "Runs where the car never moved, or stubs:"
    printf '  %s\n' "${idle[@]}"
    if (( skip_idle )); then
        git reset -q -- "${idle[@]}"
        echo "Left uncommitted (--skip-idle). They are still on disk."
    else
        echo "Committing them too. To leave them out: tools/save_runs.sh --skip-idle"
    fi
fi
if git diff --cached --quiet -- "${paths[@]}"; then
    echo "No new runs in $car_data/ -- nothing to commit."
    exit 0
fi

git diff --cached --stat -- "${paths[@]}" | tail -1
git commit -q -m "data: runs saved $(date +%Y-%m-%d)" -- "${paths[@]}"
git log --oneline -1

if (( push )); then
    git push
fi
