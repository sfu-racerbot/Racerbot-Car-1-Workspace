#!/usr/bin/env bash
set -euo pipefail
ws="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ws"
git config core.hooksPath tools/dev/hooks
python3 - "$ws" <<'PY'
from pathlib import Path
import sys

sys.path.insert(0, str(Path(sys.argv[1]) / 'src/racerbot_car'))
from racerbot_car.identity import current_car_id, UnknownCarError

try:
    print(current_car_id())
except UnknownCarError:
    print('dev box: set RACERBOT_CAR for the simulator')
PY
