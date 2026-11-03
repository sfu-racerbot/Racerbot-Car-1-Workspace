#!/usr/bin/env bash
# Reproduce every number in docs/rb1-lap-gap-2026-10-30.md, in order. Read-only on data/.
# On the dev box prefix Python with `rb` (RB=rb bash run_all.sh); on a car, source ROS first.
# Simulations use 2 workers (shared machine). Takes about 15 minutes.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)
LOC=${LAPGAP_LOC:-$HOME/.cache/racerbot/loc}
CACHE=${LAPGAP_CACHE:-$HOME/.cache/racerbot/lapgap}
# rb forwards only argv into its container, so the paths travel as an env prefix.
PY="${RB:+$RB }env LAPGAP_LOC=$LOC LAPGAP_CACHE=$CACHE python3"
M=$WS/data/rb2/racerbot_auto/20261030-185036-fixed
R1=$WS/data/rb1/racerbot_runs
R2=$WS/data/rb2/racerbot_runs
CAR1="$R1/20261030-175418 $R1/20261030-175701 $R1/20261030-180531 $R1/20261030-180746 $R1/20261030-190734"
CAR2="$R2/20261030-175124 $R2/20261030-175728 $R2/20261030-181003 $R2/20261030-182852 $R2/20261030-183134 $R2/20261030-184556 $R2/20261030-184826 $R2/20261030-185036"
names() { for r in "$@"; do basename "$r"; done; }

echo "== 1. locate every run on car 2's map (fit: >= 0.9 of beams within 10 cm)"
$PY "$WS/tools/run_analysis/loc.py" $CAR1 $CAR2 --map "$M" --out "$LOC" --workers 2
echo "== 2. true lap times"
$PY "$WS/tools/run_analysis/laps.py" $CAR1 $CAR2 --line "$M/raceline_raw.csv" --loc "$LOC"
echo "== 3. command realisation and the gain that delivers the command"
$PY "$WS/tools/run_analysis/speed_gain.py" $CAR1 --loc "$LOC"
$PY "$WS/tools/run_analysis/speed_gain.py" "$R1/20261030-180531" "$R1/20261030-190734" --loc "$LOC"
$PY "$WS/tools/run_analysis/speed_gain.py" "$R2/20261030-181003" "$R2/20261030-183134" "$R2/20261030-184556" "$R2/20261030-184826" "$R2/20261030-185036" --loc "$LOC"
echo "== 4. bag extracts, then true gain against speed on straights"
$PY "$HERE/extract.py" $CAR1 $CAR2
$PY "$HERE/speedgain.py" $(names $CAR1) >/dev/null
$PY "$HERE/speedgain.py" $(names $CAR2) >/dev/null
$PY "$HERE/gainbins.py" 20261030-175418,20261030-175701,20261030-180531,20261030-180746,20261030-190734 \
    20261030-181003,20261030-183134,20261030-184556,20261030-184826,20261030-185036,20261030-175728
echo "== 5. solo or two cars on track"
$PY "$HERE/holds.py" $(for r in $CAR1; do echo "rb1:$(basename "$r")"; done) $(for r in $CAR2; do echo "rb2:$(basename "$r")"; done)
echo "== 6. scan and drive timing"
$PY "$HERE/latency.py" $(names $CAR1)
$PY "$HERE/latency.py" 20261030-175728 20261030-181003 20261030-183134 20261030-184556 20261030-184826 20261030-185036
echo "== 7. composite flying lap per sector, car 1 solo against car 2 map_course"
$PY "$HERE/sectime.py" rb1solo:20261030-180531,20261030-190734 rb2map:20261030-184556,20261030-184826,20261030-185036
echo "== 8. which cap bound"
for r in "$R1/20261030-180531" "$R2/20261030-185036" "$R2/20261030-181003"; do
    $PY "$WS/tools/run_analysis/intent_report.py" "$r" | head -3
done
echo "== 9. simulator: lap time against command realisation, damping on and off"
$PY "$HERE/sim_gain.py" asraced=0.79,1.16 lead4040=0.874,1.16 fixed=1.0,1.16 --seeds 6 --workers 2 | grep '=='
$PY "$HERE/sim_gain.py" asraced_nodamp=0.79,1.16 fixed_nodamp=1.0,1.16 --set enable_steering_damping=false --seeds 6 --workers 2 | grep '=='
echo "== 10. simulator: time per sector, as raced against fixed"
$PY "$HERE/sim_sectors.py"
