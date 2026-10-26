#!/usr/bin/env bash
# Stop the race stack (scenery, trail, mux, camera, driver). Leaves eufs_sim2
# running, so the car simply gets the simulator's deadman brake.
#
#   bash "fake run/stop_race_mode.sh"
set -eo pipefail

PATS=(
    "fake run/fixed_path_driver.py"
    "fake run/cmd_mux.py"
    "fake run/track_scenery.py"
    "fake run/trail_recorder.py"
    "fake run/track_camera.py"
    "fake run/publish_centerline_markers.py"
)

stopped=0
for pat in "${PATS[@]}"; do
    pids=$(pgrep -f "$pat" || true)
    if [ -n "$pids" ]; then
        kill $pids 2>/dev/null || true
        stopped=$((stopped + 1))
    fi
done

sleep 1
for pat in "${PATS[@]}"; do
    pids=$(pgrep -f "$pat" || true)
    if [ -n "$pids" ]; then
        kill -9 $pids 2>/dev/null || true
    fi
done

if [ "$stopped" -eq 0 ]; then
    echo "no race processes found"
else
    echo "race stack stopped; eufs_sim2 was not touched"
fi
