#!/usr/bin/env bash
# Stop the fruit-fly fun mode in the container (mux + fly_bridge).
# The Windows flyscreen and WSL relay are separate; Ctrl-C those windows.
#
#   bash fruitfly/tools/stop_fly_mode.sh
set -eo pipefail

PATS=(
    "fake run/cmd_mux.py"
    "fly_bridge"
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
    echo "no fly-mode processes found"
else
    echo "fly mode stopped in the container; flyscreen/relay are Windows-side"
fi
