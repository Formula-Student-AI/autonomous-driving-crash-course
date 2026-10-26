#!/usr/bin/env bash
# Fruit-fly FUN mode: command mux (fly) + fly bridge.
#
#   bash fruitfly/tools/start_fly_mode.sh
#
# This is a separate activity. It publishes /fly/cmd; the mux forwards it to
# /cmd only in fly mode. Do NOT run the race driver at the same time.
# The Windows side (flyscreen live + WSL relay) must already be running; see
# fruitfly/README.md. Start the simulator and /set_mission + /go first.
set -eo pipefail

ROOT="${FSAI_WS:-/workspaces/autonomous-driving-crash-course}"

# Race and fly are separate modes. Do not take over while the race stack is
# running; stop it first (bash "fake run/stop_race_mode.sh").
for pat in "fake run/fixed_path_driver.py" "fake run/cmd_mux.py"; do
    if pgrep -f "$pat" >/dev/null 2>&1; then
        echo "[fly mode] '$pat' is already running."
        echo "  stop the race stack first: bash '$ROOT/fake run/stop_race_mode.sh'"
        exit 1
    fi
done

source /opt/ros/humble/setup.bash
source "$ROOT/sim/install/setup.bash"
source "$ROOT/fruitfly/ros2_ws/install/setup.bash"

python3 "$ROOT/fake run/cmd_mux.py" --ros-args -p mode:=fly &
MUX=$!
cleanup() {
    kill "$MUX" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[fly mode] mux pid=$MUX  -> /fly/cmd is now selected"
ros2 launch fly_bridge fly_bridge.launch.py "$@"
