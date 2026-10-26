#!/usr/bin/env bash
# Main race mode: scenery + command mux (race) + fixed-path driver.
#
#   bash "fake run/start_race_mode.sh"
#
# Start the simulator and /set_mission + /go first (see fake run/README.md).
# The fruit-fly fun mode is a separate script and must not be run at the same
# time; the mux (default mode=race) is the only /cmd publisher.
set -eo pipefail

ROOT="${FSAI_WS:-/workspaces/autonomous-driving-crash-course}"
source /opt/ros/humble/setup.bash
source "$ROOT/sim/install/setup.bash"

# Refuse to start a second controller or mux. A duplicate driver would fight
# the first one on /race/cmd; a stale mux may be in the wrong mode.
for pat in "fake run/fixed_path_driver.py" "fake run/cmd_mux.py"; do
    if pgrep -f "$pat" >/dev/null 2>&1; then
        echo "[race mode] '$pat' is already running."
        echo "  stop it first: bash '$ROOT/fake run/stop_race_mode.sh'"
        exit 1
    fi
done

# Optional custom track (from track_editor.py). All nodes read the same files.
COMMON_ARGS=()
DRIVER_ARGS=()
if [ -n "${TRACK_FILE:-}" ]; then
    COMMON_ARGS+=(--ros-args -p "track_file:=$TRACK_FILE")
    DRIVER_ARGS+=(--ros-args -p "track_file:=$TRACK_FILE")
fi
if [ -n "${CENTERLINE_FILE:-}" ]; then
    COMMON_ARGS+=(-p "centerline_file:=$CENTERLINE_FILE")
    DRIVER_ARGS+=(-p "path_file:=$CENTERLINE_FILE")
fi

python3 "$ROOT/fake run/track_scenery.py" "${COMMON_ARGS[@]}" &
SCENERY=$!
python3 "$ROOT/fake run/trail_recorder.py" "${COMMON_ARGS[@]}" &
TRAIL=$!
python3 "$ROOT/fake run/cmd_mux.py" --ros-args -p mode:=race &
MUX=$!
CAM=""
if [ "${RACE_CAMERA:-1}" = "1" ]; then
    python3 "$ROOT/fake run/track_camera.py" "${COMMON_ARGS[@]}" &
    CAM=$!
fi
cleanup() {
    kill "$SCENERY" "$TRAIL" "$MUX" ${CAM:-} 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[race mode] scenery pid=$SCENERY  trail pid=$TRAIL  mux pid=$MUX  camera=${CAM:-off}  driver follows"
python3 "$ROOT/fake run/fixed_path_driver.py" "${DRIVER_ARGS[@]}" "$@"
