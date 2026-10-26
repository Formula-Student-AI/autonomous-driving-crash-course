#!/usr/bin/env bash
# One-command race mode inside the dev container.
#
#   bash "fake run/start_race.sh"                          # foreground
#   bash "fake run/start_race.sh" --detach                 # keep running after the terminal closes
#   bash "fake run/start_race.sh" --foxglove               # also start Foxglove bridge :8765
#   bash "fake run/start_race.sh" --stop                   # stop race stack, leave sim
#   bash "fake run/start_race.sh" --track  "fake run/track_editor.csv" \
#                                 --centerline "fake run/centerline_editor.csv"
#
# It starts eufs_sim2 if needed, waits for /odom, loads a custom track (if
# given), calls /set_mission {4} + /go, then runs the race stack (scenery +
# trail + camera + mux + driver). Ctrl-C stops the stack but leaves the sim.
set -eo pipefail

ROOT="${FSAI_WS:-/workspaces/autonomous-driving-crash-course}"
FOX=0
DETACH=0
TRACK_ARG=""
CENTER_ARG=""
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --foxglove) FOX=1; shift ;;
        --detach) DETACH=1; shift ;;
        --stop) exec bash "$ROOT/fake run/stop_race_mode.sh" ;;
        --track) TRACK_ARG="$2"; shift 2 ;;
        --centerline) CENTER_ARG="$2"; shift 2 ;;
        --) shift; ARGS+=("$@"); break ;;
        *) ARGS+=("$1"); shift ;;
    esac
done

# Relative paths are relative to the workspace root (the container mount).
if [ -n "$TRACK_ARG" ]; then
    case "$TRACK_ARG" in /*) ;; *) TRACK_ARG="$ROOT/$TRACK_ARG" ;; esac
    export TRACK_FILE="$TRACK_ARG"
fi
if [ -n "$CENTER_ARG" ]; then
    case "$CENTER_ARG" in /*) ;; *) CENTER_ARG="$ROOT/$CENTER_ARG" ;; esac
    export CENTERLINE_FILE="$CENTER_ARG"
elif [ -n "${TRACK_FILE:-}" ] && [ -f "$ROOT/fake run/centerline_editor.csv" ]; then
    export CENTERLINE_FILE="$ROOT/fake run/centerline_editor.csv"
fi

source /opt/ros/humble/setup.bash
source "$ROOT/sim/install/setup.bash"

sim_up() {
    ros2 node list 2>/dev/null | grep -qx /eufs_sim2
}

if sim_up; then
    echo "[race] eufs_sim2 already running"
else
    echo "[race] starting eufs_sim2 ..."
    nohup ros2 launch eufs_sim2 eufs_sim2.launch.py > /tmp/sim.log 2>&1 &
    for _ in $(seq 1 40); do
        sleep 1
        if ros2 topic info /odom 2>/dev/null | grep -q "Publisher count: 1"; then
            break
        fi
    done
    sleep 2
fi

if [ -n "${TRACK_FILE:-}" ]; then
    echo "[race] loading custom track: $TRACK_FILE"
    ros2 service call /eufs_sim2/file eufs_msgs/srv/SetString \
        "{data: \"$TRACK_FILE\"}" >/dev/null || true
    sleep 1
    echo "[race] resetting car to the start"
    ros2 service call /reset std_srvs/srv/Trigger "{}" >/dev/null || true
    sleep 1
fi

echo "[race] set mission 4 + go"
ros2 service call /set_mission eufs_msgs/srv/SetMission "{mission: 4}" >/dev/null || true
ros2 service call /go std_srvs/srv/Trigger "{}" >/dev/null || true

if [ "$FOX" = "1" ]; then
    if pgrep -f foxglove_bridge >/dev/null 2>&1; then
        echo "[race] foxglove_bridge already running (port 8765)"
    else
        nohup ros2 run foxglove_bridge foxglove_bridge \
            --ros-args -p port:=8765 > /tmp/foxglove.log 2>&1 &
        echo "[race] foxglove_bridge on ws://localhost:8765"
    fi
fi

if [ "$DETACH" = "1" ]; then
    LOG=/tmp/race_mode.log
    if command -v setsid >/dev/null 2>&1; then
        setsid bash "$ROOT/fake run/start_race_mode.sh" "${ARGS[@]}" \
            >"$LOG" 2>&1 &
    else
        nohup bash "$ROOT/fake run/start_race_mode.sh" "${ARGS[@]}" \
            >"$LOG" 2>&1 &
    fi
    echo $! > /tmp/race_mode.pid
    echo "[race] detached pid $(cat /tmp/race_mode.pid)  log $LOG"
    echo "[race] visuals: /track/scenery  /race/trail  /race/camera"
    echo "[race] stop: bash '$ROOT/fake run/start_race.sh' --stop"
    exit 0
fi

echo "[race] visuals: /track/scenery  /race/trail  /centerline_markers  /race/camera"
exec bash "$ROOT/fake run/start_race_mode.sh" "${ARGS[@]}"
