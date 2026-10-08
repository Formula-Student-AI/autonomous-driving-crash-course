#!/usr/bin/env bash
# Build the simulator once, when the dev container is created.
#
# This is the only time anyone builds the simulator automatically. If you change
# the simulator later, rebuild it yourself; see sim/README.md for how to rebuild
# only what you changed.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash

# Most of us commit from Windows, where git does not track the executable bit
# reliably. Make sure the helper is runnable rather than relying on it.
chmod +x tools/bri 2>/dev/null || true

echo "Building the simulator (sim/). This takes a few minutes the first time."
cd sim
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "Simulator built. New terminals will pick it up automatically."
echo "Launch it with:  ros2 launch eufs_sim2 eufs_sim2.launch.py   (or: bri sim)"
