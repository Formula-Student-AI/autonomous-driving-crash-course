# sim/: the simulator

A minimal copy of the Bristol FSAI simulator ([bristol-core-sim2](https://github.com/Formula-Student-AI/bristol-core-sim2)), which is built on [eufs_sim2](https://gitlab.com/eufs/public/eufs_sim2) from Edinburgh University Formula Student. It contains only what is needed to run the simulated car and look at it in Foxglove. Course material lives in the numbered folders at the repository root; this folder is infrastructure.

This is a ROS 2 workspace: the packages are in `src/`, and `build/`, `install/` and `log/` are created here when you build (they are git-ignored).

## Packages

| Package | What it is |
|---|---|
| `eufs_sim2` | The simulator: vehicle motion, state machine, emulated sensors |
| `eufs_msgs` | Message and service types (`SetMission`, cone messages, ...) |
| `vehicle_models` | Vehicle dynamics models |
| `state_lib` | Vehicle state library |
| `map_lib` | Cone map and track library, including the competition tracks |
| `eufs_logger`, `eufs_gmock_matchers` | Logging and test helpers that the libraries above depend on |
| `eufs_sim_foxglove_plugins` | TypeScript source of the Foxglove extension that draws EUFS cone messages. The **built extension students install** ships as [`0-environment-setup/eufs-foxglove-extension.foxe`](../0-environment-setup/eufs-foxglove-extension.foxe), next to the guide that tells them to drag it onto Foxglove. To change the extension, edit here, run `npm install && npm run package`, and copy the resulting `.foxe` over that file. |

## Where it came from

- Starting point: `bristol-core-sim2`, branch `yn/slam-lib-integration`, commit `1ff527d`. **This folder is now the home of the simulator.** `bristol-core-sim2` is not kept in sync; code is ported from it by hand when needed.
- Copied from git (tracked files only), so no build output is included.

## What was removed, and why

- **Autonomy and tooling packages:** `slam`, `slam_testing`, `planning`, `planning_control`, `safety`, `noise_injection`, `ros_can`, `perception`, the ZED wrapper, the top-level launch files, the `bri` CLI and `tools/`. Students write these parts themselves in the course modules.
- **Eight example drivers in `eufs_sim2`** (`simple_cone_driver`, `spline_driver`, `stanley_cone_driver`, `slomoi_mpc_driver`, `slomoi_v2_driver`, `cone_balance_driver`, `cone_tracker_cone_driver`, `centerline_cone_driver`) and `docs/driver-testing-notes.md`. All were added by Bristol after the initial import (not part of upstream), and complete drivers would give away the answers to the Planning & Control module. Their `CMakeLists.txt` entries are removed too.
- **Three demo GIFs** (14 MB) in `eufs_sim2/docs/images`, and the lines in the docs that embedded them.

## How it is built

The dev container image ([`.devcontainer/Dockerfile`](../.devcontainer/Dockerfile)) installs these packages' system dependencies with `rosdep`. The simulator itself is compiled **once, when the container is created**, by [`.devcontainer/build-sim.sh`](../.devcontainer/build-sim.sh), which runs `colcon build` here. The result lands in `sim/install`, and every shell in the container sources it automatically.

So there is only one copy of the source: this one. Editing it and rebuilding works the way it does in any ROS workspace.

## Fixes made during the import

- `map_lib/package.xml`: added the missing `pybind11_vendor` dependency. Its `CMakeLists.txt` requires it, but it was undeclared, so the build only worked because the old container installed the whole `ros-humble-desktop` metapackage.

## Changes Bristol made to eufs_sim2 that are kept

- `control_input` plugin: the "deadman" (the car brakes if `/cmd` goes quiet for about 0.3 s), clamping of commands and rejection of NaN values.
- `twist_publisher`: angular velocity axes corrected to ROS conventions (yaw on `angular.z`).
- LiDAR emulation disabled by default in `config/plugin_params.yaml`.

## Changing it

Change the code here directly, then rebuild. A full `colcon build` recompiles all seven packages, which is rarely what you want, so rebuild only what is affected:

```bash
cd sim
source /opt/ros/humble/setup.bash

# You edited one package and nothing depends on it (e.g. eufs_sim2 itself):
colcon build --symlink-install --packages-select eufs_sim2

# You edited a library that other packages use (e.g. state_lib, map_lib, eufs_msgs):
# rebuild it AND everything that depends on it, or you get mismatched binaries.
colcon build --symlink-install --packages-above state_lib

source install/setup.bash   # in each terminal you want to use it from
```

| Flag | Rebuilds |
|---|---|
| `--packages-select X` | only `X` |
| `--packages-above X` | `X` and everything that depends on it |
| `--packages-up-to X` | `X` and everything it depends on |

Two cases need more than a rebuild:

- **You added a dependency to a `package.xml`.** Its system package is not in the image yet, so run **Dev Containers: Rebuild Container** in VS Code.
- **A build fails in a confusing way after large changes.** Delete the stale state with `rm -rf build install log` and build again.

If you port something in from `bristol-core-sim2` or from upstream `eufs_sim2`, say where it came from in your commit message.
