# 0. Environment Setup

By the end of this module you will have the Bristol FSAI simulator running on your laptop, be watching the car in Foxglove, and have driven it by typing ROS commands yourself.

## What you are setting up

```
 Your laptop (Windows / macOS / Linux)
 ├── Docker Desktop ── runs a Linux container with ROS 2 Humble + the simulator
 ├── VS Code ───────── edits files inside that container (Dev Containers extension)
 └── Foxglove ──────── visualises what the simulator is doing (connects to port 8765)
```

You do not install ROS or Ubuntu on your laptop. Everything ROS-related lives in the container, so everyone gets the same environment.

The simulator lives in [`sim/`](../sim/) in this repository. It is **eufs_sim2**, originally from Edinburgh University Formula Student. It simulates the car's motion and the cones on a Formula Student track. It does **not** render camera images; instead it publishes the cones a perception system would have detected.

## Step 1: Install the tools

| Tool | Why | Notes |
|---|---|---|
| [Docker Desktop](https://www.docker.com/products/docker-desktop/) | Runs the container | **Windows:** use the WSL 2 backend (the default in current versions). If Docker asks you to enable WSL 2 or virtualisation, follow its prompts and restart. |
| [VS Code](https://code.visualstudio.com/) | Your editor, and how you work inside the container | |
| [Git](https://git-scm.com/downloads) | Gets the code | |
| [Foxglove](https://foxglove.dev/download) | Visualisation | Desktop app recommended. You may be asked to create a free account. |

**Check Docker:** open a terminal and run `docker run hello-world`. You should see "Hello from Docker!". If not, see [Troubleshooting](#troubleshooting).

### Step 1b: Install the Dev Containers extension

This is the piece that lets VS Code work *inside* Docker. Nothing after this point works without it, so do not skip it.

1. Open VS Code.
2. Open the Extensions view: the stacked-squares icon in the left sidebar, or `Ctrl/Cmd + Shift + X`.
3. Search for **Dev Containers**.
4. Install the one published by **Microsoft**, with the ID `ms-vscode-remote.remote-containers`. Several extensions have similar names, so check the publisher before clicking Install.

**Check:** a **`><`** icon appears in the very bottom-left corner of the VS Code window (the "Remote indicator"). That button is how you open and leave containers, and you will use it in Step 3.

## Step 2: Get the code

```bash
git clone https://github.com/Formula-Student-AI/autonomous-driving-crash-course.git
cd autonomous-driving-crash-course
```

> **Windows tip:** file access across the Windows/Linux boundary is slow. If things feel sluggish, clone inside WSL (`\\wsl$\Ubuntu\home\<you>`) rather than under `C:\`, and open that folder from VS Code.

## Step 3: Open it in the dev container

**1. Open the folder in VS Code.** Either:
- run `code .` in the terminal where you cloned it, or
- in VS Code, use **File → Open Folder…** and pick the `autonomous-driving-crash-course` folder.

**2. Reopen it inside the container.** Either:
- click the **`><` icon in the bottom-left corner** of the window and choose **Reopen in Container**, or
- press `Ctrl/Cmd + Shift + P` and run **Dev Containers: Reopen in Container**.

VS Code often offers this by itself in a notification in the bottom-right ("Folder contains a Dev Container configuration file…" → **Reopen in Container**). That button does the same thing.

**3. Wait.** The first time, Docker installs ROS 2 Humble and the simulator's dependencies, and then **compiles the simulator** for you. Expect **10–20 minutes**, and more on a slow connection. It happens once; later opens take seconds.

**Check:** that same bottom-left corner now reads **Dev Container: fsai-crash-course**. Open a terminal inside VS Code (`` Ctrl+` ``) and run:

```bash
ros2 pkg list | grep eufs
```

You should see `eufs_msgs` and `eufs_sim2`. That terminal is *inside* the container, and it already knows about ROS and the simulator: this is where every ROS command in this course runs.

> **Do I need to "source" anything?** **Not in this module.** Every new terminal in this container automatically sets up ROS and the simulator, so `ros2` commands work straight away in any terminal you open.
>
> That is only true for the simulator. In [Module 1](../1-introduction-to-ros/) you build a package of your own, and you *will* have to run `source install/setup.bash` in **every new terminal** before you can run it. Forgetting that is the single most common thing that trips people up, so it is worth knowing now that the two cases are different.

> **Why didn't I have to build anything?** The simulator is C++ and takes minutes to compile, so the container built it for you when it was created: the result is in `sim/install`, and every terminal picks it up automatically. In [Module 1](../1-introduction-to-ros/) you will build your *own* code with `colcon`, which is the part worth learning.
>
> If you later change the simulator, you rebuild it yourself; [`sim/README.md`](../sim/README.md#changing-it) shows how to rebuild only the parts you changed instead of all of it.

## Step 4: Launch the simulator

```bash
ros2 launch eufs_sim2 eufs_sim2.launch.py
```

You will see a stream of log lines ending with plugins reporting "initialised". Leave this terminal running: closing it stops the simulator.

Open a **second terminal** (`+` in the VS Code terminal panel) and look around:

```bash
ros2 topic list
```

You should see topics including `/cmd`, `/odom`, `/cones`, `/imu/data`, `/tf` and `/map`.

**Check:** `ros2 topic hz /odom` prints a steady rate. If it says the topic "does not appear to be published yet", the simulator is not running; look at the first terminal.

## Step 5: See it in Foxglove

Foxglove cannot read ROS topics directly. The **foxglove_bridge** is a separate ROS node that subscribes to topics and serves them over a WebSocket on port **8765**, and the simulator knows nothing about it. Start it in a second terminal, leaving the simulator running in the first:

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

(`bri bridge` is a shortcut for the same command; see the table below.)

It prints a line for each topic it advertises. Stop it with Ctrl+C and Foxglove loses the connection while the simulator keeps running.

**1. Connect.** Open Foxglove → **Open connection → Foxglove WebSocket**, enter `ws://localhost:8765` and connect.

**2. Add a 3D panel.** Click **+ Add panel** (top-left of the Foxglove window), then choose **3D**. On a fresh layout the panel starts empty and black: that is expected, because nothing is switched on yet.

**3. Switch on the car.** Everything a 3D panel draws is controlled from that panel's own settings, not from the main sidebar:

- Hover over the 3D panel and click its **gear / settings icon** (top-right of the panel). A settings list opens beside the panel.
- Find the **Topics** section. It lists every topic the panel is able to draw. Tick **`/robot_description`**: that is the car's 3D model.
- Frames from `/tf` are handled in their own **Transforms** section, listed by frame name (`map`, `base_footprint`, the `zed_*` camera frames) rather than as a topic called `/tf`. Expand it and make sure the frames are visible.
- If the car is off-screen, look for **Frame → Display frame** (sometimes "Follow frame") in the same settings and set it to `map`, then scroll to zoom out.

The topic list is long. There is a filter box at the top of the settings if you cannot spot a topic.

**Check:** you see the car model in the 3D panel.

> Foxglove's layout changes between versions, so the exact icon positions may differ from the above. What does not change: a 3D panel draws nothing until you tick topics in **that panel's** settings.

If the connection fails, check the **PORTS** tab in VS Code (next to the terminal): port 8765 should be listed. See [Troubleshooting](#troubleshooting).

### Install the EUFS Foxglove extension

Foxglove cannot draw the simulator's cone messages on its own, so an extension is included in this folder. Install it **on your host, in the Foxglove app** (not in the container):

1. Open this folder on your computer and find **[`eufs-foxglove-extension.foxe`](eufs-foxglove-extension.foxe)** (it sits next to this README).
2. Drag that file onto the Foxglove window.
3. Reload Foxglove.

You can now show coloured cones, and the Mission State and Joystick panels become available.

> If you cloned inside WSL, the file is reachable from Windows Explorer at
> `\\wsl$\Ubuntu\home\<you>\autonomous-driving-crash-course\0-environment-setup\`.

**Check:** tick **`/cones`** in the 3D panel's settings (the same **Topics** list you used for `/robot_description`) and you see blue and yellow cones around the car.

## Step 6: Drive the car with commands

The car is a state machine: it ignores commands until it is told which event to run and given the go-ahead. In your second terminal:

**1. Pick the event** (4 = Trackdrive; others: 1 Acceleration, 2 Skidpad, 3 Autocross):

```bash
ros2 service call /set_mission eufs_msgs/srv/SetMission "{mission: 4}"
```

**2. Send the GO signal:**

```bash
ros2 service call /go std_srvs/srv/Trigger "{}"
```

**3. Tell the car to accelerate:**

```bash
ros2 topic pub /cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {acceleration: 2.0, steering_angle: 0.0}}" --rate 10
```

**Check:** the car drives forward in Foxglove. Press `Ctrl+C` to stop publishing. Try `steering_angle: 0.2` to make it turn.

> `/cmd` takes an **acceleration** (m/s²) and a **steering angle** (radians); the `speed` field is ignored. If commands stop arriving for about 0.3 s the simulator brakes the car on purpose (a "deadman" safety), so keep a steady `--rate 10`.

Reset between runs:

```bash
ros2 service call /reset std_srvs/srv/Trigger "{}"
```

A mission can only be selected once per run of the simulator, so to start a fresh run, stop the simulator with `Ctrl+C` and launch it again.

**You just controlled a robot with three ROS commands.** In [Module 1](../1-introduction-to-ros/) you will find out what each of them actually does, and write code that does the same thing.

## A shortcut, once you know the long way

Restarting the simulator and re-sending mission and GO gets tedious fast, so the repository includes a small helper called `bri` ([`tools/bri`](../tools/bri)) that is already on your `PATH` in the container:

| Shortcut | What it actually runs |
|---|---|
| `bri sim` | `ros2 launch eufs_sim2 eufs_sim2.launch.py` |
| `bri bridge` | `ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765` |
| `bri go` | `/set_mission` (Trackdrive) followed by `/go` |
| `bri build` | `colcon build --symlink-install` in the workspace you are in |

It is one short shell script with no hidden behaviour: `bri help` prints the raw command behind each shortcut, and you can read the whole thing in a minute. Use the long commands while you are learning them, and `bri` once you are tired of typing them.

## Troubleshooting

**`docker run hello-world` fails, or Docker will not start.** On Windows, virtualisation must be enabled (a BIOS/UEFI setting) and WSL 2 installed. Docker Desktop reports the specific error; search for it together with "Docker Desktop WSL 2".

**The container build is very slow or runs out of memory.** Raise Docker's memory limit (Docker Desktop → Settings → Resources; 8 GB or more if your laptop has it) and close other heavy applications. The build compiles C++, so it uses every core you give it.

**`package 'eufs_sim2' not found`.** You are probably in a terminal on your host rather than inside the container. Check the bottom-left corner of VS Code says **Dev Container**.

**Foxglove cannot connect to `ws://localhost:8765`.**
1. Check the PORTS tab in VS Code lists 8765.
2. Confirm the bridge is running: `ros2 node list` should include `/foxglove_bridge`. If not, start it with `ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765` (or `bri bridge`).

**I see topics but no cones in Foxglove.** Install the extension (Step 5) and reload Foxglove.

**The car does not move.** Call `/set_mission` and then `/go` (Step 6) *before* publishing to `/cmd`, and publish continuously (`--rate 10`). If you already set a mission in this run of the simulator, restart it.

**Odd behaviour after several launches (duplicate topics, stale state).** An old simulator process may still be running. Find it with `ros2 node list` and stop it, or rebuild the container from the Command Palette (**Dev Containers: Rebuild Container**).

