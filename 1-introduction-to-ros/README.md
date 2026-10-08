# 1. Introduction to ROS

In [Module 0](../0-environment-setup/README.md) you drove the car by typing three commands. In this module you learn what those commands actually do, then write the same thing as code: nodes that **listen** to the car, **drive** it, and **control** it.

**Needs:** Module 0 finished (the simulator runs and you can see it in Foxglove).

> **Going deeper.** This module teaches only the ROS you need for the exercises. When you want the full picture, use these (we are on ROS 2 **Humble**):
> - [ROS 2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html): the official course. Start with *Beginner: CLI tools* and *Beginner: Client libraries*.
> - [ROS 2 concepts](https://docs.ros.org/en/humble/Concepts/Basic.html): nodes, topics, services, actions.
> - [EUFS Mobile Robotics Crash Course](https://gitlab.com/eufs/admin/mobile-robotics-crash-course): the course this one is modelled on. Its ROS exercise lives in their [Getting Started repo](https://gitlab.com/eufs/admin/onboarding/getting_started). It uses ROS 2 Galactic; the ideas are identical.
> - [ROS 2 tutorials video playlist (Hummingbird)](https://www.youtube.com/playlist?list=PLgG0XDQqJckkSJDPhXsFU_RIqEh08nG0V) if you prefer video.

## Part 1: The core concepts

A ROS system is a set of small programs that talk to each other. The simulator, your code and Foxglove are all separate programs.

| Idea | What it is | In our simulator |
|---|---|---|
| **Node** | One running program in the ROS system | the simulator (`eufs_sim2`), `foxglove_bridge`, and the nodes you will write |
| **Topic** | A named stream of messages. Anyone can **publish** to it and anyone can **subscribe** | `/odom` (where the car is), `/cones` (what the car sees), `/cmd` (what you tell the car to do) |
| **Message** | The data format on a topic. Each topic has one type | `nav_msgs/Odometry`, `ackermann_msgs/AckermannDriveStamped` |
| **Service** | A request and a reply, for one-off actions rather than streams | `/set_mission`, `/go`, `/reset` |
| **Frame (TF)** | A coordinate system attached to something, like the car or the map. TF tracks how frames relate | `/tf`, used by Foxglove to place the car and sensors |

Publishers and subscribers don't know about each other, only about the topic name. That is why you can replace any part of the system (the simulator with the real car, our planner with yours) without changing the rest.

```
 ┌───────────────┐   /odom, /cones    ┌─────────────┐
 │   simulator   │ ─────────────────► │  your node  │
 │  (eufs_sim2)  │ ◄───────────────── │             │
 └───────────────┘      /cmd          └─────────────┘
         ▲                                  
         └── /set_mission, /go (services) ──┘
```

This loop is the whole job of an autonomous car: read sensors, decide, command, repeat.

**Quality of Service (QoS).** Publishers and subscribers also agree on delivery settings called *quality of service*. Most topics here work with the default (`10` as the queue size, as in the exercises). If you ever subscribe and receive nothing, check `ros2 topic info /topic --verbose`: a mismatch in reliability settings is the usual reason.

## Part 2: Look inside the running system 

Start the simulator ([Module 0, Step 4](../0-environment-setup/README.md#step-4-launch-the-simulator)) and use a second terminal. In each of these, **predict** what you will see before you run it.

```bash
ros2 node list                    # which programs are running?
ros2 topic list                   # which streams exist?
ros2 topic info /cmd              # what type is it, who publishes, who subscribes?
ros2 interface show ackermann_msgs/msg/AckermannDriveStamped   # what is inside the message?
ros2 topic echo /odom --once      # one message from the car: position, orientation, velocity
ros2 topic hz /odom               # how often is it published?
ros2 service list                 # which services exist?
ros2 interface show eufs_msgs/srv/SetMission                   # request (above ---) and response (below)
ros2 run tf2_tools view_frames    # saves frames.pdf: how the car's frames connect
```

**Checks:**
- `/cmd` has a subscriber (the simulator) and no publisher until you publish.
- `ros2 topic info /odom` says `nav_msgs/msg/Odometry`.
- You can point at the message field that holds the car's forward speed.

In Foxglove, add a **Plot** panel for `/odom.twist.twist.linear.x` and a **Raw Messages** panel for `/cones`. You will use them to check the exercises.

## Part 3: Exercises

You will write five small nodes. The skeletons are in [`exercises/src/fsai_intro`](exercises/src/fsai_intro/fsai_intro/) with `TODO` markers. Full solutions are in [`solutions/`](solutions/): try each exercise properly before looking.

### Set up the workspace (once)

A *workspace* is a folder of ROS packages you build together. You already have one: `sim/`, which the container built for you. This second one is for **your** code: `1-introduction-to-ros/exercises`. Keeping them separate means you never wait for the simulator's C++ to recompile when you change a line of your own Python.

In a terminal **inside the dev container**:

```bash
cd 1-introduction-to-ros/exercises
colcon build --symlink-install
source install/setup.bash
```

That last line is what makes `ros2 run fsai_intro ...` work, and you need it in **every new terminal** where you run your own nodes. Forgetting it is the most common cause of `package 'fsai_intro' not found`.

<details>
<summary><b>What does <code>source</code> actually do, and why is the simulator exempt?</b></summary>

`colcon build` puts the built package in `install/`, but your shell has no idea it is there. `install/setup.bash` is a generated script that sets a handful of environment variables — `AMENT_PREFIX_PATH`, `PYTHONPATH`, `PATH`, `LD_LIBRARY_PATH` — which is how `ros2` finds packages at all. `source` means "run this script in my current shell so the variables stick", rather than in a child shell that exits immediately.

Environment variables are per-shell, which is why a new terminal starts out knowing nothing again.

The simulator is exempt only because the container does it for you automatically. When this image was built, two lines were appended to `/etc/bash.bashrc`, the file bash runs for every interactive shell:

```bash
source /opt/ros/humble/setup.bash
[ -n "$FSAI_WS" ] && [ -f "$FSAI_WS/sim/install/setup.bash" ] && source "$FSAI_WS/sim/install/setup.bash"
```

So every terminal you open sources ROS, and then the simulator if it has been built (`FSAI_WS` is set to this folder by `.devcontainer/devcontainer.json`). There is no magic: it is the same `source` command you just typed, run on your behalf.

Two consequences worth knowing:

- A terminal you opened **before** the simulator finished building will not have it. Open a new one.
- `/etc/bash.bashrc` only runs for *interactive* shells. A script you run with `bash myscript.sh` does not get it, so scripts have to source what they need themselves — which is exactly what [`.devcontainer/build-sim.sh`](../.devcontainer/build-sim.sh) does on its first line.

You could add your exercise workspace to `bashrc` too. We deliberately do not: typing it is how you learn what it does, and in the 2.x modules you will switch between workspaces.
</details>

`bri build` does the same `colcon build` from wherever you are, if you prefer the shortcut.

Because of `--symlink-install`, after this first build **you do not need to rebuild when you edit the Python files**. Save, then re-run the node. (Rebuild only if you add a new node or change `setup.py`; all five nodes are already registered.)

> **The development loop you will use for the rest of the course:** edit → save → run → watch what happens → repeat. Keep the simulator in one terminal, your node in another, and Foxglove open.

Every exercise runs with `ros2 run fsai_intro <name>`. Stop a node with `Ctrl+C`.

### Exercise 1: Listen to the car

**Idea.** A *subscription* tells ROS "call this function every time a message arrives on this topic".

**Task.** In `ex1_listener.py`, subscribe to `/odom` and log the car's speed (m/s) once a second.

```bash
ros2 run fsai_intro ex1_listener
```

**Check.** With the sim running and the car stopped, it logs `speed: 0.00 m/s`. Now drive the car using the typed commands from Module 0 in a third terminal. The logged speed should rise, and match the plot in Foxglove.

### Exercise 2: Drive the car

**Idea.** A *publisher* sends messages to a topic; a *timer* makes your node do something at a steady rate. The simulator brakes the car if `/cmd` stops arriving for about 0.3 s, so a steady 10 Hz stream is required.

**Task.** In `ex2_driver.py`, publish an `AckermannDriveStamped` on `/cmd` at 10 Hz with acceleration `2.0` and steering angle `0.2`.

The car must be told which event to run before it will move. For now do that from the command line (the two `ros2 service call` commands from [Module 0, Step 6](../0-environment-setup/README.md#step-6-drive-the-car-with-commands), or just `bri go`), then:

```bash
ros2 run fsai_intro ex2_driver
```

**Check.** The car drives in a circle in Foxglove. Try changing the numbers, and see what happens when you stop the node.

### Exercise 3: Start the car from code

**Idea.** A *service client* asks another node to do something and waits for the reply.

**Task.** In `ex3_mission.py`, replace those two `ros2 service call` commands with code: call `/set_mission` (Trackdrive) and then `/go`.

Restart the simulator first (`Ctrl+C`, then launch again), because the car only accepts a mission once per run.

```bash
ros2 run fsai_intro ex3_mission
```

**Check.** The node logs `success: True` twice. Then `ros2 run fsai_intro ex2_driver` drives the car without any typed service calls.

### Exercise 4: Close the loop

**Idea.** Control means using a measurement to decide the next command. A *proportional controller* sets the command in proportion to the error: `acceleration = KP × (target speed − current speed)`.

**Task.** In `ex4_speed_hold.py`, finish `control()` so the car holds 5 m/s. The node already subscribes to `/odom` and publishes `/cmd` for you.

Restart the simulator, run `ex3_mission`, then:

```bash
ros2 run fsai_intro ex4_speed_hold
```

(Do not run `ex2_driver` at the same time: two nodes publishing `/cmd` fight each other.)

**Check.** In the Foxglove plot, speed climbs and settles near 5 m/s. Change `TARGET_SPEED` and `KP` and watch the result. What happens with a very large `KP`?

### Exercise 5: See what the car sees

**Idea.** The simulator publishes the cones the car's camera would detect, in `/cones`, each with a position and colour probabilities. These are the inputs to SLAM and planning.

**Task.** In `ex5_nearest_cone.py`, find the cone nearest the car and log its distance and most likely colour.

```bash
ros2 run fsai_intro ex5_nearest_cone
```

**Check.** The log shows a plausible distance (a few metres) and a colour. Compare it with the 3D view in Foxglove.

**Stretch.** Combine exercises 4 and 5 in one node: slow down when the nearest cone is close. This is the beginning of the planning and control track.

## Part 4: The whole system on one page

Everything you have just done sits inside a bigger picture. Here is the full driverless system:

![Architecture of the driverless system: green sensor blocks (Lidar, Camera, IMU, GPS, wheel odometry) feed yellow perception blocks (Lidar Pipeline, Camera Pipeline, Object Fusion) which output cones; blue localisation and mapping blocks (EKF Localisation, SLAM) output state and a map; red planning and control blocks (Local planning, Global planning, MPC) output controls.](../system-architecture-diagram.png)

Every box is one or more ROS nodes, and every arrow is a topic. That is the whole trick: you already know how to read this diagram, because you spent this module writing the boxes and arrows yourself.

> **Ignore the two Lidar boxes.** Our car uses a **stereo camera** only, so there is no lidar and no lidar pipeline in our stack. The simulator can emulate a lidar, but we have it switched off (`lidar: enabled: false` in `sim/src/eufs_sim2/config/plugin_params.yaml`), which is why you see `/camera/cones` but nothing useful on `/lidar_grid/cones`.

Match it to what you did:

| In the diagram | What you saw |
|---|---|
| Green **sensors** | `/imu/data`, the wheel speeds, the GPS fix — all published by the simulator. No lidar: we use a stereo camera |
| Yellow **perception** → cones | The simulator skips this and publishes `/cones` directly, as if the camera pipeline had already run. This is why [2.1 Perception](../2.1-perception/) works on real images instead |
| Blue **localisation and mapping** | `/odom` is the simulator handing you a perfect answer. On the real car, [2.2 SLAM](../2.2-slam/) has to work it out from noisy cones |
| Red **planning and control** → controls | Your Exercise 4 node, in miniature: read the state, decide, publish `/cmd`. [2.3](../2.3-planning-and-control/) is this box done properly |

Two things worth noticing:

- **The simulator stands in for the left-hand side**, which is why you could drive a car on day one without writing a detector.
- **Every arrow is a decision someone made** about what message type to use and what the data means. Keeping those arrows consistent across teams is a real job, and a frequent source of bugs.

## What to remember

- **Everything is nodes talking over topics and services.** You can inspect it all live with `ros2 topic`, `ros2 node` and `ros2 service`.
- **The edit → run → observe loop is the work.** In the track modules the checks get more sophisticated, but the loop is the same.
- **Our stack is the diagram above:** perception, SLAM, planning and control are nodes that subscribe to the previous stage's topic and publish their own.

## Next

Choose at least two tasters, in any order:
- [2.1 Perception](../2.1-perception/)
- [2.2 SLAM](../2.2-slam/)
- [2.3 Planning & Control](../2.3-planning-and-control/)

---

*Maintainer notes:* the exercises have not been run against the simulator yet (only syntax-checked). Open questions: confirm `/cones` positions are relative to the car; confirm the `/go` transition timing; decide whether to add an automated checker per exercise.
