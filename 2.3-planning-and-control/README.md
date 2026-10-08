# 2.3 Planning & Control

> **Status: outline, not yet written.** Needs an author from the planning/control team. Follows [TRACK_TEMPLATE.md](../TRACK_TEMPLATE.md).

**The question:** you know where the cones are. What path should the car take, and what steering and throttle make it follow that path?

**Where it sits:** the red boxes in the [system diagram](../system-architecture-diagram.png) — local and global planning, and the controller that turns a path into commands. Input: the cone map and the car's pose. Output: steering and acceleration commands on `/cmd`. This is the track where your code moves the car most directly.

**Needs:** Modules 0 and 1 (the hello-car node is the starting point).

## What the real stack does
Our planner pairs blue and yellow cones, takes midpoints, orders them and fits a spline. The controller uses PID steering on a lookahead point and PID speed control (see `bristol-core-sim2/planning_control`).

## Planned shape
- **Hands-on:** a small function such as `steering_from_cones(cones) -> angle` that steers toward the midpoint of the nearest cone pair, with instant tests on fixed examples; then tuning a lookahead distance and a PID gain.
- **See it work:** wrap the function in the hello-car node and watch the car follow the track in Foxglove; plot the steering command.
- **Check:** how far the car gets / how many cones it hits.
- **What the real work is like:** to be written with the planning/control team (C++ rebuilds, tuning, lap-time vs safety).

## To decide
- Confirm a ~30-line reactive controller can actually complete a lap in this simulator before handing it out. This has not been tested.
- Only one node may publish `/cmd` at a time, so this exercise must launch the simulator alone, with no other driver running.
- Who writes and verifies the solutions.
