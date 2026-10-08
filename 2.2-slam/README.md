# 2.2 SLAM

> **Status: outline, not yet written.** Needs an author from the SLAM team. Follows [TRACK_TEMPLATE.md](../TRACK_TEMPLATE.md).

**The question:** the car sees noisy cone detections and knows how fast it is moving. How do you build a map of the track and work out where the car is in it?

**Where it sits:** the blue boxes in the [system diagram](../system-architecture-diagram.png), between perception and planning. Input: detected cones plus the car's motion. Output: a map (`/slam/map`) and the car's pose (`/slam/pose`).

**Needs:** Modules 0 and 1.

## What the real stack does
Our SLAM (not yet in this repository; it lives in `bristol-core-sim2/slam`) is pose-graph SLAM: it matches new cone detections to cones already in the map (**data association**) and then optimises the whole map and trajectory. It has its own benchmark, `bri bench slam`, that scores localisation error from a recorded bag.

## Planned shape
- **Hands-on:** a cut-down exercise on one slice of the problem, probably **data association** (matching detections to map cones with a distance gate) on a small recorded bag, written as a pure function with instant tests.
- **See it work:** the SLAM map markers in Foxglove (`/slam/visualization`) lining up with the true cones.
- **Check:** an error number (position error against ground truth) from a small script.
- **What the real work is like:** to be written with the SLAM team (reading covariance plots, benchmarking runs, tuning).

## To decide
- Which slice of SLAM is the best first exercise (data association is the suggestion).
- Record a short bag from the simulator to ship as data (about 30–60 s).
- Who writes and verifies the solutions.
