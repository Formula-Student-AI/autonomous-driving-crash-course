# Autonomous Driving Crash Course

A hands-on introduction to the software behind a Formula Student AI car, from the Bristol Formula Student AI society. No robotics experience needed, just basic programming and the willingness to type commands into a terminal.

By the end you will have run our simulator, made a car move with code you wrote, and tried the kind of work each of our software teams does, so you can choose where you want to contribute.

> Modelled on the [EUFS Mobile Robotics Crash Course](https://gitlab.com/eufs/admin/mobile-robotics-crash-course). The simulator is [eufs_sim2](https://gitlab.com/eufs/public/eufs_sim2) from Edinburgh University Formula Student.

## What an autonomous car does

![Architecture of the driverless system: green sensor blocks (Lidar, Camera, IMU, GPS, wheel odometry) feed yellow perception blocks (Lidar Pipeline, Camera Pipeline, Object Fusion) which output cones; blue localisation and mapping blocks (EKF Localisation, SLAM) output state and a map; red planning and control blocks (Local planning, Global planning, MPC) output controls.](system-architecture-diagram.png)

Read it left to right: **sensors** see the world, **perception** turns raw data into cones, **localisation and mapping** works out where the car is and builds a map of the track, and **planning and control** decides where to go and sends steering and throttle. The colours are the teams.

> **We do not use lidar.** The diagram shows the general shape of a driverless system, but our car finds cones with a **stereo camera** only, so the Lidar and Lidar Pipeline boxes do not exist in our stack. Lidar emulation is switched off in the simulator too (`enabled: false` in its config). Ignore those two boxes.

**What the simulator does to this picture:** it replaces everything on the left. Rather than rendering images for a camera pipeline to process, it publishes the cones directly, as if perception had already run. So in Modules 0 and 1 you work with the right-hand half of the diagram, and perception is practised separately on real images.

## The course

| | Module | You will |
|---|---|---|
| 0 | [Environment Setup](0-environment-setup/) | Install the tools, run the simulator, drive the car with commands |
| 1 | [Introduction to ROS](1-introduction-to-ros/) | Learn nodes, topics and services, then write code that drives the car |
| 2.1 | [Perception](2.1-perception/) | Find cones in images |
| 2.2 | [SLAM](2.2-slam/) | Build a map and locate the car in it |
| 2.3 | [Planning & Control](2.3-planning-and-control/) | Make the car follow the track |

**How to use it:** do Modules 0 and 1 in order. Then try **at least one** of the 2.x modules in any order. They are tasters; each shows what working in that team is really like, including the boring parts. They do not depend on each other.

## If you have not done much programming

You do not need to be a strong programmer to start, and you definitely do not need to finish a course before joining us. **Start Module 0 now and look things up when you get stuck.** Reading about programming is a slow way to learn it; writing code that does something you care about is a fast one.

Keep these open in a tab as **references to dip into**, not courses to complete:

| For | Use |
|---|---|
| **Python** — most of what you will write here, including every Module 1 exercise | [Automate the Boring Stuff](https://automatetheboringstuff.com/) (free, practical) or the [official Python tutorial](https://docs.python.org/3/tutorial/) |
| **C++** — the simulator core, and our planning and control code | [learncpp.com](https://www.learncpp.com/) — thorough and free. Chapters 1–8 are plenty to begin with |
| **The terminal, and git** — you will live in both | [MIT's The Missing Semester](https://missing.csail.mit.edu/), the practical skills no one teaches you |
| **ROS** | [ROS 2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html), but start with [Module 1](1-introduction-to-ros/) — it is shorter and uses our car |

**Start with Python.** It is what the exercises use, and you can be useful with it quickly. Pick up C++ later, when you want to work on the simulator or on control.

A good rule when you are stuck: try it, read the error message properly, search that error, then ask us. Getting stuck and unstuck *is* the skill.
