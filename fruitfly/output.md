What was added
fruitfly/ (305 KB, uncommitted) — everything needed to reuse this in another repo:
    fruitfly/ros2_ws/src/fly_bridge/ — ROS 2 package
        track.py — orders the blue/yellow cones along the FSUK centreline, connects them into wall polylines, estimates half-width, answers lateral queries
        vision.py — projects the walls into a 160×120 pinhole “retina”
        control.py — pure-pursuit assist + fly bias + virtual wall guard
        http_client.py, node.py — HTTP worker + ROS node
    fruitfly/flyscreen_live/ — drop-in extension for the portable flyscreen (live.py, config.py, web/server.py, webui.py, live_webui.py, install.ps1)
    fruitfly/tools/ — WSL relay, start scripts, and four test/probe scripts
    fruitfly/README.md — architecture, quick start, topics, tuning, troubleshooting
Your actual flyscreen has been patched in place (backup under .flyscreen_live_backup/). It now has live_webui.py, webui.py --live, and --posture.

Requirements → implementation
Blue/yellow cones as walls — each colour becomes one closed 99-point wall polyline (blue = left, yellow = right), with measured half-width 1.59–2.43 m (median 1.94 m). The walls are rendered into the fly’s retina, published to Foxglove on /fly/walls, and used by a virtual collision guard (/fly/wall_contact, brake + steer back).
Controller/API in the ROS 2 container — fly_bridge publishes /cmd at 20 Hz and reads the fly’s live motor signal. Modes: assist (default), fly (raw connectome), pursuit (no fly). The flyscreen live API is POST /api/live/frame, GET /api/live/motor, GET /api/live/{brain,body}.jpg, POST /api/live/reset.
Brain map + MuJoCo body — the flyscreen web UI shows both; the bridge republishes them to ROS as /fly/brain/compressed, /fly/body/compressed, plus /fly/retina and /fly/walls for Foxglove.

Verified end-to-end
Offline geometry/vision/control test: PASS
Live HTTP API + full brain/body loop: PASS (non-zero motor, JPEGs present)
ROS end-to-end: /odom and /cmd at 20 Hz, brain/body/retina streaming; car at ~2.3 m/s, |lateral| ≤ 0.64 m over ~100 m, 0 wall contacts
Original video mode regression: PASS (27 frames, 3.1 fps)
The fly’s live render loop runs at ~1.4 fps (MuJoCo/brain bound); the bridge treats the fly’s own motor timestamp as freshness and the assist keeps the car smooth at 20 Hz.

Run it

# once
powershell -ExecutionPolicy Bypass -File fruitfly\flyscreen_live\install.ps1

# terminal 1 — flyscreen live
fruitfly\tools\start_flyscreen_live.cmd

# terminal 2 — WSL relay (container reaches Windows only through this)
fruitfly\tools\start_relay.cmd

# container terminal 3 — sim
source sim/install/setup.bash
ros2 launch eufs_sim2 eufs_sim2.launch.py
ros2 service call /set_mission eufs_msgs/srv/SetMission "{mission: 4}"
ros2 service call /go std_srvs/srv/Trigger "{}"

# container terminal 4 — bridge
cd fruitfly/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch fly_bridge fly_bridge.launch.py
Foxglove panels: /fly/brain/compressed, /fly/body/compressed, /fly/retina, /fly/walls.

Two honest caveats: the connectome is not a trained driver — the assist is what completes the lap (mode:=fly shows the raw behaviour); and the walls are virtual because the EUFS bicycle model has no collision world. The body may flip onto its back; --posture 0.1 delays it and --float keeps it airborne. The dev container was stopped after testing, and nothing was committed.






1. More realistic simulator scene
EUFS has no 3D renderer — its Foxglove view is markers. I added a marker-based scene in scenery.py, published latched on /fly/scenery:

Real cone meshes — tapered bodies, white reflective band, black square base (not cylinders/dots)
Asphalt road ribbon with white edge lines and a start/finish checker
Barriers with red top rails on both sides
Trees outside the barriers
Grass ground plane
Optional sky dome (publish_sky:=true); setting the Foxglove 3D panel background to #87CEEB is usually better for camera framing
The fly’s retina in vision.py was upgraded too: sky gradient, ground, filled road between the walls, and each cone drawn as a tapered marker with a bright band.

The default EUFS Foxglove extension still draws cone cylinders; our meshes cover them. Hide the extension’s cone display if you want only the realistic meshes.

2. Pure pursuit no longer clips cones
Root cause found: the wall guard compared the car’s centre to half_width - 0.35 m, but the car is ~1.5 m wide. The edge could pass the cone line while the centre looked safe.

Fixes in control.py and node.py:

Limit is now half_width - car_half_width - wall_margin (whole car inside the cones)
Speed-dependent lookahead (shorter at low speed, so it stops cutting corners)
Stanley-style cross-track correction added to pure pursuit
Heading-based prediction v * sin(yaw - heading) — sees a wall hit coming without the projection-noise false positives
Predicted warnings slow the car; only an actual violation hard-brakes
Cone-hit accounting: /fly/cone_hits, /fly/penalty_s, /fly/lap (per-lap, reset at the start line)
lap_min_s prevents the spawn-line crossing being counted as a lap
Verification: completed a lap at 0 cone hits / 0 s penalty; final 25 s run had contact_count = 0, lateral error ≤ 0.15 m, 2.3–3.3 m/s.

3. Planning & control design document
Written to docs/planning_control_design.md. It covers:

End-to-end flow and rate/latency budget
Requirements (functional, performance, safety, interfaces)
Common data contracts (map, reference path, state, control)
Global planning: centreline extraction, racing line, speed profile, inputs/outputs
Four planning strategies:
Straight-road acceleration (traction/power/braking-distance formulas)
Corner optimisation — inner vs outer line, minimum curvature vs shortest path, out-in-out
Local planning with limited visibility (rolling horizon, corridor/gap selection, visibility speed limit, stop fallback)
Global planning with the full map (offline optimisation, re-plan on map change)
MPC: kinematic/dynamic bicycle model, state/control definitions, cost function, constraints (steering, accel, corridor, friction circle), solvers (OSQP/acados/CasADi), rates, inputs/outputs, and current Formula Student AI practice
CAN output: ROS → CAN chain, signal table (steer/torque/brake/enable/mission/heartbeat/feedback), torque mapping, deadman/EBS/plausibility safety
Where the fly bridge fits and how to replace pure pursuit with the MPC
Validation plan and parameter summary
Key MPC formulation in the doc:


x = [X, Y, psi, v]
u = [delta, a]
Ẋ = v cos psi,  Ẏ = v sin psi,  psi̇ = v/L tan delta,  v̇ = a

J = Σ (eᵀQe + uᵀRu + ΔuᵀSΔu) + e_NᵀP e_N
with corridor, steering, acceleration, and friction-circle constraints.

Run / view
The commands are unchanged (rebuild first because the last edits are not compiled into the stopped container):


cd fruitfly/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch fly_bridge fly_bridge.launch.py
New Foxglove panels: /fly/scenery (3D), /fly/cone_hits, /fly/penalty_s, /fly/lap. The README’s Foxglove section lists everything.

Honest limits: this is a marker-based scene, not a game-engine render. A truly photoreal track (textured meshes, lighting, skybox) would need Gazebo/Isaac or a custom renderer. The cone-hit penalty is a proxy (0.95 m radius, 5 s/cone, per lap), not an official judge. The fly is still a research/art layer; the assist/planner is what completes the lap.

Your initial fruitfly/ commit is intact; these follow-up changes are uncommitted. I stopped the simulator, container, relay, and flyscreen after testing. If you want the design doc as a shareable web page, I can publish it as an artifact.