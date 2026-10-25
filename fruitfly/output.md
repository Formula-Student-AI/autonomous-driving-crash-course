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