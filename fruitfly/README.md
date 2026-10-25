# fruitfly — flyscreen brain/body driving the EUFS car

A reusable bridge between:

* **flyscreen** — the MaleCNS v1.0 connectome (141,781 neurons / 5.5M synapses)
  running a whole-brain LIF network, plus a MuJoCo `flybody`, rendered as a
  brain point cloud and a moving fly (Windows native), and
* **eufs_sim2** — the Formula Student simulator in the ROS 2 dev container.

The fly does not get a hand-written policy. Its motor-neuron groups produce a
low-frequency steering/throttle signal; a pure-pursuit assist keeps the car on
the centreline, and the blue/yellow cones become physical-ish walls. You can
watch the brain light up and the fly body twitch while the car laps.

> Honest note: a connectome is not a trained driver. The fly supplies a real
> (if chaotic) bias; the assist layer is what completes the lap. `mode: fly`
> removes the assist and shows the raw behaviour.

---

## 1. What gets built

```
blue/yellow cones ──> ordered wall polylines ──> 160x120 "retina"
                                                     │  POST /api/live/frame
                                                     ▼
 /odom ──> pure-pursuit assist + wall guard ──> /cmd      flyscreen (Windows)
                ▲                                   │            │
                └──── /fly/motor ◄── JSON ◄──────────┼────────────┘
                                                    │
 /fly/brain/compressed, /fly/body/compressed, /fly/retina, /fly/walls
                     └──────────── Foxglove ────────┘
```

* **Blue = left wall, yellow = right wall.** Both are ordered by projecting every
  cone onto the FSUK centreline and connecting neighbours of the same colour.
  On the 2023 trackdrive map this produces one closed 99-point polyline per
  colour, with a measured half-width of **1.59–2.43 m** (median 1.94 m).
* **Virtual collision.** The EUFS vehicle model has no collision world, so the
  bridge treats `|lateral| > half_width - wall_margin` as a wall hit: it brakes,
  steers back toward the centreline, publishes `/fly/wall_contact`, and
  optionally resets the fly.* **The fly's eye** is the wall polylines projected into a pinhole view in the
  car frame. It never sees the cones as dots; it sees two continuous walls.
* **Foxglove** gets the brain render, the MuJoCo body, the retina the fly is
  actually seeing, and the wall markers.

---

## 2. Quick start

### 2.0 Prerequisites

* The portable flyscreen is extracted (e.g. `C:\Users\V9\flyscreen-portable\flyscreen-portable\sesame-flyscreen`).
* The dev container is running and `sim/` is built.
* Run the simulator first (in the container):

  ```bash
  source sim/install/setup.bash
  ros2 launch eufs_sim2 eufs_sim2.launch.py
  ros2 service call /set_mission eufs_msgs/srv/SetMission "{mission: 4}"
  ros2 service call /go std_srvs/srv/Trigger "{}"
  ```

### 2.1 Install the flyscreen extension (once)

```powershell
powershell -ExecutionPolicy Bypass -File fruitfly\flyscreen_live\install.ps1
```

This copies five files into `sesame-flyscreen` and keeps a timestamped backup in
`sesame-flyscreen\.flyscreen_live_backup\`. Pass `-Flyscreen <path>` if yours is
elsewhere.

### 2.2 Start the flyscreen live server (Windows, terminal 1)

```powershell
fruitfly\tools\start_flyscreen_live.cmd
```

or directly:

```powershell
cd C:\Users\V9\flyscreen-portable\flyscreen-portable\sesame-flyscreen
..\runtime\python\python.exe live_webui.py --size mid
```

Open <http://127.0.0.1:8760/> to see the brain and body. The page says `LIVE`.
The first start takes ~10–30 s (5.5M synapses + numba + MuJoCo).

Optional: `--posture 0.1` keeps the body upright for a few seconds before the
brain-driven twitches win (0.3 is unstable); `--float` gives a floating body with
moving legs/wings and no ground contact. Pass `-ExtraArgs @("--size","mid")` to
`start_flyscreen_live.ps1` if you want the original pure-physics body.

### 2.3 Start the WSL relay (Windows, terminal 2)

```powershell
fruitfly\tools\start_relay.cmd
```

This forwards `172.17.0.1:8780` (the WSL VM, reachable from the container) to
`127.0.0.1:8760` (flyscreen on Windows). It must stay open.

Why: WSL mirrored networking lets Linux reach Windows' `localhost`, but the
Docker container cannot. The relay is the smallest bridge that works without
exposing flyscreen on `0.0.0.0`.

### 2.4 Build and run the bridge (container, terminal 3)

```bash
cd /workspaces/autonomous-driving-crash-course/fruitfly/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch fly_bridge fly_bridge.launch.py
```

The car should start moving within a second. Watch `/fly/status`:

```bash
ros2 topic echo /fly/status
```

A healthy line contains `"fly_connected": true` and a non-zero `motor.steer`.

### 2.5 Foxglove

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

Add these panels:

| Topic | Panel |
|---|---|
| `/fly/brain/compressed` | Image — the brain point cloud |
| `/fly/body/compressed` | Image — the MuJoCo fly |
| `/fly/retina` | Image — what the fly sees (mono8) |
| `/fly/walls` | 3D — blue/yellow walls + green car dot |
| `/odom`, `/cmd` | 3D / Raw Messages |

`/camera/cones` and `/cones` are still available for the normal EUFS workflow.

---

## 3. Modes and tuning

Set in `ros2_ws/src/fly_bridge/config/fly_bridge.yaml` or override at launch:

```bash
ros2 run fly_bridge fly_bridge --ros-args -p mode:=assist -p fly_steer_gain:=0.18
```

| Parameter | Default | Meaning |
|---|---|---|
| `mode` | `assist` | `assist` = fly bias + pursuit; `fly` = fly steering only (chaotic); `pursuit` = ignore the fly |
| `fly_url` | `http://172.17.0.1:8780` | flyscreen live API via the WSL relay |
| `target_speed` | `3.0` m/s | pursuit speed before curvature slowdown and fly wish |
| `lookahead` | `4.0` m | pure-pursuit lookahead |
| `fly_steer_gain` | `0.18` rad | how much of the fly's `steer` reaches the wheel |
| `fly_speed_gain` | `2.0` m/s | how much `throttle-0.5` moves the target speed |
| `wall_margin` | `0.35` m | start braking this far inside the cone line |
| `wall_guard` | `true` | enable the virtual wall |
| `reset_on_wall` | `false` | reset the fly brain after a wall hit |
| `publish_images` | `true` | republish brain/body JPEGs to ROS |

Good first experiments:

* `mode:=fly` to see the raw connectome drive (expect a crash quickly).
* `fly_steer_gain:=0.35` to give the fly more authority while keeping the assist.
* `target_speed:=2.0` for a calmer first run.
* `wall_margin:=0.6` to keep further from the cones.

---

## 4. ROS interface

### Published

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/cmd` | `ackermann_msgs/AckermannDriveStamped` | 20 Hz | assist + fly command |
| `/fly/motor` | `std_msgs/Float32MultiArray` | 20 Hz | `[steer, accel, target_speed, lateral, half_width, s, fly_left, fly_right, fly_wing, fly_throttle, contact]` |
| `/fly/wall_contact` | `std_msgs/Bool` | 20 Hz | virtual wall hit |
| `/fly/status` | `std_msgs/String` (JSON) | 1 Hz | mode, connectivity, motor, speed, contacts |
| `/fly/brain/compressed` | `sensor_msgs/CompressedImage` (jpeg) | ~10 Hz | brain point cloud |
| `/fly/body/compressed` | `sensor_msgs/CompressedImage` (jpeg) | ~10 Hz | MuJoCo body |
| `/fly/retina` | `sensor_msgs/Image` (`mono8`) | ~15 Hz | the fly's eye |
| `/fly/walls` | `visualization_msgs/MarkerArray` | latched + 5 s | blue/yellow walls, car dot |

### Subscribed

| Topic | Type | Use |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | pose, speed, rendering the retina |

No `eufs_msgs` dependency: the track comes from the CSV copies in
`ros2_ws/src/fly_bridge/data/`.

---

## 5. Files

```
fruitfly/
├── flyscreen_live/              # drop-in extension for sesame-flyscreen
│   ├── flyscreen/live.py        #   thread-safe frame/motor mailbox
│   ├── flyscreen/config.py      #   + LiveConfig
│   ├── flyscreen/web/server.py  #   + /api/live/* routes
│   ├── webui.py                 #   + --live, --posture, _live_one_frame
│   ├── live_webui.py            #   entry point
│   └── install.ps1
├── ros2_ws/src/fly_bridge/
│   ├── fly_bridge/track.py      # cone ordering, walls, lateral queries
│   ├── fly_bridge/vision.py     # wall polylines -> 160x120 retina
│   ├── fly_bridge/control.py    # pure pursuit + fly bias + wall guard
│   ├── fly_bridge/http_client.py
│   ├── fly_bridge/node.py       # ROS node + HTTP worker
│   ├── config/fly_bridge.yaml
│   ├── launch/fly_bridge.launch.py
│   └── data/trackdrive.csv, data/centerline.csv
├── tools/
│   ├── wsl_relay.py             # WSL -> Windows TCP relay
│   ├── start_relay.ps1/.cmd
│   ├── start_flyscreen_live.ps1/.cmd
│   ├── test_track.py            # offline: walls, retina, control
│   ├── test_live_api.py         # live HTTP API smoke test
│   ├── test_drive_loop.py       # full brain/body motor test
│   └── ros_probe.py             # rclpy probe (odom/cmd/images/lateral)
└── out/                         # test artefacts (retina_start.pgm/png)
```

---

## 6. The live API (for another repo)

The flyscreen extension adds four routes to the existing web server:

```
POST /api/live/frame?w=160&h=120&fmt=gray
     body: width*height bytes, one byte per pixel (0..255)
GET  /api/live/motor
     -> {"connected": true, "frame_id": 12, "motor": {...}, "status": {...}}
GET  /api/live/brain.jpg
GET  /api/live/body.jpg
POST /api/live/reset
```

`motor` contains `steer` (-1..1, right legs positive => left turn), `throttle`
(0..1), `left`, `right`, `wing`, `activity`, `spikes`, `upright`, `t`.
The values are low-pass filtered with `LiveConfig.motor_tau` (default 0.35 s);
the raw twitch is still visible in the body render.

To reuse this in another vehicle/track:

1. Copy `fruitfly/` into the new repo.
2. Replace `ros2_ws/src/fly_bridge/data/trackdrive.csv` and `centerline.csv`
   (or set `track_file` / `centerline_file` parameters).
3. Set `fly_url` to wherever the relay listens as seen from that container.
4. Map `motor.steer` / `motor.throttle` into that vehicle's command type in
   `node.py` (`_on_control`); the wall/vision code is vehicle-agnostic.

The ROS package needs `rclpy`, `ackermann_msgs`, `std_msgs`, `sensor_msgs`,
`nav_msgs`, `geometry_msgs`, `visualization_msgs` and `ament_index_python`.
They are all present in the course dev container; in a fresh image use
`rosdep install --from-paths ros2_ws/src --ignore-src -r -y`.

---

## 7. Tests

Offline (Windows or Linux, no ROS):

```powershell
runtime\python\python.exe -I fruitfly\tools\test_track.py
```

It checks the wall ordering (blue left / yellow right), centreline projection,
the retina, and the wall guard. Current result:

```
blue: lateral sign +98 / -0
yellow: lateral sign +0 / -98
half-width 1.59..2.43 m (median 1.94)
RESULT: PASS
```

Live flyscreen (start `live_webui.py` first):

```powershell
runtime\python\python.exe -I fruitfly\tools\test_live_api.py
runtime\python\python.exe -I fruitfly\tools\test_drive_loop.py --seconds 8
```

ROS (container, after starting the sim and bridge):

```bash
python3 fruitfly/tools/ros_probe.py --seconds 10 --verbose
```

A good run shows `cmd ~20 Hz`, `brain/body/retina` messages, and lateral offsets
inside the measured half-width. In the verification run the car held
`|lateral| <= 0.64 m` over ~100 m at ~2.3 m/s with `contact_count = 0`.

---

## 8. Troubleshooting

**`fly_connected: false`** — check, in order:
1. `http://127.0.0.1:8760/api/state` on Windows has `"live": true`;
2. `start_relay.ps1` is running;
3. from the container:
   `python3 -c 'import urllib.request; print(urllib.request.urlopen("http://172.17.0.1:8780/api/state").read())'`.

**The car does not move** — the `control_input` plugin has a 0.3 s deadman.
Check `/cmd` is publishing at 20 Hz (`ros_probe.py`) and that the sim got
`/set_mission {mission: 4}` + `/go` *after* the track finished loading.

**The body is on its back** — that is the default pure-physics behaviour. Start
flyscreen with `--posture 0.1` (it stays upright for a few seconds; 0.3 is
unstable) or `--float` for a floating body.

**The car steers oddly at the start** — it spawns at `(0,0)`, about 2.4 m from
the centreline start `(2.42,0.14)`. The first few seconds are the assist pulling
it onto the track; afterwards lateral offsets are small.

**The fly's retina is blank** — the bridge only posts frames after `/odom`
arrives. Start the sim, then the bridge.

**Port 8780 already in use** — an old relay is running. Stop it (Ctrl-C in the
relay window) or use another `listen_port`, and set `fly_url` to match.

---

## 9. Licence / data

The flyscreen code and this bridge are MIT. The connectome data is MaleCNS v1.0
(HHMI Janelia FlyEM, CC BY 4.0); the body is flybody / MuJoCo Menagerie. The
FSUK track CSV is part of the course simulator.

This is a research/art piece, not a digital twin and not a trained controller.
