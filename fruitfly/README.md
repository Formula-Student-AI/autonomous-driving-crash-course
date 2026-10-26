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
 /odom ──> pure-pursuit assist + wall guard ──> /fly/cmd ──> cmd_mux ──> /cmd
                ▲                                   │                      │
                └──── /fly/motor ◄── JSON ◄──────────┼──────────────────────┘
                                                    │
 /fly/brain/compressed, /fly/body/compressed, /fly/retina, /fly/walls, /fly/scenery
                     └──────────── Foxglove ────────┘
```

> **This is a separate fun mode.** It publishes `/fly/cmd`, never `/cmd`.
> `fake run/cmd_mux.py` forwards only the selected source to `/cmd`; the default
> is `race`. Never run the race `fixed_path_driver.py` and fly mode together.
> The main race stack and its planning/MPC design are in
> [`2.3-planning-and-control/README.md`](../2.3-planning-and-control/README.md).

The full main-car autonomy stack (global planning, local planning with four
strategies, MPC, and CAN output) is specified in
[`2.3-planning-and-control/README.md`](../2.3-planning-and-control/README.md).
The `fly_bridge` is the pure-pursuit baseline + fly layer described below.

* **Blue = left wall, yellow = right wall.** Both are ordered by projecting every
  cone onto the FSUK centreline and connecting neighbours of the same colour.
  On the 2023 trackdrive map this produces one closed 99-point polyline per
  colour, with a measured half-width of **1.59–2.43 m** (median 1.94 m).
* **Virtual collision.** The EUFS vehicle model has no collision world, so the
  bridge treats `|lateral| > half_width - car_half_width - wall_margin` as a
  wall hit. It uses the car's **width**, not just its centre, so the body cannot
  clip the inside cones. A heading-based prediction (`v * sin(yaw - heading)`)
  sees a hit coming; an actual violation brakes hard, a prediction only slows.
* **Cone-hit penalty.** The bridge counts each unique cone the car comes within
  `cone_hit_radius` of, adds `cone_penalty_s` per hit, and publishes
  `/fly/cone_hits`, `/fly/penalty_s` and `/fly/lap`. This is competition-style
  bookkeeping, not a simulator physics change.
* **The fly's eye** is the wall polylines projected into a pinhole view in the
  car frame, with road/sky/ground shading and each cone drawn as a tapered
  marker with a bright band — not two bare lines.
* **Foxglove scenery** (`/fly/scenery`): an asphalt ribbon with edge lines and a
  start/finish checker, low barriers with red rails, real tapered cone meshes
  (blue/yellow/orange + white band + black base), and trees outside the track.
  An optional sky dome is available with `publish_sky:=true`; setting the 3D
  panel background to `#87CEEB` is usually better.
* **Foxglove** gets the brain render, the MuJoCo body, the retina the fly is
  actually seeing, the wall lines, and the 3D scenery.

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

### 2.4 Build and run the fly mode (container, terminal 3)

```bash
# 1) build once
cd /workspaces/autonomous-driving-crash-course/fruitfly/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 2) one command: mux in fly mode + fly bridge (publishes /fly/cmd)
bash /workspaces/autonomous-driving-crash-course/fruitfly/tools/start_fly_mode.sh
```

The explicit version is:

```bash
python3 "/workspaces/autonomous-driving-crash-course/fake run/cmd_mux.py" \
    --ros-args -p mode:=fly
source /workspaces/autonomous-driving-crash-course/fruitfly/ros2_ws/install/setup.bash
ros2 launch fly_bridge fly_bridge.launch.py
```

The car should start moving within a second. Watch `/fly/status`:

```bash
ros2 topic echo /fly/status
```

A healthy line contains `"fly_connected": true` and a non-zero `motor.steer`.

> Do **not** run `fake run/fixed_path_driver.py` in fly mode. The mux stays in
> charge: `mode:=fly` forwards the fly, `mode:=race` (default) forwards the race
> driver. Use `ros2 topic pub /control_mode std_msgs/String "{data: race}"` to
> switch back at runtime. If the race stack or a mux is already running, stop it
> first: `bash "fake run/stop_race_mode.sh"`.
>
> Stop fly mode (container side): `bash fruitfly/tools/stop_fly_mode.sh`.
> Stop the Windows flyscreen/relay with Ctrl-C in their windows.

### 2.5 Foxglove

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

Add these panels:

| Topic | Panel |
|---|---|
| `/fly/brain/compressed` | Image — the brain point cloud |
| `/fly/body/compressed` | Image — the MuJoCo fly |
| `/fly/retina` | Image — what the fly sees (mono8, road + cone shapes) |
| `/fly/scenery` | 3D — road, barriers, real cone meshes, trees |
| `/fly/walls` | 3D — blue/yellow wall lines + green car dot |
| `/fly/cone_hits`, `/fly/penalty_s`, `/fly/lap` | Raw Messages / plots |
| `/odom`, `/cmd`, `/fly/cmd` | 3D / Raw Messages |

The default EUFS Foxglove extension also draws cones (cylinders). Our meshes
are slightly larger and cover them; hide the extension's cone display if you
want only the realistic cones. Set the 3D panel background to `#87CEEB` for a
sky, or run with `publish_sky:=true` for a sky dome.

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
| `lookahead`, `lookahead_min`, `lookahead_gain` | `4.0`, `2.5`, `0.35` | speed-dependent lookahead; shorter at low speed so it does not cut corners |
| `k_lat` | `0.6` | Stanley-style cross-track correction added to pure pursuit |
| `car_half_width` | `0.75` m | keeps the whole car inside the cone line, not just the centre |
| `wall_margin` | `0.35` m | gap between the car edge and the cone line |
| `predict_horizon`, `predict_margin` | `0.4` s, `0.15` m | heading-based early warning |
| `lat_slowdown` | `0.5` | fraction of target speed lost near the wall |
| `cone_hit_radius` | `0.95` m | cone-hit detection radius (car half-width + cone) |
| `cone_penalty_s` | `5.0` s | penalty added per unique cone hit |
| `wall_guard` | `true` | enable the virtual wall |
| `reset_on_wall` | `false` | reset the fly brain after a wall hit |
| `publish_images` | `true` | republish brain/body JPEGs to ROS |
| `publish_scenery` | `true` | publish road/barriers/cones/trees on `/fly/scenery` |
| `publish_sky` | `false` | add the sky dome (can affect 3D panel framing) |

Good first experiments:

* `mode:=fly` to see the raw connectome drive (expect a crash quickly).
* `fly_steer_gain:=0.35` to give the fly more authority while keeping the assist.
* `target_speed:=2.0` for a calmer first run.
* `wall_margin:=0.2` to use more of the track (watch `/fly/cone_hits`).
* `publish_sky:=true` for the sky dome, or set the 3D panel background instead.

---

## 4. ROS interface

### Published

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/fly/cmd` | `ackermann_msgs/AckermannDriveStamped` | 20 Hz | assist + fly command; the mux forwards it in fly mode |
| `/fly/motor` | `std_msgs/Float32MultiArray` | 20 Hz | `[steer, accel, target_speed, lateral, half_width, s, fly_left, fly_right, fly_wing, fly_throttle, contact]` |
| `/fly/wall_contact` | `std_msgs/Bool` | 20 Hz | virtual wall hit/warning |
| `/fly/cone_hits` | `std_msgs/Int32` | 20 Hz | unique cone hits this lap |
| `/fly/penalty_s` | `std_msgs/Float32` | 20 Hz | time penalty this lap |
| `/fly/lap` | `std_msgs/Int32` | 20 Hz | completed laps |
| `/fly/status` | `std_msgs/String` (JSON) | 1 Hz | mode, connectivity, motor, speed, contacts, hits, penalty, laps |
| `/fly/brain/compressed` | `sensor_msgs/CompressedImage` (jpeg) | ~10 Hz | brain point cloud |
| `/fly/body/compressed` | `sensor_msgs/CompressedImage` (jpeg) | ~10 Hz | MuJoCo body |
| `/fly/retina` | `sensor_msgs/Image` (`mono8`) | ~15 Hz | the fly's eye |
| `/fly/walls` | `visualization_msgs/MarkerArray` | latched + 5 s | blue/yellow walls, car dot |
| `/fly/scenery` | `visualization_msgs/MarkerArray` | latched + 10 s | road, barriers, cone meshes, trees, optional sky |

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
│   ├── fly_bridge/vision.py     # road/sky/ground + cone shapes -> retina
│   ├── fly_bridge/scenery.py    # 3D road, barriers, cone meshes, trees, sky
│   ├── fly_bridge/control.py    # pure pursuit + Stanley + predictive wall
│   ├── fly_bridge/http_client.py
│   ├── fly_bridge/node.py       # ROS node + HTTP worker
│   ├── config/fly_bridge.yaml
│   ├── launch/fly_bridge.launch.py
│   └── data/trackdrive.csv, data/centerline.csv
├── tools/
│   ├── wsl_relay.py             # WSL -> Windows TCP relay
│   ├── start_relay.ps1/.cmd
│   ├── start_flyscreen_live.ps1/.cmd
│   ├── start_fly_mode.sh        # mux (fly) + fly bridge, one command
│   ├── stop_fly_mode.sh         # stop mux + fly bridge in the container
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

A good run shows `cmd ~20 Hz`, `brain/body/retina/scenery` messages, and lateral
offsets inside the measured half-width. In the final verification run the car
completed a lap with **0 cone hits, 0 s penalty**, `|lateral| <= 0.4 m`, and no
false wall warnings at 2.3–3.3 m/s. (The earlier, loose wall guard had allowed
the car's *edge* to clip cones even when its centre looked fine; that is fixed
by subtracting `car_half_width` from the limit.)

The `2.3-planning-and-control/README.md` document specifies the full
global/local planning stack, the four planning strategies, and an MPC that
replaces this pure-pursuit baseline and outputs CAN signals.

---

## 8. Troubleshooting

**`fly_connected: false`** — check, in order:
1. `http://127.0.0.1:8760/api/state` on Windows has `"live": true`;
2. `start_relay.ps1` is running;
3. from the container:
   `python3 -c 'import urllib.request; print(urllib.request.urlopen("http://172.17.0.1:8780/api/state").read())'`.

**The car does not move** — the `control_input` plugin has a 0.3 s deadman.
Check `/fly/cmd` is publishing at 20 Hz (`ros_probe.py`), that the mux is
running with `mode:=fly`, and that the sim got `/set_mission {mission: 4}` +
`/go` *after* the track finished loading. In race mode the mux needs
`mode:=race` and the race driver, not the fly.

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
