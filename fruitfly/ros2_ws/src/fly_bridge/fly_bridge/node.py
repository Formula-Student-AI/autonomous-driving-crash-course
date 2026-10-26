"""ROS 2 node that couples the EUFS car to the flyscreen fly.

Data flow::

    /odom ──> wall retina (160x120 grayscale) ──POST──> flyscreen
                                                          │
    /cmd <── pure-pursuit assist + fly bias <──GET motor ──┘

The node also republishes the flyscreen brain/body JPEGs and the wall markers
so Foxglove can show everything next to the simulator.
"""
from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_directory

from .control import Command, ControlConfig, PurePursuit
from .http_client import FlyClient
from .scenery import build_scenery
from .track import Track
from .vision import render_view


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class BridgeWorker(threading.Thread):
    """All HTTP I/O lives here so ROS timers never block on the network."""

    def __init__(self, node: "FlyBridge") -> None:
        super().__init__(name="fly-http", daemon=True)
        self.node = node
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        node = self.node
        period = 1.0 / max(1.0, node.vision_rate)
        i = 0
        while not self.stop_event.is_set():
            t0 = time.time()

            odom = node.odom_snapshot()
            if odom is not None:
                x, y, yaw = odom
                retina = render_view(
                    node.track.walls, x, y, yaw,
                    width=node.vision_w, height=node.vision_h,
                    hfov=node.vision_hfov, max_range=node.vision_range,
                    near=node.vision_near, camera_height=node.camera_height,
                    thickness=node.vision_thickness,
                    cones=node.track.cones,
                )
                resp = node.fly.post_frame(retina, node.vision_w, node.vision_h)
                with node.lock:
                    node.retina = retina
                    node.retina_serial += 1
                    node.fly_post_ok = bool(resp and resp.get("ok"))

            snap = node.fly.get_motor()
            if snap is not None:
                with node.lock:
                    node.fly_snapshot = snap
                    node.fly_t = time.time()
                    node.fly_connected = bool(snap.get("connected"))

            if node.publish_images and i % 2 == 0:
                brain = node.fly.get_brain_jpeg()
                body = node.fly.get_body_jpeg()
                with node.lock:
                    if brain:
                        node.brain_jpeg = brain
                        node.brain_serial += 1
                    if body:
                        node.body_jpeg = body
                        node.body_serial += 1

            with node.lock:
                do_reset = node.reset_requested
                node.reset_requested = False
            if do_reset:
                node.fly.post_reset()
                node.get_logger().info("flyscreen reset requested after wall contact")

            i += 1
            dt = period - (time.time() - t0)
            if dt > 0:
                self.stop_event.wait(dt)


class FlyBridge(Node):
    def __init__(self) -> None:
        super().__init__("fly_bridge")

        # ---------------------------------------------------------- params
        self.fly_url = self._p("fly_url", "http://172.17.0.1:8780")
        self.mode = str(self._p("mode", "assist")).lower()
        # IMPORTANT: in the course workflow the fly must not write /cmd.
        # It writes /fly/cmd; fake run/cmd_mux.py forwards the selected mode.
        self.cmd_topic = str(self._p("cmd_topic", "/fly/cmd"))
        self.track_file = str(self._p("track_file", ""))
        self.centerline_file = str(self._p("centerline_file", ""))

        self.vision_rate = float(self._p("vision_rate", 15.0))
        self.vision_w = int(self._p("vision_w", 160))
        self.vision_h = int(self._p("vision_h", 120))
        self.vision_hfov = float(self._p("vision_hfov", 1.91889))
        self.vision_range = float(self._p("vision_range", 30.0))
        self.vision_near = float(self._p("vision_near", 0.6))
        self.camera_height = float(self._p("camera_height", 0.55))
        self.vision_thickness = int(self._p("vision_thickness", 1))

        self.control_rate = float(self._p("control_rate", 20.0))
        self.publish_images = bool(self._p("publish_images", True))
        self.image_rate = float(self._p("image_rate", 10.0))
        self.reset_on_wall = bool(self._p("reset_on_wall", False))

        self.lookahead = float(self._p("lookahead", 4.0))
        self.lookahead_min = float(self._p("lookahead_min", 2.5))
        self.lookahead_gain = float(self._p("lookahead_gain", 0.35))
        self.wheelbase = float(self._p("wheelbase", 1.58))
        self.target_speed = float(self._p("target_speed", 3.0))
        self.max_steer = float(self._p("max_steer", 0.5))
        self.max_accel = float(self._p("max_accel", 3.0))
        self.max_brake = float(self._p("max_brake", 5.0))
        self.k_lat = float(self._p("k_lat", 0.6))
        self.fly_steer_gain = float(self._p("fly_steer_gain", 0.18))
        self.fly_speed_gain = float(self._p("fly_speed_gain", 2.0))
        # The flyscreen render loop is slow (~1-3 fps on a laptop). The fly's
        # own motor timestamp is the honest freshness signal; allow a couple of
        # seconds before dropping its bias.
        self.fly_require_fresh_s = float(self._p("fly_require_fresh_s", 2.0))
        self.car_half_width = float(self._p("car_half_width", 0.75))
        self.wall_guard = bool(self._p("wall_guard", True))
        self.wall_margin = float(self._p("wall_margin", 0.35))
        self.wall_steer = float(self._p("wall_steer", 0.45))
        self.wall_brake = float(self._p("wall_brake", 5.0))
        self.predict_horizon = float(self._p("predict_horizon", 0.4))
        self.predict_margin = float(self._p("predict_margin", 0.10))
        self.lat_slowdown = float(self._p("lat_slowdown", 0.5))

        # cone-hit penalty (a proxy for the competition time penalty)
        self.cone_hit_radius = float(self._p("cone_hit_radius", 0.95))
        self.cone_penalty_s = float(self._p("cone_penalty_s", 5.0))
        self.lap_min_s = float(self._p("lap_min_s", 10.0))

        # Foxglove scenery
        self.publish_scenery = bool(self._p("publish_scenery", True))
        self.scenery_trees = bool(self._p("scenery_trees", True))
        self.publish_sky = bool(self._p("publish_sky", False))
        self.scenery_rate = float(self._p("scenery_rate", 10.0))

        # ---------------------------------------------------------- track
        share = get_package_share_directory("fly_bridge")
        track_file = self.track_file or f"{share}/data/trackdrive.csv"
        center_file = self.centerline_file or f"{share}/data/centerline.csv"
        self.track = Track(track_file, center_file)
        self.get_logger().info(self.track.summary())

        cfg = ControlConfig(
            lookahead=self.lookahead,
            lookahead_min=self.lookahead_min,
            lookahead_gain=self.lookahead_gain,
            wheelbase=self.wheelbase,
            target_speed=self.target_speed,
            max_steer=self.max_steer,
            max_accel=self.max_accel,
            max_brake=self.max_brake,
            k_lat=self.k_lat,
            fly_steer_gain=self.fly_steer_gain,
            fly_speed_gain=self.fly_speed_gain,
            fly_require_fresh_s=self.fly_require_fresh_s,
            car_half_width=self.car_half_width,
            wall_margin=self.wall_margin,
            wall_steer=self.wall_steer,
            wall_brake=self.wall_brake,
            predict_horizon=self.predict_horizon,
            predict_margin=self.predict_margin,
            lat_slowdown=self.lat_slowdown,
        )
        self.pursuit = PurePursuit(self.track, cfg)
        self.fly = FlyClient(self.fly_url)

        # ---------------------------------------------------------- state
        self.lock = threading.Lock()
        self.odom: tuple[float, float, float, float] | None = None
        self.fly_snapshot: dict = {}
        self.fly_t = 0.0
        self.fly_connected = False
        self.fly_post_ok = False
        self.retina = b""
        self.retina_serial = 0
        self.brain_jpeg = b""
        self.brain_serial = 0
        self.body_jpeg = b""
        self.body_serial = 0
        self.reset_requested = False
        self._was_contact = False
        self.contact_count = 0
        self._seen = {"brain": 0, "body": 0, "retina": 0}
        self._last_walls = 0.0
        self._last_log = 0.0

        # race / penalty bookkeeping
        self.cone_hits = 0
        self.penalty_s = 0.0
        self._hit_cones: set[int] = set()
        self._last_s: float | None = None
        self.laps = 0
        self._lap_t0 = time.time()
        self._last_lap_time: float | None = None

        # static Foxglove scenery (built once, republished periodically)
        self.scenery = (build_scenery(self.track, include_trees=self.scenery_trees,
                                      include_sky=self.publish_sky)
                        if self.publish_scenery else None)
        self._last_scenery = 0.0

        # ---------------------------------------------------------- ROS
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, self.cmd_topic, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, "/fly/motor", 10)
        self.contact_pub = self.create_publisher(Bool, "/fly/wall_contact", 10)
        self.status_pub = self.create_publisher(String, "/fly/status", 10)
        self.cone_hits_pub = self.create_publisher(Int32, "/fly/cone_hits", 10)
        self.penalty_pub = self.create_publisher(Float32, "/fly/penalty_s", 10)
        self.lap_pub = self.create_publisher(Int32, "/fly/lap", 10)
        self.brain_pub = self.create_publisher(
            CompressedImage, "/fly/brain/compressed", 1)
        self.body_pub = self.create_publisher(
            CompressedImage, "/fly/body/compressed", 1)
        self.retina_pub = self.create_publisher(Image, "/fly/retina", 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.walls_pub = self.create_publisher(MarkerArray, "/fly/walls", latched)
        self.scenery_pub = self.create_publisher(
            MarkerArray, "/fly/scenery", latched)

        self.create_timer(1.0 / max(1.0, self.control_rate), self._on_control)
        self.create_timer(1.0 / max(1.0, self.image_rate), self._on_viz)
        self.create_timer(1.0, self._on_status)

        self.worker = BridgeWorker(self)
        self.worker.start()
        self._publish_walls()
        self._publish_scenery()
        self.get_logger().info(
            f"fly bridge up: mode={self.mode} fly={self.fly_url} "
            f"cmd={self.cmd_topic} "
            f"vision={self.vision_w}x{self.vision_h}@{self.vision_rate:g}Hz "
            f"control={self.control_rate:g}Hz")

    # ------------------------------------------------------------ helpers
    def _p(self, name: str, default):
        return self.declare_parameter(name, default).value

    def odom_snapshot(self):
        with self.lock:
            if self.odom is None:
                return None
            x, y, yaw, _ = self.odom
            return x, y, yaw

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = float(msg.twist.twist.linear.x)
        with self.lock:
            self.odom = (float(p.x), float(p.y),
                         yaw_from_quat(msg.pose.pose.orientation), v)

    # ------------------------------------------------------------ control
    def _on_control(self) -> None:
        with self.lock:
            odom = self.odom
            snap = dict(self.fly_snapshot)
            fly_t = self.fly_t
            fly_connected = self.fly_connected
        if odom is None:
            return
        x, y, yaw, v = odom

        motor = snap.get("motor", {}) if snap else {}
        # Prefer the timestamp the flyscreen stamped on the motor readout; it
        # only advances when the brain/body actually produced a new frame.
        motor_t = float(motor.get("t", 0.0)) if motor else 0.0
        age = (time.time() - motor_t) if motor_t else (time.time() - fly_t)
        fresh = age <= self.fly_require_fresh_s
        fly_ok = bool(fly_connected and fresh and motor)
        fly_steer = float(motor.get("steer", 0.0))
        fly_throttle = float(motor.get("throttle", 0.0))

        if self.mode == "fly":
            cmd = self._fly_direct(x, y, yaw, v, fly_ok,
                                   fly_steer, fly_throttle)
        elif self.mode == "pursuit":
            cmd = self.pursuit.command(x, y, yaw, v, wall_guard=self.wall_guard)
        else:  # "assist" (default)
            cmd = self.pursuit.command(
                x, y, yaw, v,
                fly_steer=fly_steer, fly_throttle=fly_throttle,
                fly_ok=fly_ok, wall_guard=self.wall_guard)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = float(cmd.accel)
        msg.drive.steering_angle = float(cmd.steer)
        self.cmd_pub.publish(msg)

        arr = Float32MultiArray()
        arr.data = [
            float(cmd.steer), float(cmd.accel), float(cmd.target_speed),
            float(cmd.lateral), float(cmd.half_width), float(cmd.s),
            float(motor.get("left", 0.0)), float(motor.get("right", 0.0)),
            float(motor.get("wing", 0.0)), float(motor.get("throttle", 0.0)),
            1.0 if cmd.wall_contact else 0.0,
        ]
        self.motor_pub.publish(arr)

        if cmd.wall_contact and not self._was_contact:
            self.contact_count += 1
            self.get_logger().warn(
                f"wall contact #{self.contact_count}: lateral={cmd.lateral:+.2f} m "
                f"(half-width {cmd.half_width:.2f} m, margin {self.wall_margin:.2f})")
            if self.reset_on_wall:
                with self.lock:
                    self.reset_requested = True
        self._was_contact = cmd.wall_contact
        self.contact_pub.publish(Bool(data=bool(cmd.wall_contact)))

        self._update_penalty(x, y, cmd.s)

    def _update_penalty(self, x: float, y: float, s: float) -> None:
        """Lap bookkeeping + a conservative cone-hit time penalty.

        The simulator's own collision tracker only publishes proximity, not a
        race penalty. This uses the real cone positions and a radius that
        covers the car's half-width plus a cone, so it matches what a judge
        would call a hit.
        """
        if self._last_s is not None and s < self._last_s - self.track.length * 0.5:
            now = time.time()
            lap_time = now - self._lap_t0
            # The car spawns just before the start line and crosses it almost
            # immediately; that is not a lap.
            if lap_time >= self.lap_min_s:
                self.laps += 1
                self._last_lap_time = lap_time
                self.get_logger().info(
                    f"lap {self.laps} complete: {lap_time:.1f}s, "
                    f"{self.cone_hits} cone hits, {self.penalty_s:.1f}s penalty")
            # Either a real lap or the initial start-line crossing: start a
            # fresh per-lap penalty tally.
            self.cone_hits = 0
            self.penalty_s = 0.0
            self._lap_t0 = now
            self._hit_cones.clear()
        self._last_s = s

        r2 = self.cone_hit_radius * self.cone_hit_radius
        for i, (tag, cx, cy) in enumerate(self.track.cones):
            if i in self._hit_cones:
                continue
            if (cx - x) * (cx - x) + (cy - y) * (cy - y) <= r2:
                self._hit_cones.add(i)
                self.cone_hits += 1
                self.penalty_s += self.cone_penalty_s
                self.get_logger().warn(
                    f"cone hit #{self.cone_hits} ({tag}) at "
                    f"({x:.1f},{y:.1f}); penalty {self.penalty_s:.1f}s")

        self.cone_hits_pub.publish(Int32(data=int(self.cone_hits)))
        self.penalty_pub.publish(Float32(data=float(self.penalty_s)))
        self.lap_pub.publish(Int32(data=int(self.laps)))

    def _fly_direct(self, x, y, yaw, v, fly_ok, fly_steer, fly_throttle) -> Command:
        """No pure-pursuit: the fly's motor signal is the steering command."""
        s, lat, dist = self.track.project(x, y)
        half = self.track.half_width_at(s)
        contact = False
        target = 0.0
        steer = 0.0
        if fly_ok:
            steer = fly_steer * self.max_steer
            target = 0.8 + 4.0 * fly_throttle
        accel = 2.0 * (target - v)
        if self.wall_guard:
            limit = max(0.35, half - self.car_half_width - self.wall_margin)
            if abs(lat) > limit:
                contact = True
                steer = -math.copysign(self.wall_steer, lat)
                accel = -self.wall_brake
                target = 0.0
        steer = max(-self.max_steer, min(self.max_steer, steer))
        accel = max(-self.max_brake, min(self.max_accel, accel))
        return Command(steer, accel, target, lat, half, s, dist,
                       contact, bool(fly_ok))

    # ------------------------------------------------------------ viz
    def _on_viz(self) -> None:
        now = self.get_clock().now().to_msg()
        with self.lock:
            brain, brain_serial = self.brain_jpeg, self.brain_serial
            body, body_serial = self.body_jpeg, self.body_serial
            retina, retina_serial = self.retina, self.retina_serial

        if brain and brain_serial != self._seen["brain"]:
            self._seen["brain"] = brain_serial
            m = CompressedImage()
            m.header.stamp = now
            m.header.frame_id = "map"
            m.format = "jpeg"
            m.data = brain
            self.brain_pub.publish(m)
        if body and body_serial != self._seen["body"]:
            self._seen["body"] = body_serial
            m = CompressedImage()
            m.header.stamp = now
            m.header.frame_id = "map"
            m.format = "jpeg"
            m.data = body
            self.body_pub.publish(m)
        if retina and retina_serial != self._seen["retina"]:
            self._seen["retina"] = retina_serial
            m = Image()
            m.header.stamp = now
            m.header.frame_id = "base_footprint"
            m.height = self.vision_h
            m.width = self.vision_w
            m.encoding = "mono8"
            m.is_bigendian = 0
            m.step = self.vision_w
            m.data = retina
            self.retina_pub.publish(m)

        if time.time() - self._last_walls > 5.0:
            self._publish_walls()
        if (self.scenery is not None
                and time.time() - self._last_scenery > self.scenery_rate):
            self._publish_scenery()

    def _publish_scenery(self) -> None:
        """Republish the static road/barrier/cone/tree markers (latched)."""
        if self.scenery is None:
            return
        self._last_scenery = time.time()
        stamp = self.get_clock().now().to_msg()
        for m in self.scenery.markers:
            m.header.stamp = stamp
        self.scenery_pub.publish(self.scenery)

    def _publish_walls(self) -> None:
        self._last_walls = time.time()
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        mid = 0
        for _name, wall in self.track.walls.items():
            for line in wall.polylines:
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = stamp
                m.ns = f"wall_{wall.name}"
                m.id = mid
                mid += 1
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.12
                m.color.r, m.color.g, m.color.b, m.color.a = wall.color
                m.pose.orientation.w = 1.0
                m.points = [Point(x=float(px), y=float(py), z=0.0)
                            for px, py in line]
                arr.markers.append(m)

        # current car position, handy in Foxglove
        with self.lock:
            odom = self.odom
        if odom is not None:
            car = Marker()
            car.header.frame_id = "map"
            car.header.stamp = stamp
            car.ns = "fly_car"
            car.id = 0
            car.type = Marker.SPHERE
            car.action = Marker.ADD
            car.scale.x = car.scale.y = car.scale.z = 0.6
            car.color.r, car.color.g, car.color.b, car.color.a = 0.2, 1.0, 0.4, 0.8
            car.pose.position.x = float(odom[0])
            car.pose.position.y = float(odom[1])
            car.pose.orientation.w = 1.0
            arr.markers.append(car)
        self.walls_pub.publish(arr)

    # ------------------------------------------------------------ status
    def _on_status(self) -> None:
        with self.lock:
            snap = dict(self.fly_snapshot)
            odom = self.odom
            fly_connected = self.fly_connected
            fly_post_ok = self.fly_post_ok
        motor = snap.get("motor", {}) if snap else {}
        motor_t = float(motor.get("t", 0.0)) if motor else 0.0
        motor_age = (round(time.time() - motor_t, 2) if motor_t else None)
        payload = {
            "mode": self.mode,
            "fly_url": self.fly_url,
            "fly_connected": fly_connected,
            "fly_post_ok": fly_post_ok,
            "fly_frame_id": snap.get("frame_id"),
            "fly_age_s": motor_age,
            "motor": motor,
            "speed": round(odom[3], 2) if odom else None,
            "contact_count": self.contact_count,
            "cone_hits": self.cone_hits,
            "penalty_s": round(self.penalty_s, 1),
            "laps": self.laps,
            "last_lap_s": (round(self._last_lap_time, 1)
                           if self._last_lap_time is not None else None),
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

        if time.time() - self._last_log > 5.0:
            self._last_log = time.time()
            self.get_logger().info(
                f"mode={self.mode} fly={'on' if fly_connected else 'off'} "
                f"steer={motor.get('steer', 0):+.2f} "
                f"thr={motor.get('throttle', 0):.2f} "
                f"speed={payload['speed']} contacts={self.contact_count} "
                f"hits={self.cone_hits} penalty={self.penalty_s:.0f}s "
                f"laps={self.laps}")

    def shutdown(self) -> None:
        self.worker.stop()
        self.worker.join(timeout=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
