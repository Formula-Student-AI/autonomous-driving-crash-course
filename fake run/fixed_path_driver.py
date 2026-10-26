#!/usr/bin/env python3
"""Fixed-path driver for EUFS sim2 (FSUK 2023 trackdrive) — main race stack.

Improvements over the simple pure-pursuit baseline:

* speed-dependent lookahead (short at low speed) so it stops cutting corners,
* Stanley-style cross-track correction,
* a wall limit based on the **car width**, not just the centre:
  ``|lateral| <= half_width - car_half_width - wall_margin``,
* heading-based prediction (``v * sin(yaw - heading)``) so a wall hit is seen
  coming; a prediction slows, an actual violation brakes,
* curvature / lateral-error speed reduction,
* cone-hit time penalty (``/race/cone_hits``, ``/race/penalty_s``, `/race/lap``).

The driver publishes ``/race/cmd`` by default.  Run ``fake run/cmd_mux.py``
(race mode) to forward it to ``/cmd``; this keeps the fruit-fly fun mode
(``/fly/cmd``) from ever fighting the race controller.  For the old standalone
behaviour use ``--ros-args -p cmd_topic:=/cmd``.

Run AFTER: /set_mission {mission: 4} + /go
"""
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32

from track_geometry import Track


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def default_track_file() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("map_lib"),
            "maps", "competitions", "FSUK", "2023", "trackdrive.csv")
    except Exception:  # noqa: BLE001
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(
            here, "..", "sim", "src", "map_lib", "maps",
            "competitions", "FSUK", "2023", "trackdrive.csv"))


class FixedPathDriver(Node):
    def __init__(self) -> None:
        super().__init__("fixed_path_driver")
        here = os.path.dirname(os.path.abspath(__file__))

        # ---------------------------------------------------------- paths
        self.path_file = str(self.declare_parameter(
            "path_file", os.path.join(here, "fixed_path_centerline.csv")).value)
        self.track_file = str(self.declare_parameter(
            "track_file", default_track_file()).value)
        if not os.path.isfile(self.path_file):
            raise FileNotFoundError(f"centreline not found: {self.path_file}")
        if not os.path.isfile(self.track_file):
            raise FileNotFoundError(
                f"track file not found: {self.track_file}\n"
                "Pass -p track_file:=/path/to/trackdrive.csv")
        self.track = Track(self.track_file, self.path_file)
        self.get_logger().info(self.track.summary())

        # ---------------------------------------------------------- tuning
        self.cmd_topic = str(self.declare_parameter(
            "cmd_topic", "/race/cmd").value)
        self.lookahead = float(self.declare_parameter("lookahead", 4.0).value)
        self.lookahead_min = float(self.declare_parameter(
            "lookahead_min", 2.5).value)
        self.lookahead_gain = float(self.declare_parameter(
            "lookahead_gain", 0.35).value)
        self.target_speed = float(self.declare_parameter(
            "target_speed", 4.0).value)
        self.wheelbase = float(self.declare_parameter("wheelbase", 1.58).value)
        self.max_steer = float(self.declare_parameter("max_steer", 0.5).value)
        self.max_accel = float(self.declare_parameter("max_accel", 3.0).value)
        self.max_brake = float(self.declare_parameter("max_brake", 5.0).value)
        self.k_lat = float(self.declare_parameter("k_lat", 0.6).value)
        self.car_half_width = float(self.declare_parameter(
            "car_half_width", 0.75).value)
        self.car_length = float(self.declare_parameter(
            "car_length", 2.4).value)
        self.wall_margin = float(self.declare_parameter(
            "wall_margin", 0.35).value)
        self.predict_horizon = float(self.declare_parameter(
            "predict_horizon", 0.4).value)
        self.predict_margin = float(self.declare_parameter(
            "predict_margin", 0.15).value)
        self.lat_slowdown = float(self.declare_parameter(
            "lat_slowdown", 0.5).value)
        self.max_lat_accel = float(self.declare_parameter(
            "max_lat_accel", 4.0).value)
        self.cone_hit_radius = float(self.declare_parameter(
            "cone_hit_radius", 0.95).value)
        self.cone_penalty_s = float(self.declare_parameter(
            "cone_penalty_s", 5.0).value)
        self.lap_min_s = float(self.declare_parameter("lap_min_s", 10.0).value)

        # ---------------------------------------------------------- state
        self.x = self.y = self.yaw = self.v = 0.0
        self.have_odom = False
        self._last_s: float | None = None
        self._lap_t0 = 0.0
        self._last_lap_time: float | None = None
        self.laps = 0
        self.cone_hits = 0
        self.penalty_s = 0.0
        self._hit_cones: set[int] = set()
        self._last_log = 0.0

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odom, 10)
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, self.cmd_topic, 10)
        self.hits_pub = self.create_publisher(Int32, "/race/cone_hits", 10)
        self.penalty_pub = self.create_publisher(Float32, "/race/penalty_s", 10)
        self.lap_pub = self.create_publisher(Int32, "/race/lap", 10)
        self.timer = self.create_timer(0.05, self.step)  # 20 Hz
        self.get_logger().info(
            f"driver -> {self.cmd_topic}  target={self.target_speed:.1f} m/s  "
            f"car={self.car_length:.1f}x{2*self.car_half_width:.1f} m  "
            f"wall_margin={self.wall_margin:.2f}")

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.v = float(msg.twist.twist.linear.x)
        self.have_odom = True

    def step(self) -> None:
        if not self.have_odom:
            return
        track = self.track
        s, lat, _dist = track.project(self.x, self.y)
        half = track.half_width_at(s)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self._lap_t0 == 0.0:
            self._lap_t0 = now_s

        # ------------------------------------------------------ pursuit
        ld = min(self.lookahead,
                 max(self.lookahead_min,
                     self.lookahead_min + self.lookahead_gain * self.v))
        gx, gy = track.point_at(s + ld)
        dx, dy = gx - self.x, gy - self.y
        lx = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        ly = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
        if lx <= 0.1:
            heading = track.heading_at(s)
            ly = math.sin(wrap_pi(heading - self.yaw)) * max(
                1.0, abs(dx) + abs(dy))
            lx = 1.0
        steer = math.atan2(2.0 * self.wheelbase * ly, max(ld * ld, 1e-3))

        # ------------------------------------------------------ speed
        target = self.target_speed
        dtheta = abs(wrap_pi(track.heading_at(s + ld) - track.heading_at(s)))
        curvature = dtheta / max(ld, 1e-3)
        if curvature > 1e-3:
            target = min(target, math.sqrt(self.max_lat_accel / curvature))
        target = max(1.0, target)

        # ------------------------------------------------------ tracking
        limit = max(0.35, half - self.car_half_width - self.wall_margin)
        if abs(lat) > 1e-3:
            steer -= self.k_lat * math.atan2(lat, max(self.v, 0.8))
        # A car is not a point: its front/rear corners can leave the cone
        # corridor even when the centre looks fine. Check all four corners.
        fp_contact, fp_ratio, fp_lat = self._footprint(self.x, self.y, self.yaw)
        if limit > 1e-3:
            edge = max(0.0, min(1.0,
                               (abs(lat) - 0.4 * limit) / (0.6 * limit)))
            if fp_ratio > 0.8:
                edge = max(edge, min(1.0, (fp_ratio - 0.8) / 0.2))
            target *= 1.0 - self.lat_slowdown * edge
        target = max(0.8, target)
        accel = 2.0 * (target - self.v)

        # ------------------------------------------------------ walls
        heading = track.heading_at(s)
        lat_rate = self.v * math.sin(wrap_pi(self.yaw - heading))
        lat_rate = max(-3.0, min(3.0, lat_rate))
        lat_pred = lat + lat_rate * self.predict_horizon
        over_now = abs(lat) > limit or fp_contact
        over_pred = self.v > 0.5 and abs(lat_pred) > limit + self.predict_margin
        if over_now or over_pred:
            if abs(lat) > 1e-2:
                ref = lat
            elif abs(fp_lat) > 1e-2:
                ref = fp_lat
            else:
                ref = lat_pred
            steer = -math.copysign(0.45, ref)
            if over_now:
                accel = -self.max_brake
                target = 0.0
            else:
                accel = min(accel, -2.0)
                target = max(1.0, target * 0.6)

        steer = max(-self.max_steer, min(self.max_steer, steer))
        accel = max(-self.max_brake, min(self.max_accel, accel))

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = float(accel)
        msg.drive.steering_angle = float(steer)
        self.cmd_pub.publish(msg)

        self._update_penalty(now_s, s, limit)
        if now_s - self._last_log > 5.0:
            self._last_log = now_s
            self.get_logger().info(
                f"v={self.v:4.2f} target={target:4.2f} steer={steer:+.2f} "
                f"lat={lat:+.2f}/{limit:.2f} fp={fp_ratio:.2f} "
                f"hits={self.cone_hits} penalty={self.penalty_s:.0f}s "
                f"laps={self.laps}")

    def _footprint(self, x: float, y: float, yaw: float):
        """Check the four car corners against the local wall limits.

        Returns (contact, worst_ratio, worst_lateral). ``worst_ratio`` is
        ``|corner lateral| / (local half-width - wall_margin)``; > 1 means a
        corner is outside the safe corridor.
        """
        half_len = 0.5 * self.car_length
        half_w = self.car_half_width
        c, s = math.cos(yaw), math.sin(yaw)
        contact = False
        worst_ratio = 0.0
        worst_lat = 0.0
        for dx in (half_len, -half_len):
            for dy in (half_w, -half_w):
                px = x + c * dx - s * dy
                py = y + s * dx + c * dy
                cs, clat, _ = self.track.project(px, py)
                chalf = self.track.half_width_at(cs)
                lim = max(0.25, chalf - self.wall_margin)
                ratio = abs(clat) / max(lim, 1e-3)
                if ratio > worst_ratio:
                    worst_ratio = ratio
                    worst_lat = clat
                if abs(clat) > lim:
                    contact = True
        return contact, worst_ratio, worst_lat

    def _update_penalty(self, now_s: float, s: float, limit: float) -> None:
        if self._last_s is not None and s < self._last_s - self.track.length * 0.5:
            lap_time = now_s - self._lap_t0
            if lap_time >= self.lap_min_s:
                self.laps += 1
                self._last_lap_time = lap_time
                self.get_logger().info(
                    f"lap {self.laps} complete: {lap_time:.1f}s, "
                    f"{self.cone_hits} cone hits, {self.penalty_s:.1f}s penalty")
            self.cone_hits = 0
            self.penalty_s = 0.0
            self._lap_t0 = now_s
            self._hit_cones.clear()
        self._last_s = s

        r2 = self.cone_hit_radius * self.cone_hit_radius
        for i, (tag, cx, cy) in enumerate(self.track.cones):
            if i in self._hit_cones:
                continue
            if (cx - self.x) ** 2 + (cy - self.y) ** 2 <= r2:
                self._hit_cones.add(i)
                self.cone_hits += 1
                self.penalty_s += self.cone_penalty_s
                self.get_logger().warn(
                    f"cone hit #{self.cone_hits} ({tag}) at "
                    f"({self.x:.1f},{self.y:.1f}); penalty {self.penalty_s:.1f}s")

        self.hits_pub.publish(Int32(data=int(self.cone_hits)))
        self.penalty_pub.publish(Float32(data=float(self.penalty_s)))
        self.lap_pub.publish(Int32(data=int(self.laps)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedPathDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
