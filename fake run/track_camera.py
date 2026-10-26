#!/usr/bin/env python3
"""Realistic first-person OpenCV camera for the main race stack.

    python3 "fake run/track_camera.py"

Renders sky, grass, the asphalt road, white edge lines, barriers, tapered
cones, trees and a small HUD, then publishes the frame as a ROS image:

    /race/camera   sensor_msgs/Image (bgr8)   ~15 Hz

It uses `/odom` for the pose and the track CSV for the scene.  This is a
visualisation only; it does not touch /cmd.
"""
from __future__ import annotations

import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, Int32

from track_geometry import Track


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


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# Six cameras, 60 deg apart, 60 deg horizontal FOV each (360 deg total).
# Positive offset = left of the car; names follow the car's perspective.
PERCEPTION_CAMERAS = [
    ("front", 0.0),
    ("front_left", 60.0),
    ("rear_left", 120.0),
    ("rear", 180.0),
    ("rear_right", 240.0),
    ("front_right", 300.0),
]


class TrackCamera(Node):
    def __init__(self) -> None:
        super().__init__("track_camera")
        here = os.path.dirname(os.path.abspath(__file__))
        self.track_file = str(self.declare_parameter(
            "track_file", default_track_file()).value)
        self.centerline_file = str(self.declare_parameter(
            "centerline_file", os.path.join(here, "fixed_path_centerline.csv")).value)
        self.W = int(self.declare_parameter("width", 640).value)
        self.H = int(self.declare_parameter("height", 360).value)
        self.fps = float(self.declare_parameter("fps", 15.0).value)
        self.hfov = float(self.declare_parameter("hfov", 1.91889).value)
        self.camera_height = float(self.declare_parameter(
            "camera_height", 1.15).value)
        self.pitch = float(self.declare_parameter("pitch", 0.08).value)
        self.max_range = float(self.declare_parameter(
            "max_range", 45.0).value)
        self.show = bool(self.declare_parameter("show", False).value)
        self.num_cameras = int(self.declare_parameter("num_cameras", 6).value)
        self.pw = int(self.declare_parameter("perception_width", 320).value)
        self.ph = int(self.declare_parameter("perception_height", 180).value)
        self.perception_fov = float(self.declare_parameter(
            "perception_fov", math.pi / 3.0).value)
        self.render_front = bool(self.declare_parameter(
            "render_front", True).value)
        self.perception = PERCEPTION_CAMERAS[:max(0, min(6, self.num_cameras))]
        self.perception_f = (self.pw / 2.0) / math.tan(
            max(1e-3, self.perception_fov) / 2.0)

        self.track = Track(self.track_file, self.centerline_file)
        self.x = self.y = self.yaw = self.v = 0.0
        self.have_odom = False
        self.hits = 0
        self.penalty = 0.0

        self.f = (self.W / 2.0) / math.tan(self.hfov / 2.0)
        self.horizon = int(self.H / 2.0 - self.f * math.tan(self.pitch))
        self.near = 0.8
        self._c = math.cos(self.pitch)
        self._s = math.sin(self.pitch)

        # centreline samples for the road / barriers
        self.samples = self._center_samples()
        self.trees = self._tree_positions()

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Int32, "/race/cone_hits", self.on_hits, 10)
        self.create_subscription(Float32, "/race/penalty_s", self.on_penalty, 10)
        self.pub = self.create_publisher(Image, "/race/camera", 1)
        self.info_pub = self.create_publisher(
            CameraInfo, "/race/camera_info", 1)
        self.info = self._make_info(self.W, self.H, self.f)

        self.perception_pubs = {}
        self.perception_info_pubs = {}
        self.perception_infos = {}
        for name, _deg in self.perception:
            self.perception_pubs[name] = self.create_publisher(
                Image, f"/perception/camera/{name}/image", 1)
            self.perception_info_pubs[name] = self.create_publisher(
                CameraInfo, f"/perception/camera/{name}/camera_info", 1)
            self.perception_infos[name] = self._make_info(
                self.pw, self.ph, self.perception_f)

        self.create_timer(1.0, self.publish_info)
        self.publish_info()
        self.create_timer(1.0 / max(1.0, self.fps), self.render)
        self.get_logger().info(
            f"front {self.W}x{self.H}@{self.fps:g}Hz -> /race/camera; "
            f"{len(self.perception)} perception cameras "
            f"{self.pw}x{self.ph} at {math.degrees(self.perception_fov):.0f} deg "
            f"FOV -> /perception/camera/<name>/image  "
            f"(h={self.camera_height:.2f} pitch={self.pitch:.2f})")

    def _make_info(self, width: int, height: int, focal: float) -> CameraInfo:
        info = CameraInfo()
        info.header.frame_id = "base_footprint"
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [focal, 0.0, width / 2.0,
                  0.0, focal, height / 2.0,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [focal, 0.0, width / 2.0, 0.0,
                  0.0, focal, height / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    # ------------------------------------------------------------ setup
    def _center_samples(self):
        pts = self.track.center
        n = len(pts)
        s = [0.0] * n
        for i in range(1, n):
            s[i] = s[i - 1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                         pts[i][1] - pts[i - 1][1])
        out = []
        for i in range(n):
            x, y = pts[i]
            qx, qy = pts[(i - 1) % n]
            rx, ry = pts[(i + 1) % n]
            tx, ty = rx - qx, ry - qy
            length = math.hypot(tx, ty)
            if length < 1e-9:
                tx, ty, length = 1.0, 0.0, 1.0
            nx, ny = -ty / length, tx / length
            out.append((x, y, s[i], nx, ny, self.track.half_width_at(s[i])))
        return out

    def _tree_positions(self):
        trees = []
        n = len(self.samples)
        stride = max(3, n // 16)
        for k in range(0, n, stride):
            x, y, _s, nx, ny, half = self.samples[k]
            for side in (+1, -1):
                jitter = 2.0 + 4.0 * (((k * 37 + (0 if side > 0 else 7)) % 13) / 13.0)
                off = half + 8.0 + jitter
                tx = x + nx * off * side
                ty = y + ny * off * side
                h = 2.8 + 1.4 * (((k * 19) % 7) / 7.0)
                r = 1.5 + 0.7 * (((k * 11) % 7) / 7.0)
                trees.append((tx, ty, h, r))
        return trees

    # ------------------------------------------------------------ inputs
    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.x, self.y = float(p.x), float(p.y)
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.v = float(msg.twist.twist.linear.x)
        self.have_odom = True

    def on_hits(self, msg: Int32) -> None:
        self.hits = int(msg.data)

    def on_penalty(self, msg: Float32) -> None:
        self.penalty = float(msg.data)

    # ------------------------------------------------------------ projection
    def _cam(self, px: float, py: float, pz: float):
        dx, dy = px - self.x, py - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        fx = c * dx + s * dy
        fy = -s * dx + c * dy
        zr = pz - self.camera_height
        zc = fx * self._c - zr * self._s
        yc = fx * self._s + zr * self._c
        xc = -fy
        return xc, yc, zc

    def _project(self, px: float, py: float, pz: float):
        xc, yc, zc = self._cam(px, py, pz)
        if zc <= self.near or zc > self.max_range:
            return None
        u = self.W / 2.0 + self.f * xc / zc
        v = self.H / 2.0 - self.f * yc / zc
        if abs(u) > 8000 or abs(v) > 8000:
            return None
        return int(round(u)), int(round(v)), zc

    def _quad(self, img, world_pts, color):
        pts = []
        for p in world_pts:
            q = self._project(*p)
            if q is None:
                return
            pts.append((q[0], q[1]))
        cv2.fillPoly(img, [np.array(pts, dtype=np.int32)], color, lineType=cv2.LINE_AA)

    # ------------------------------------------------------------ render
    def render(self) -> None:
        img = np.empty((self.H, self.W, 3), dtype=np.uint8)

        # sky gradient and grass
        horizon = max(1, min(self.H - 1, self.horizon))
        top = np.array([205, 160, 105], dtype=np.float32)      # BGR, deeper blue
        hor = np.array([240, 215, 170], dtype=np.float32)      # BGR, pale horizon
        for y in range(horizon):
            t = y / max(1, horizon - 1)
            img[y, :] = (top * (1.0 - t) + hor * t).astype(np.uint8)
        img[horizon:, :] = (42, 110, 42)

        if not self.have_odom:
            self._hud(img, "waiting for /odom")
            self._publish(img)
            return

        # road quads
        left, right = [], []
        for x, y, _s, nx, ny, half in self.samples:
            left.append((x + nx * half, y + ny * half))
            right.append((x - nx * half, y - ny * half))
        n = len(left)
        for i in range(n):
            j = (i + 1) % n
            self._quad(img, [(left[i][0], left[i][1], 0.02),
                             (right[i][0], right[i][1], 0.02),
                             (right[j][0], right[j][1], 0.02),
                             (left[j][0], left[j][1], 0.02)],
                       (60, 60, 62))
        # edge lines
        for side in (left, right):
            prev = None
            for px, py in side:
                q = self._project(px, py, 0.03)
                if prev is not None and q is not None:
                    cv2.line(img, (prev[0], prev[1]), (q[0], q[1]),
                             (235, 235, 235), 2, cv2.LINE_AA)
                prev = q

        # barriers: vertical wall + red top rail
        for side, sign in ((left, +1), (right, -1)):
            prev = None
            for (x, y, _s, nx, ny, _half), (ex, ey) in zip(self.samples, side):
                bx = ex + nx * 0.65 * sign
                by = ey + ny * 0.65 * sign
                qb = self._project(bx, by, 0.0)
                qt = self._project(bx, by, 0.45)
                cur = (qb, qt)
                if prev is not None and all(v is not None for v in (prev[0], prev[1], qb, qt)):
                    pts = [(prev[0][0], prev[0][1]), (qb[0], qb[1]),
                           (qt[0], qt[1]), (prev[1][0], prev[1][1])]
                    cv2.fillPoly(img, [np.array(pts, dtype=np.int32)],
                                 (180, 180, 185), lineType=cv2.LINE_AA)
                    cv2.line(img, (prev[1][0], prev[1][1]), (qt[0], qt[1]),
                             (40, 40, 200), 2, cv2.LINE_AA)
                prev = cur

        # trees (far to near)
        tree_items = []
        for tx, ty, h, r in self.trees:
            qb = self._project(tx, ty, 0.0)
            qc = self._project(tx, ty, h + r * 0.5)
            if qb is not None and qc is not None and 0 <= qb[0] < self.W:
                tree_items.append((qb, qc, r / max(qb[2], 1.0)))
        for qb, qc, scale in sorted(tree_items, key=lambda t: t[0][2], reverse=True):
            cv2.line(img, (qb[0], qb[1]), (qc[0], qc[1]), (32, 70, 110),
                     max(2, int(self.f * 0.28 * scale)), cv2.LINE_AA)
            cv2.circle(img, (qc[0], qc[1]), max(3, int(self.f * scale)),
                       (30, 130, 40), -1, cv2.LINE_AA)

        # cones (far to near)
        colors = {"blue": (255, 80, 0), "yellow": (0, 230, 255),
                  "orange": (0, 150, 255)}
        cone_items = []
        for tag, px, py in self.track.cones:
            qb = self._project(px, py, 0.0)
            qt = self._project(px, py, 0.32)
            if qb is not None and qt is not None and -40 <= qb[0] <= self.W + 40:
                cone_items.append((qb, qt, colors.get(tag, (200, 200, 200))))
        for qb, qt, color in sorted(cone_items, key=lambda c: c[0][2], reverse=True):
            u, v0 = qb[0], qb[1]
            bw = max(2, min(40, int(0.14 * self.f / max(qb[2], 1.0))))
            pts = np.array([(u - bw, v0), (u + bw, v0), (u, qt[1])], dtype=np.int32)
            cv2.fillPoly(img, [pts], color, lineType=cv2.LINE_AA)
            band_y = int(v0 - 0.55 * (v0 - qt[1]))
            cv2.line(img, (u - max(1, bw // 2), band_y),
                     (u + max(1, bw // 2), band_y), (245, 245, 245),
                     max(1, bw // 4), cv2.LINE_AA)
            cv2.ellipse(img, (u, v0), (bw, max(1, bw // 4)), 0, 0, 360,
                        (25, 25, 25), -1, cv2.LINE_AA)

        self._hud(img, f"v {self.v:4.1f} m/s   hits {self.hits}   "
                       f"penalty {self.penalty:4.1f} s")
        self._publish(img)

    def _hud(self, img, text: str) -> None:
        cv2.rectangle(img, (8, 8), (self.W - 8, 36), (0, 0, 0), -1)
        cv2.putText(img, text, (16, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (235, 235, 235), 1, cv2.LINE_AA)

    def publish_info(self) -> None:
        self.info.header.stamp = self.get_clock().now().to_msg()
        try:
            self.info_pub.publish(self.info)
        except Exception:  # noqa: BLE001 - context shutting down
            pass

    def _publish(self, img) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.height = self.H
        msg.width = self.W
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = self.W * 3
        msg.data = img.tobytes()
        try:
            self.pub.publish(msg)
        except Exception:  # noqa: BLE001 - context shutting down
            return
        if self.show:
            try:
                cv2.imshow("race camera", img)
                cv2.waitKey(1)
            except cv2.error:
                self.show = False

    def destroy_node(self) -> bool:
        if self.show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackCamera()
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
