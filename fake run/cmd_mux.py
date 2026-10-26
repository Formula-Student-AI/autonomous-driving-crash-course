#!/usr/bin/env python3
"""Command mux: exactly one controller may drive the car.

    /race/cmd  (fixed_path_driver.py)  ─┐
                                        ├──> /cmd   (eufs_sim2)
    /fly/cmd   (fly_bridge)            ─┘

Mode is chosen by the `mode` parameter or by publishing `/control_mode`
(`race`, `fly`, `none`).  The default is `race`, so starting the fun fly bridge
by accident cannot move the car.

    python3 "fake run/cmd_mux.py"                          # race (default)
    python3 "fake run/cmd_mux.py" --ros-args -p mode:=fly  # fruit-fly mode

If the selected source stops publishing for `stale_s`, the mux commands a
brake.  It never republishes a stale command.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

VALID_MODES = ("race", "fly", "none")


class CmdMux(Node):
    def __init__(self) -> None:
        super().__init__("cmd_mux")

        # Only one mux may own /cmd. A second one would publish a second
        # stream and fight the first; refuse to start instead.
        self._lock = None
        try:
            import fcntl
            self._lock = open("/tmp/eufs_cmd_mux.lock", "w")
            try:
                fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                self.get_logger().error(
                    "another cmd_mux is already running; refusing to start")
                raise SystemExit(1)
        except ImportError:
            # Windows has no fcntl; the course workflow runs the mux in the
            # container, where this guard is active.
            pass

        self.mode = str(self.declare_parameter("mode", "race").value).lower()
        if self.mode not in VALID_MODES:
            self.get_logger().warn(
                f"unknown mode {self.mode!r}; using race")
            self.mode = "race"
        self.race_topic = str(self.declare_parameter(
            "race_topic", "/race/cmd").value)
        self.fly_topic = str(self.declare_parameter(
            "fly_topic", "/fly/cmd").value)
        self.out_topic = str(self.declare_parameter(
            "out_topic", "/cmd").value)
        self.stale_s = float(self.declare_parameter("stale_s", 0.25).value)
        self.rate = float(self.declare_parameter("rate", 20.0).value)

        self.race_msg: AckermannDriveStamped | None = None
        self.race_t = 0.0
        self.fly_msg: AckermannDriveStamped | None = None
        self.fly_t = 0.0

        self.create_subscription(AckermannDriveStamped, self.race_topic,
                                 self._on_race, 10)
        self.create_subscription(AckermannDriveStamped, self.fly_topic,
                                 self._on_fly, 10)
        self.create_subscription(String, "/control_mode", self._on_mode, 10)

        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, self.out_topic, 10)
        self.mode_pub = self.create_publisher(
            String, "/control_mode_active", 10)
        self.create_timer(1.0 / max(1.0, self.rate), self._step)
        self.get_logger().info(
            f"cmd mux: mode={self.mode}  {self.race_topic} | {self.fly_topic} "
            f"-> {self.out_topic}  (brake if stale > {self.stale_s:.2f}s)")

    # ------------------------------------------------------------ inputs
    def _on_race(self, msg: AckermannDriveStamped) -> None:
        self.race_msg = msg
        self.race_t = time.time()

    def _on_fly(self, msg: AckermannDriveStamped) -> None:
        self.fly_msg = msg
        self.fly_t = time.time()

    def _on_mode(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode not in VALID_MODES:
            self.get_logger().warn(
                f"/control_mode: {mode!r} not in {VALID_MODES}")
            return
        if mode != self.mode:
            self.get_logger().info(f"cmd mux mode -> {mode}")
        self.mode = mode

    # ------------------------------------------------------------ output
    def _step(self) -> None:
        now = time.time()
        chosen: AckermannDriveStamped | None = None
        source = "-"
        if self.mode == "race" and self.race_msg is not None \
                and now - self.race_t <= self.stale_s:
            chosen, source = self.race_msg, "race"
        elif self.mode == "fly" and self.fly_msg is not None \
                and now - self.fly_t <= self.stale_s:
            chosen, source = self.fly_msg, "fly"

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        if chosen is not None:
            out.drive.acceleration = float(chosen.drive.acceleration)
            out.drive.steering_angle = float(chosen.drive.steering_angle)
        else:
            # safe fallback: brake; the sim's 0.3 s deadman would also catch it
            out.drive.acceleration = -5.0
            out.drive.steering_angle = 0.0
        self.cmd_pub.publish(out)
        self.mode_pub.publish(String(data=f"{self.mode}:{source}"))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdMux()
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
