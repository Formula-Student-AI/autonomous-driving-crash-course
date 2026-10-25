"""Pure-pursuit assist + virtual wall guard for the fly-driven car.

The fly's connectome is not a trained driver.  It supplies a low-frequency
steering bias and a speed wish; this module keeps the car on the centreline and
turns the cone walls into a hard boundary.

Steering sign follows ROS/Ackermann: positive angle = left.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .track import Track


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class ControlConfig:
    lookahead: float = 4.0
    wheelbase: float = 1.58
    target_speed: float = 3.0
    max_steer: float = 0.5
    max_accel: float = 3.0
    max_brake: float = 5.0
    speed_p_gain: float = 2.0
    curvature_slowdown: float = 1.0
    max_lat_accel: float = 4.0

    # fly influence
    fly_steer_gain: float = 0.18      # rad per unit of fly steer
    fly_speed_gain: float = 2.0       # m/s per unit of (throttle - 0.5)
    fly_require_fresh_s: float = 2.0  # ignore motor data older than this

    # virtual walls
    wall_margin: float = 0.35         # metres inside the cone line
    wall_steer: float = 0.45          # rad of recovery steer on contact
    wall_brake: float = 5.0


@dataclass
class Command:
    steer: float
    accel: float
    target_speed: float
    lateral: float
    half_width: float
    s: float
    distance: float
    wall_contact: bool
    fly_used: bool


class PurePursuit:
    def __init__(self, track: Track, cfg: ControlConfig) -> None:
        self.track = track
        self.cfg = cfg

    def command(
        self,
        x: float,
        y: float,
        yaw: float,
        speed: float,
        fly_steer: float = 0.0,
        fly_throttle: float | None = None,
        fly_ok: bool = False,
        wall_guard: bool = True,
    ) -> Command:
        cfg = self.cfg
        s, lat, dist = self.track.project(x, y)
        half = self.track.half_width_at(s)

        # ---------------------------------------------------------- pursuit
        s_goal = (s + cfg.lookahead) % self.track.length
        gx, gy = self.track.point_at(s_goal)
        dx, dy = gx - x, gy - y
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # If the lookahead point is behind us (large cross-track error), fall
        # back to the tangent heading instead of steering backwards.
        if lx <= 0.1:
            heading = self.track.heading_at(s)
            ly = math.sin(wrap_pi(heading - yaw)) * max(1.0, abs(dx) + abs(dy))
            lx = 1.0
        steer = math.atan2(2.0 * cfg.wheelbase * ly,
                           max(cfg.lookahead ** 2, 1e-3))

        # ---------------------------------------------------------- speed
        target = float(cfg.target_speed)
        dtheta = abs(wrap_pi(self.track.heading_at(s_goal)
                             - self.track.heading_at(s)))
        curvature = dtheta / max(cfg.lookahead, 1e-3)
        if curvature > 1e-3:
            v_curve = math.sqrt(cfg.max_lat_accel / curvature)
            target = min(target, v_curve)
        target = max(1.0, target)

        # ---------------------------------------------------------- fly bias
        fly_used = bool(fly_ok)
        if fly_used:
            steer += cfg.fly_steer_gain * float(fly_steer)
            if fly_throttle is not None:
                target += cfg.fly_speed_gain * (float(fly_throttle) - 0.5)
                target = max(0.8, target)

        accel = cfg.speed_p_gain * (target - speed)

        # ---------------------------------------------------------- walls
        contact = False
        if wall_guard:
            limit = max(0.45, half - cfg.wall_margin)
            if abs(lat) > limit:
                contact = True
                steer = -math.copysign(cfg.wall_steer, lat)
                accel = -cfg.wall_brake
                target = 0.0

        steer = max(-cfg.max_steer, min(cfg.max_steer, steer))
        accel = max(-cfg.max_brake, min(cfg.max_accel, accel))
        return Command(steer, accel, target, lat, half, s, dist,
                       contact, fly_used)
