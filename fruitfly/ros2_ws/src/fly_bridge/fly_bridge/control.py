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
    # Speed-dependent lookahead: Ld = clamp(min + gain*v, min, lookahead).
    # A 4 m lookahead at 2-3 m/s cuts corners and clips inside cones.
    lookahead_min: float = 2.5
    lookahead_gain: float = 0.35
    wheelbase: float = 1.58
    target_speed: float = 3.0
    max_steer: float = 0.5
    max_accel: float = 3.0
    max_brake: float = 5.0
    speed_p_gain: float = 2.0
    curvature_slowdown: float = 1.0
    max_lat_accel: float = 4.0

    # Cross-track correction (Stanley-style term, added to pure pursuit).
    k_lat: float = 0.6

    # fly influence
    fly_steer_gain: float = 0.18      # rad per unit of fly steer
    fly_speed_gain: float = 2.0       # m/s per unit of (throttle - 0.5)
    fly_require_fresh_s: float = 2.0  # ignore motor data older than this

    # virtual walls: keep the *whole car* inside the cone line, not its centre
    car_half_width: float = 0.75      # metres, body half-width + tyre allowance
    wall_margin: float = 0.35         # metres between the car edge and the cone line
    wall_steer: float = 0.45          # rad of recovery steer on contact
    wall_brake: float = 5.0
    # predictive guard: use the measured lateral rate to see a hit coming
    predict_horizon: float = 0.4      # s
    predict_margin: float = 0.15      # m of extra warning
    lat_slowdown: float = 0.5         # fraction of target speed lost at the limit


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

        # Predicted lateral offset from the car's heading relative to the
        # track tangent: lateral velocity = v * sin(yaw - heading). This is
        # smooth, unlike differencing the projection (which jumps at corners
        # and at the start-line wrap and used to cause false wall warnings).
        heading = self.track.heading_at(s)
        lat_rate = speed * math.sin(wrap_pi(yaw - heading))
        lat_rate = max(-3.0, min(3.0, lat_rate))
        lat_pred = lat + lat_rate * cfg.predict_horizon

        # ---------------------------------------------------------- pursuit
        # Short lookahead at low speed: a long one cuts the corner and clips
        # the inside cones.
        ld = min(cfg.lookahead,
                 max(cfg.lookahead_min,
                     cfg.lookahead_min + cfg.lookahead_gain * speed))
        s_goal = (s + ld) % self.track.length
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
        steer = math.atan2(2.0 * cfg.wheelbase * ly, max(ld * ld, 1e-3))

        # ---------------------------------------------------------- speed
        target = float(cfg.target_speed)
        dtheta = abs(wrap_pi(self.track.heading_at(s_goal)
                             - self.track.heading_at(s)))
        curvature = dtheta / max(ld, 1e-3)
        if curvature > 1e-3:
            v_curve = math.sqrt(cfg.max_lat_accel / curvature)
            target = min(target, v_curve)
        target = max(1.0, target)

        # ---------------------------------------------------------- tracking
        # The car's centre may be `half - car_half_width - margin` from the
        # centreline; this keeps the *whole car* inside the cone line.
        limit = max(0.35, half - cfg.car_half_width - cfg.wall_margin)
        if abs(lat) > 1e-3:
            # Stanley-style cross-track term: left of the line -> steer right.
            steer -= cfg.k_lat * math.atan2(lat, max(speed, 0.8))
        if limit > 1e-3:
            edge = max(0.0, min(1.0, (abs(lat) - 0.4 * limit) / (0.6 * limit)))
            target *= 1.0 - cfg.lat_slowdown * edge
        target = max(0.8, target)

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
            over_now = abs(lat) > limit
            over_pred = speed > 0.5 and abs(lat_pred) > limit + cfg.predict_margin
            if over_now or over_pred:
                contact = True
                ref = lat if abs(lat) > 1e-2 else lat_pred
                steer = -math.copysign(cfg.wall_steer, ref)
                if over_now:
                    # already over the cone line: hard brake
                    accel = -cfg.wall_brake
                    target = 0.0
                else:
                    # heading toward the wall: slow down but don't stop
                    accel = min(accel, -2.0)
                    target = max(1.0, target * 0.6)

        steer = max(-cfg.max_steer, min(cfg.max_steer, steer))
        accel = max(-cfg.max_brake, min(cfg.max_accel, accel))
        return Command(steer, accel, target, lat, half, s, dist,
                       contact, fly_used)
