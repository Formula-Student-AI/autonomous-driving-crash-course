"""Render the fly's retina: a small grayscale view of the two cone walls.

The bridge does not use a rendered camera from the simulator (the EUFS car has
no image topic).  Instead it projects the ordered wall polylines into a simple
pinhole view in the car frame.  Blue is the left wall, yellow the right wall.

The output is a raw ``width*height`` byte buffer (0..255), which is exactly what
``POST /api/live/frame`` expects.
"""
from __future__ import annotations

import math

from .track import Wall


def _line(buf: bytearray, w: int, h: int,
          x0: int, y0: int, x1: int, y1: int,
          value: int, thickness: int = 1) -> None:
    """Bresenham line with a small square brush, max-composited."""
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    r = max(0, int(thickness))
    while True:
        for ox in range(-r, r + 1):
            xx = x0 + ox
            if xx < 0 or xx >= w:
                continue
            base = xx
            for oy in range(-r, r + 1):
                yy = y0 + oy
                if 0 <= yy < h:
                    idx = yy * w + base
                    if value > buf[idx]:
                        buf[idx] = value
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def _clip_near_far(p0, p1, near: float, far: float):
    """Clip a segment in car-frame (fx, fy) to near <= fx <= far."""
    (fx0, fy0), (fx1, fy1) = p0, p1
    if fx0 < near and fx1 < near:
        return None
    if fx0 > far and fx1 > far:
        return None
    dfx = fx1 - fx0
    t0, t1 = 0.0, 1.0
    if abs(dfx) < 1e-9:
        if fx0 < near or fx0 > far:
            return None
    else:
        ta = (near - fx0) / dfx
        tb = (far - fx0) / dfx
        lo, hi = (ta, tb) if ta <= tb else (tb, ta)
        t0 = max(t0, lo)
        t1 = min(t1, hi)
        if t1 <= t0:
            return None
    ax = fx0 + t0 * dfx
    ay = fy0 + t0 * (fy1 - fy0)
    bx = fx0 + t1 * dfx
    by = fy0 + t1 * (fy1 - fy0)
    return (ax, ay), (bx, by)


def _triangle(buf: bytearray, w: int, h: int,
              p0: tuple[int, int], p1: tuple[int, int], p2: tuple[int, int],
              value: int) -> None:
    """Fill a small triangle (max-composited)."""
    xs = (p0[0], p1[0], p2[0])
    ys = (p0[1], p1[1], p2[1])
    minx = max(0, min(xs))
    maxx = min(w - 1, max(xs))
    miny = max(0, min(ys))
    maxy = min(h - 1, max(ys))
    if maxx < minx or maxy < miny:
        return
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    d = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
    if abs(d) < 1e-9:
        return
    for y in range(miny, maxy + 1):
        for x in range(minx, maxx + 1):
            a = ((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) / d
            b = ((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) / d
            c = 1.0 - a - b
            if a >= -0.15 and b >= -0.15 and c >= -0.15:
                idx = y * w + x
                if value > buf[idx]:
                    buf[idx] = value


def render_view(
    walls: dict[str, Wall],
    x: float,
    y: float,
    yaw: float,
    width: int = 160,
    height: int = 120,
    hfov: float = 1.91889,
    max_range: float = 30.0,
    near: float = 0.6,
    camera_height: float = 0.55,
    thickness: int = 1,
    cones: list[tuple[str, float, float]] | None = None,
    sky: bool = True,
    road: bool = True,
    cone_marks: bool = True,
) -> bytes:
    """Return a grayscale camera-like view (bytes, row-major, width*height).

    The top half is a sky gradient, the bottom is ground; the area between the
    two wall lines is filled as asphalt, and each cone is drawn as a small
    tapered marker with a bright band.
    """
    w = int(width)
    h = int(height)
    buf = bytearray([0]) * (w * h)
    horizon = h // 2

    # ---- sky / ground gradient -----------------------------------------
    if sky:
        for yy in range(h):
            if yy < horizon:
                val = 34 + int(36 * (yy / max(1, horizon)))
            else:
                val = 14
            start = yy * w
            buf[start:start + w] = bytes([val]) * w
    else:
        for i in range(w * h):
            buf[i] = 0

    f_px = (w / 2.0) / math.tan(max(1e-3, hfov) / 2.0)
    cy = h / 2.0
    # world -> car frame: +x forward, +y left
    c, s = math.cos(yaw), math.sin(yaw)

    values = {"blue": 90, "yellow": 235}

    # ---- wall lines -----------------------------------------------------
    for name, wall in walls.items():
        value = values.get(name, 160)
        for line in wall.polylines:
            prev_car = None
            for (px, py) in line:
                dx, dy = px - x, py - y
                car = (c * dx + s * dy, -s * dx + c * dy)  # (fx, fy)
                if prev_car is not None:
                    clipped = _clip_near_far(prev_car, car, near, max_range)
                    if clipped is not None:
                        (ax, ay), (bx, by) = clipped
                        u0 = int(round(w / 2.0 - f_px * (ay / ax)))
                        v0 = int(round(cy + f_px * (camera_height / ax)))
                        u1 = int(round(w / 2.0 - f_px * (by / bx)))
                        v1 = int(round(cy + f_px * (camera_height / bx)))
                        _line(buf, w, h, u0, v0, u1, v1, value, thickness)
                prev_car = car

    # ---- fill the road between the wall lines ---------------------------
    if road:
        for yy in range(horizon, h):
            row = yy * w
            lit = [xx for xx in range(w) if buf[row + xx] >= 80]
            if len(lit) >= 2:
                lo, hi = min(lit), max(lit)
                if hi - lo > 2:
                    for xx in range(lo + 1, hi):
                        if buf[row + xx] < 80:
                            buf[row + xx] = 36

    # ---- cone markers ---------------------------------------------------
    if cone_marks and cones:
        cvals = {"blue": 90, "yellow": 235, "orange": 160}
        for tag, px, py in cones:
            dx, dy = px - x, py - y
            fx = c * dx + s * dy
            fy = -s * dx + c * dy
            if fx < near or fx > max_range:
                continue
            u = int(round(w / 2.0 - f_px * (fy / fx)))
            v = int(round(cy + f_px * (camera_height / fx)))
            if u < -4 or u > w + 4 or v < horizon or v >= h:
                continue
            sz = max(2, min(9, int(20.0 / max(fx, 1.0))))
            bw = max(1, sz // 3)
            val = cvals.get(tag, 160)
            _triangle(buf, w, h, (u - bw, v), (u + bw, v),
                      (u, v - sz), val)
            band_y = v - max(1, sz // 2)
            for xx in range(u - bw, u + bw + 1):
                if 0 <= xx < w and 0 <= band_y < h:
                    idx = band_y * w + xx
                    buf[idx] = 255

    return bytes(buf)


def render_to_pgm(data: bytes, width: int, height: int, path: str) -> None:
    """Write a raw frame as a PGM (handy for offline tests)."""
    with open(path, "wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        f.write(data)
