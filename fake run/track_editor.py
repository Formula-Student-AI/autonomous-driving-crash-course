#!/usr/bin/env python3
"""EUFS track editor: draw a centreline or click cones, then save a track.

Run on Windows with the portable Python (tkinter is included):

    C:\\Users\\V9\\flyscreen-portable\\flyscreen-portable\\runtime\\python\\python.exe ^
      "C:\\Users\\V9\\autonomous-driving-crash-course\\fake run\\track_editor.py"

Modes
-----
1. Draw centreline: left-click points; "Auto cones" offsets the line and places
   blue (left) and yellow (right) cones, plus orange start cones.
2. Place cones: choose Blue/Yellow/Orange and left-click; right-click deletes
   the nearest cone. "Centreline from cones" derives a centreline from the
   blue/yellow pairs.

Save writes:
    fake run/track_editor.csv       (sim cone CSV, car_start at 0,0,0)
    fake run/centerline_editor.csv  (ordered centreline)

Then, inside the container:
    bash "fake run/start_race.sh" \
      --track "fake run/track_editor.csv" \
      --centerline "fake run/centerline_editor.csv"

The tree scenery is generated automatically by track_scenery.py / track_camera.py
from the saved centreline; the editor draws a preview (green circles).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

try:
    from track_geometry import Track
except Exception:  # noqa: BLE001
    Track = None

DEFAULT_TRACK = os.path.normpath(os.path.join(
    ROOT, "sim", "src", "map_lib", "maps", "competitions", "FSUK", "2023",
    "trackdrive.csv"))
DEFAULT_CENTER = os.path.join(HERE, "fixed_path_centerline.csv")
OUT_TRACK = os.path.join(HERE, "track_editor.csv")
OUT_CENTER = os.path.join(HERE, "centerline_editor.csv")

COLORS = {"blue": "#1f6bff", "yellow": "#ffd400", "orange": "#ff6a00"}
BG = "#0e1219"
GRID = "#1b2432"
GRID2 = "#141b26"
AXIS = "#3b4a5c"
CENTRE = "#4ec3e0"
TREE = "#2f8f3c"
TRUNK = "#8a5a2b"
START = "#22c55e"


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def resample(points, spacing, closed):
    if len(points) < 2:
        return list(points)
    segs = [(points[i], points[i + 1]) for i in range(len(points) - 1)]
    if closed:
        segs.append((points[-1], points[0]))
    lengths = [dist(a, b) for a, b in segs]
    total = sum(lengths)
    if total <= 1e-9:
        return list(points)
    spacing = max(0.2, float(spacing))
    out = []
    target = 0.0
    while target <= total + 1e-9:
        acc = 0.0
        for (a, b), length in zip(segs, lengths):
            if length <= 1e-9:
                continue
            if target <= acc + length + 1e-9:
                t = min(1.0, max(0.0, (target - acc) / length))
                out.append((a[0] + t * (b[0] - a[0]),
                            a[1] + t * (b[1] - a[1])))
                break
            acc += length
        target += spacing
    if closed and len(out) > 1 and dist(out[0], out[-1]) < spacing * 0.5:
        out.pop()
    return out


def centre_from_cones(cones):
    blue = [(x, y) for tag, x, y in cones if tag == "blue"]
    yellow = [(x, y) for tag, x, y in cones if tag == "yellow"]
    if not blue or not yellow:
        return []
    used = set()
    mids = []
    for bx, by in blue:
        best, bd = -1, 1e18
        for j, (yx, yy) in enumerate(yellow):
            if j in used:
                continue
            d = (bx - yx) ** 2 + (by - yy) ** 2
            if d < bd:
                bd, best = d, j
        if best >= 0:
            used.add(best)
            yx, yy = yellow[best]
            mids.append(((bx + yx) / 2.0, (by + yy) / 2.0))
    if len(mids) < 2:
        return mids
    ordered = [mids[0]]
    remaining = set(range(1, len(mids)))
    while remaining:
        last = ordered[-1]
        j = min(remaining, key=lambda k: dist(mids[k], last))
        remaining.remove(j)
        ordered.append(mids[j])
    return ordered


def auto_cones_from_centerline(centerline, half_width, spacing, closed=True):
    """Blue left / yellow right cones at ``spacing`` along the centreline."""
    samples = resample(centerline, spacing, closed)
    n = len(samples)
    if n < 2:
        return []
    cones = []
    for i, (x, y) in enumerate(samples):
        q = samples[(i - 1) % n] if closed else samples[max(0, i - 1)]
        r = samples[(i + 1) % n] if closed else samples[min(n - 1, i + 1)]
        tx, ty = r[0] - q[0], r[1] - q[1]
        length = math.hypot(tx, ty)
        if length < 1e-9:
            tx, ty, length = 1.0, 0.0, 1.0
        nx, ny = -ty / length, tx / length
        cones.append(("blue", x + nx * half_width, y + ny * half_width))
        cones.append(("yellow", x - nx * half_width, y - ny * half_width))
    x, y = samples[0]
    q = samples[-1]
    r = samples[1]
    tx, ty = r[0] - q[0], r[1] - q[1]
    length = math.hypot(tx, ty) or 1.0
    nx, ny = -ty / length, tx / length
    cones.insert(0, ("orange", x + nx * half_width, y + ny * half_width))
    cones.insert(1, ("orange", x, y))
    cones.insert(2, ("orange", x - nx * half_width, y - ny * half_width))
    return cones


def read_centerline_csv(path):
    out = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                out.append((float(row["x"]), float(row["y"])))
            except (KeyError, TypeError, ValueError):
                continue
    return out


def write_track_csv(track_path, center_path, cones, centerline):
    """Write the sim cone CSV (identity car_start) + ordered centreline."""
    cones = list(cones)
    centre = list(centerline)
    if not centre and cones:
        centre = centre_from_cones(cones)
    if not cones:
        raise ValueError("no cones to save")
    if not centre:
        raise ValueError("no centreline to save")
    with open(track_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["tag", "x", "y", "direction",
                    "x_variance", "y_variance", "xy_covariance"])
        for i, (tag, x, y) in enumerate(cones):
            w.writerow([tag, f"{x:.4f}", f"{y:.4f}", i,
                        "0.01", "0.01", "0.0"])
        # Identity start transform: the editor works in the map frame.
        w.writerow(["car_start", "0.0", "0.0", "0.0",
                    "0.0", "0.0", "0.0"])
    with open(center_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["x", "y"])
        for x, y in centre:
            w.writerow([f"{x:.4f}", f"{y:.4f}"])


class Editor:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("EUFS Track Editor")
        self.root.geometry("1150x820")
        self.root.configure(bg=BG)

        self.centerline: list[tuple[float, float]] = []
        self.cones: list[tuple[str, float, float]] = []
        self.mode = tk.StringVar(value="centre")
        self.color = tk.StringVar(value="blue")
        self.closed = tk.BooleanVar(value=True)
        self.show_trees = tk.BooleanVar(value=True)
        self.half_width = tk.DoubleVar(value=2.0)
        self.spacing = tk.DoubleVar(value=3.0)
        self.status = tk.StringVar(value="Load the current track or draw a centreline.")
        self.scale = 8.0
        self.ox = 500.0
        self.oy = 400.0
        self.history: list[tuple[list, list]] = []
        self._pan = None

        self._build_ui()
        self._bind()
        self.load_current(silent=True)
        self.redraw()

    # ------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        top = tk.Frame(self.root, bg=BG)
        top.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)

        tk.Label(top, text="Mode", bg=BG, fg="#c8d6e5").pack(side=tk.LEFT)
        ttk.Radiobutton(top, text="1. Draw centreline", value="centre",
                        variable=self.mode).pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(top, text="2. Place cones", value="cones",
                        variable=self.mode).pack(side=tk.LEFT, padx=4)

        tk.Label(top, text="  Cone", bg=BG, fg="#c8d6e5").pack(side=tk.LEFT)
        for tag in ("blue", "yellow", "orange"):
            tk.Radiobutton(top, text=tag, value=tag, variable=self.color,
                           bg=BG, fg=COLORS[tag], selectcolor=BG,
                           activebackground=BG).pack(side=tk.LEFT)

        tk.Label(top, text="  half-width", bg=BG, fg="#c8d6e5").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.half_width, width=5).pack(side=tk.LEFT)
        tk.Label(top, text="spacing", bg=BG, fg="#c8d6e5").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.spacing, width=5).pack(side=tk.LEFT)
        tk.Checkbutton(top, text="closed", variable=self.closed, bg=BG,
                       fg="#c8d6e5", selectcolor=BG,
                       activebackground=BG).pack(side=tk.LEFT, padx=6)
        tk.Checkbutton(top, text="trees", variable=self.show_trees, bg=BG,
                       fg="#c8d6e5", selectcolor=BG,
                       activebackground=BG).pack(side=tk.LEFT)

        bar = tk.Frame(self.root, bg=BG)
        bar.pack(side=tk.TOP, fill=tk.X, padx=8)
        ttk.Button(bar, text="Auto cones", command=self.auto_cones).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar, text="Centreline from cones",
                   command=self.centre_from_cones).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar, text="Reverse centreline",
                   command=self.reverse_centre).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar, text="Load current", command=self.load_current).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar, text="Undo", command=self.undo).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar, text="Clear", command=self.clear).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar, text="Save (fake run/)", command=self.save_default).pack(side=tk.LEFT, padx=12)
        ttk.Button(bar, text="Save As...", command=self.save_as).pack(side=tk.LEFT, padx=3)

        self.canvas = tk.Canvas(self.root, bg=BG, highlightthickness=0)
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=6)

        bottom = tk.Frame(self.root, bg=BG)
        bottom.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(0, 6))
        tk.Label(bottom, textvariable=self.status, bg=BG, fg="#9fb3c8",
                 anchor="w").pack(side=tk.LEFT, fill=tk.X, expand=True)

    def _bind(self) -> None:
        self.canvas.bind("<Button-1>", self.on_left)
        self.canvas.bind("<Button-3>", self.on_right)
        self.canvas.bind("<ButtonPress-2>", self.on_pan_start)
        self.canvas.bind("<B2-Motion>", self.on_pan)
        self.canvas.bind("<ButtonRelease-2>", self.on_pan_end)
        self.canvas.bind("<Motion>", self.on_motion)
        self.canvas.bind("<MouseWheel>", self.on_wheel)       # Windows
        self.canvas.bind("<Button-4>", lambda e: self.zoom_at(e.x, e.y, 1.12))
        self.canvas.bind("<Button-5>", lambda e: self.zoom_at(e.x, e.y, 1 / 1.12))
        self.root.bind("<Control-z>", lambda e: self.undo())
        self.root.bind("<Control-s>", lambda e: self.save_default())

    # ------------------------------------------------------------ transforms
    def w2s(self, x, y):
        return self.ox + x * self.scale, self.oy - y * self.scale

    def s2w(self, px, py):
        return (px - self.ox) / self.scale, (self.oy - py) / self.scale

    def fit_view(self) -> None:
        pts = list(self.centerline) + [(x, y) for _t, x, y in self.cones]
        w = max(200, self.canvas.winfo_width())
        h = max(200, self.canvas.winfo_height())
        if not pts:
            self.scale = 6.0
            self.ox, self.oy = w / 2, h / 2
            return
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
        dx = max(1.0, xmax - xmin)
        dy = max(1.0, ymax - ymin)
        self.scale = max(0.5, min(60.0, min((w - 80) / dx, (h - 80) / dy)))
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        self.ox = w / 2 - cx * self.scale
        self.oy = h / 2 + cy * self.scale

    def zoom_at(self, px, py, factor) -> None:
        wx, wy = self.s2w(px, py)
        self.scale = max(0.5, min(120.0, self.scale * factor))
        self.ox = px - wx * self.scale
        self.oy = py + wy * self.scale
        self.redraw()

    def on_wheel(self, e) -> None:
        self.zoom_at(e.x, e.y, 1.12 if e.delta > 0 else 1 / 1.12)

    # ------------------------------------------------------------ drawing
    def redraw(self) -> None:
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 10:
            w, h = 1000, 650
        step = 1.0
        while step * self.scale < 28:
            step *= 5.0
        x0, y0 = self.s2w(0, h)
        x1, y1 = self.s2w(w, 0)
        ix = math.floor(x0 / step) * step
        while ix <= x1:
            sx, _ = self.w2s(ix, 0)
            c.create_line(sx, 0, sx, h, fill=GRID2 if abs(ix) > 1e-6 else AXIS)
            ix += step
        iy = math.floor(y0 / step) * step
        while iy <= y1:
            _, sy = self.w2s(0, iy)
            c.create_line(0, sy, w, sy, fill=GRID2 if abs(iy) > 1e-6 else AXIS)
            iy += step

        # car start (0,0)
        sx, sy = self.w2s(0, 0)
        c.create_oval(sx - 6, sy - 6, sx + 6, sy + 6, outline=START, width=2)
        c.create_text(sx + 10, sy - 10, text="car start (0,0)", fill=START,
                      anchor="w", font=("Consolas", 9))

        # trees preview
        if self.show_trees and len(self.centerline) >= 2:
            self._draw_trees(c)

        # centreline
        if len(self.centerline) >= 2:
            pts = list(self.centerline)
            if self.closed.get():
                pts.append(pts[0])
            coords = []
            for x, y in pts:
                px, py = self.w2s(x, y)
                coords.extend((px, py))
            c.create_line(*coords, fill=CENTRE, width=2)
            for x, y in self.centerline:
                px, py = self.w2s(x, y)
                c.create_oval(px - 2, py - 2, px + 2, py + 2,
                              fill=CENTRE, outline="")

        # cones
        for tag, x, y in self.cones:
            px, py = self.w2s(x, y)
            r = 5
            c.create_oval(px - r, py - r, px + r, py + r,
                          fill=COLORS.get(tag, "#aaaaaa"), outline="#05070a")
            c.create_oval(px - 2, py - 2, px + 2, py + 2,
                          fill="#f2f2f2", outline="")

        self._update_status()

    def _draw_trees(self, c) -> None:
        pts = self.centerline
        n = len(pts)
        if n < 3:
            return
        half = float(self.half_width.get() or 2.0)
        stride = max(2, n // 12)
        for k in range(0, n, stride):
            x, y = pts[k]
            qx, qy = pts[(k - 1) % n] if self.closed.get() else pts[max(0, k - 1)]
            rx, ry = pts[(k + 1) % n] if self.closed.get() else pts[min(n - 1, k + 1)]
            tx, ty = rx - qx, ry - qy
            L = math.hypot(tx, ty)
            if L < 1e-9:
                continue
            nx, ny = -ty / L, tx / L
            for side in (1, -1):
                off = half + 9.0 + 3.0 * (((k * 37 + (0 if side > 0 else 7)) % 11) / 11.0)
                txp, typ = x + nx * off * side, y + ny * off * side
                px, py = self.w2s(txp, typ)
                r = 7
                c.create_line(px, py, px, py - r, fill=TRUNK, width=2)
                c.create_oval(px - r, py - 2 * r, px + r, py,
                              fill=TREE, outline="")

    def _update_status(self) -> None:
        self.status.set(
            f"mode={self.mode.get()}  centre points={len(self.centerline)}  "
            f"cones={len(self.cones)}  scale={self.scale:.1f}px/m  "
            f"half-width={self.half_width.get():.2f}m  spacing={self.spacing.get():.2f}m  "
            f"| left-click add, right-click delete, middle-drag pan, wheel zoom, Ctrl-Z undo, Ctrl-S save")

    # ------------------------------------------------------------ history
    def push_history(self) -> None:
        self.history.append((list(self.centerline), list(self.cones)))
        if len(self.history) > 80:
            self.history.pop(0)

    def undo(self) -> None:
        if not self.history:
            self.status.set("nothing to undo")
            return
        self.centerline, self.cones = self.history.pop()
        self.redraw()

    # ------------------------------------------------------------ editing
    def on_left(self, e) -> None:
        x, y = self.s2w(e.x, e.y)
        self.push_history()
        if self.mode.get() == "centre":
            self.centerline.append((x, y))
        else:
            self.cones.append((self.color.get(), x, y))
        self.redraw()

    def on_right(self, e) -> None:
        x, y = self.s2w(e.x, e.y)
        tol = 10.0 / self.scale
        self.push_history()
        if self.mode.get() == "cones" and self.cones:
            idx = min(range(len(self.cones)),
                      key=lambda i: dist((self.cones[i][1], self.cones[i][2]), (x, y)))
            if dist((self.cones[idx][1], self.cones[idx][2]), (x, y)) <= tol:
                del self.cones[idx]
        elif self.centerline:
            idx = min(range(len(self.centerline)),
                      key=lambda i: dist(self.centerline[i], (x, y)))
            if dist(self.centerline[idx], (x, y)) <= tol:
                del self.centerline[idx]
        self.redraw()

    def on_pan_start(self, e) -> None:
        self._pan = (e.x, e.y, self.ox, self.oy)

    def on_pan(self, e) -> None:
        if self._pan is None:
            return
        x0, y0, ox, oy = self._pan
        self.ox = ox + (e.x - x0)
        self.oy = oy + (e.y - y0)
        self.redraw()

    def on_pan_end(self, e) -> None:
        self._pan = None

    def on_motion(self, e) -> None:
        x, y = self.s2w(e.x, e.y)
        self.status.set(
            f"x={x:8.2f}  y={y:8.2f}   |  mode={self.mode.get()}  "
            f"centre={len(self.centerline)}  cones={len(self.cones)}  "
            f"scale={self.scale:.1f}  | left add, right delete, middle pan, wheel zoom")

    # ------------------------------------------------------------ actions
    def reverse_centre(self) -> None:
        if not self.centerline:
            return
        self.push_history()
        self.centerline.reverse()
        self.redraw()
        self.status.set("centreline reversed — auto cones will swap sides")

    def auto_cones(self) -> None:
        if len(self.centerline) < 2:
            messagebox.showwarning("Auto cones", "Draw a centreline first.")
            return
        if self.cones and not messagebox.askyesno(
                "Auto cones", "Replace all existing blue/yellow/orange cones?"):
            return
        try:
            half = max(0.3, float(self.half_width.get()))
            spacing = max(0.5, float(self.spacing.get()))
        except Exception:  # noqa: BLE001
            messagebox.showerror("Auto cones", "half-width / spacing must be numbers")
            return
        self.push_history()
        cones = auto_cones_from_centerline(
            self.centerline, half, spacing, self.closed.get())
        if not cones:
            return
        self.cones = cones
        self.redraw()
        self.status.set(
            f"auto cones: {len(cones)} placed (blue left / yellow right, "
            f"{spacing:.1f} m spacing, half-width {half:.2f} m)")

    def centre_from_cones(self) -> None:
        if not any(t == "blue" for t, _x, _y in self.cones) or \
           not any(t == "yellow" for t, _x, _y in self.cones):
            messagebox.showwarning("Centreline", "Place blue and yellow cones first.")
            return
        self.push_history()
        self.centerline = centre_from_cones(self.cones)
        self.redraw()
        self.status.set(f"centreline from cones: {len(self.centerline)} points")

    def load_current(self, silent: bool = False) -> None:
        if Track is None:
            if not silent:
                messagebox.showerror("Load", "track_geometry.py not found next to the editor.")
            return
        try:
            track = Track(DEFAULT_TRACK, DEFAULT_CENTER if os.path.isfile(DEFAULT_CENTER) else None)
        except Exception as e:  # noqa: BLE001
            if not silent:
                messagebox.showerror("Load", f"{type(e).__name__}: {e}")
            return
        self.push_history()
        self.centerline = list(track.center)
        self.cones = list(track.cones)
        try:
            self.half_width.set(round(float(track.fallback_half_width), 2))
        except Exception:  # noqa: BLE001
            pass
        self.closed.set(True)
        self.fit_view()
        self.redraw()
        if not silent:
            self.status.set(
                f"loaded current track: {len(self.cones)} cones, "
                f"{len(self.centerline)} centre points")

    def clear(self) -> None:
        if not messagebox.askyesno("Clear", "Clear the centreline and all cones?"):
            return
        self.push_history()
        self.centerline = []
        self.cones = []
        self.redraw()

    # ------------------------------------------------------------ save
    def _write(self, track_path: str, center_path: str) -> None:
        write_track_csv(track_path, center_path, self.cones, self.centerline)
        rel_track = os.path.relpath(track_path, ROOT).replace("\\", "/")
        rel_center = os.path.relpath(center_path, ROOT).replace("\\", "/")
        messagebox.showinfo(
            "Saved",
            f"Track:       {track_path}\nCentreline:  {center_path}\n\n"
            f"Run inside the container:\n"
            f'bash "fake run/start_race.sh" --track "{rel_track}" '
            f'--centerline "{rel_center}"')
        self.status.set(f"saved {os.path.basename(track_path)} + "
                        f"{os.path.basename(center_path)}")

    def save_default(self) -> None:
        try:
            self._write(OUT_TRACK, OUT_CENTER)
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("Save", f"{type(e).__name__}: {e}")

    def save_as(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save track CSV", initialdir=HERE,
            initialfile="track_editor.csv", defaultextension=".csv",
            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        center = os.path.splitext(path)[0] + "_centerline.csv"
        try:
            self._write(path, center)
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("Save", f"{type(e).__name__}: {e}")


def main() -> None:
    root = tk.Tk()
    Editor(root)
    root.mainloop()


def cli(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--auto", action="store_true",
                    help="headless: auto-place cones along a centreline")
    ap.add_argument("--centerline", default=DEFAULT_CENTER)
    ap.add_argument("--half", type=float, default=2.0,
                    help="track half-width in metres")
    ap.add_argument("--spacing", type=float, default=3.0,
                    help="cone spacing in metres")
    ap.add_argument("--out", default=OUT_TRACK)
    ap.add_argument("--center-out", default=OUT_CENTER)
    ap.add_argument("--no-transform", action="store_true",
                    help="read the centreline as-is (already in map frame)")
    a = ap.parse_args(argv)
    if not a.auto:
        main()
        return
    if a.no_transform or Track is None:
        center = read_centerline_csv(a.centerline)
    else:
        # The current FSUK file carries a car_start transform; Track applies it.
        track = Track(DEFAULT_TRACK, a.centerline)
        center = list(track.center)
    if not center:
        raise SystemExit(f"no centreline in {a.centerline}")
    cones = auto_cones_from_centerline(
        center, max(0.3, a.half), max(0.5, a.spacing), True)
    write_track_csv(a.out, a.center_out, cones, center)
    rel_track = os.path.relpath(a.out, ROOT).replace("\\", "/")
    rel_center = os.path.relpath(a.center_out, ROOT).replace("\\", "/")
    print(f"wrote {a.out}: {len(cones)} cones, {len(center)} centre points")
    print(f"wrote {a.center_out}")
    print("trees are auto-generated by track_scenery.py / track_camera.py")
    print(f'run: bash "fake run/start_race.sh" --track "{rel_track}" '
          f'--centerline "{rel_center}"')


if __name__ == "__main__":
    cli()
