"""Live bridge state for flyscreen.

The ROS 2 bridge runs in the dev container, while flyscreen runs natively on
Windows (MuJoCo + numba are happiest there).  The two processes talk over a
small HTTP API exposed by :mod:`flyscreen.web.server`:

    POST /api/live/frame?w=160&h=120&fmt=gray
        body: w*h bytes, one byte per pixel (0..255)

    GET  /api/live/motor          -> JSON readout of the fly's motor signal
    GET  /api/live/brain.jpg      -> JPEG of the brain point cloud
    GET  /api/live/body.jpg       -> JPEG of the MuJoCo flybody
    POST /api/live/reset          -> reset the brain/canvas

This module only holds the state shared between the HTTP threads and the
render loop.  It deliberately knows nothing about ROS or HTTP.
"""
from __future__ import annotations

import threading
import time

import numpy as np


class LiveHub:
    """Thread-safe latest-frame / latest-readout mailbox."""

    def __init__(self) -> None:
        self._lock = threading.Lock()

        # newest camera frame (float32 0..1, HxW) and its sequence number
        self._frame: np.ndarray | None = None
        self._frame_id = 0
        self._frame_time = 0.0
        self._frames_received = 0

        # newest readout produced by the render loop
        self._motor: dict = {}
        self._status: dict = {}

        # newest JPEGs for Foxglove / the web page
        self._brain_jpeg = b""
        self._body_jpeg = b""
        self._images_time = 0.0

        # reset handshake: HTTP sets it, the render loop consumes it
        self._reset_serial = 0
        self._reset_seen = 0

    # ------------------------------------------------------------ frames
    def put_frame(self, gray: np.ndarray) -> int:
        """Store a grayscale frame.  Accepts uint8 0..255 or float 0..1."""
        g = np.asarray(gray)
        if g.ndim != 2:
            raise ValueError(f"live frame must be 2-D, got shape {g.shape}")
        if g.dtype == np.uint8:
            g = g.astype(np.float32) / 255.0
        else:
            g = g.astype(np.float32)
            peak = float(g.max()) if g.size else 0.0
            if peak > 1.0:
                g /= 255.0
            np.clip(g, 0.0, 1.0, out=g)
        with self._lock:
            self._frame = np.ascontiguousarray(g)
            self._frame_id += 1
            self._frame_time = time.time()
            self._frames_received += 1
            return self._frame_id

    def get_frame(self) -> tuple[np.ndarray | None, int]:
        with self._lock:
            if self._frame is None:
                return None, self._frame_id
            return self._frame.copy(), self._frame_id

    # ------------------------------------------------------------ outputs
    def set_outputs(
        self,
        motor: dict | None = None,
        status: dict | None = None,
        brain_jpeg: bytes | None = None,
        body_jpeg: bytes | None = None,
    ) -> None:
        with self._lock:
            if motor is not None:
                self._motor = dict(motor)
            if status is not None:
                self._status = dict(status)
            if brain_jpeg is not None:
                self._brain_jpeg = brain_jpeg
            if body_jpeg is not None:
                self._body_jpeg = body_jpeg
            if brain_jpeg is not None or body_jpeg is not None:
                self._images_time = time.time()

    def get_motor(self) -> tuple[dict, dict]:
        with self._lock:
            return dict(self._motor), dict(self._status)

    def get_images(self) -> tuple[bytes, bytes]:
        with self._lock:
            return self._brain_jpeg, self._body_jpeg

    # ------------------------------------------------------------ reset
    def request_reset(self) -> int:
        with self._lock:
            self._reset_serial += 1
            return self._reset_serial

    def take_reset(self) -> bool:
        with self._lock:
            if self._reset_serial == self._reset_seen:
                return False
            self._reset_seen = self._reset_serial
            return True

    # ------------------------------------------------------------ snapshot
    def snapshot(self) -> dict:
        with self._lock:
            now = time.time()
            return {
                "ok": True,
                "connected": self._frame_id > 0,
                "frame_id": self._frame_id,
                "frames_received": self._frames_received,
                "frame_age_s": (round(now - self._frame_time, 3)
                                if self._frame_time else None),
                "motor": dict(self._motor),
                "status": dict(self._status),
                "has_brain": bool(self._brain_jpeg),
                "has_body": bool(self._body_jpeg),
                "images_age_s": (round(now - self._images_time, 3)
                                 if self._images_time else None),
            }
