"""Tiny HTTP client for the flyscreen live API.

Only the standard library is used so the ROS 2 container needs no extra Python
packages.  Every method returns ``None`` (or ``False``) on failure; the node
treats that as "fly offline" and keeps the assist controller running.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request


class FlyClient:
    def __init__(self, base_url: str, timeout: float = 0.5) -> None:
        self.base = base_url.rstrip("/")
        self.timeout = float(timeout)
        self.last_error = ""

    # ------------------------------------------------------------ helpers
    def _get(self, path: str) -> bytes | None:
        try:
            with urllib.request.urlopen(self.base + path, timeout=self.timeout) as r:
                return r.read()
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            self.last_error = f"{type(e).__name__}: {e}"
            return None

    def _json(self, path: str) -> dict | None:
        raw = self._get(path)
        if raw is None:
            return None
        try:
            return json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as e:
            self.last_error = f"bad JSON: {e}"
            return None

    # ------------------------------------------------------------ API
    def post_frame(self, gray: bytes, width: int, height: int) -> dict | None:
        url = f"{self.base}/api/live/frame?w={int(width)}&h={int(height)}&fmt=gray"
        req = urllib.request.Request(
            url, data=gray, method="POST",
            headers={"Content-Type": "application/octet-stream"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as e:
            self.last_error = f"{type(e).__name__}: {e}"
            return None

    def get_motor(self) -> dict | None:
        """Full live snapshot: frame_id, motor, status, connectivity."""
        return self._json("/api/live/motor")

    def get_brain_jpeg(self) -> bytes | None:
        return self._get("/api/live/brain.jpg")

    def get_body_jpeg(self) -> bytes | None:
        return self._get("/api/live/body.jpg")

    def post_reset(self) -> bool:
        req = urllib.request.Request(f"{self.base}/api/live/reset", data=b"",
                                     method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                r.read()
            return True
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            self.last_error = f"{type(e).__name__}: {e}"
            return False
