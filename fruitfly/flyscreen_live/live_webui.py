"""flyscreen 实时模式入口（供 ROS 2 bridge 推帧/读运动信号）。

用法：

    python live_webui.py
    python live_webui.py --port 8760 --size big

等价于 `python webui.py --live`。浏览器仍然打开 http://127.0.0.1:8760/ 看
脑点云和 MuJoCo 身体，只是片源不再是视频文件，而是 ROS 2 bridge 通过
HTTP POST 上来的灰度帧。
"""
from __future__ import annotations

import sys

from webui import main

# Same three sizes as fly.py: (canvas w, canvas h, body w, body h)
SIZES = {
    "small": (600, 490, 460, 490),
    "mid": (800, 660, 580, 660),
    "big": (1000, 820, 700, 820),
}


def _translate(argv: list[str]) -> list[str]:
    """Accept fly.py-style --size in addition to webui.py's explicit sizes."""
    out: list[str] = []
    i = 0
    while i < len(argv):
        if argv[i] == "--size":
            if i + 1 >= len(argv):
                raise SystemExit("--size needs a value")
            name = argv[i + 1]
            if name not in SIZES:
                raise SystemExit(f"unknown size {name!r}; pick {sorted(SIZES)}")
            cw, ch, bw, bh = SIZES[name]
            out += ["--canvas-size", f"{cw}x{ch}", "--body-size", f"{bw}x{bh}"]
            i += 2
            continue
        out.append(argv[i])
        i += 1
    return out


def run() -> int:
    argv = ["--live", *_translate(sys.argv[1:])]
    return main(argv)


if __name__ == "__main__":
    sys.exit(run())
