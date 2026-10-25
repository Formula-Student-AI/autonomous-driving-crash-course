"""TCP relay for WSL -> Windows service access.

Under WSL mirrored networking the container can reach the WSL VM (usually
``172.17.0.1`` from Docker), but not a service bound to Windows' ``127.0.0.1``.
Run this relay in WSL; it listens on ``0.0.0.0:LISTEN_PORT`` and forwards every
connection to ``TARGET_HOST:TARGET_PORT`` (by default the Windows-local
flyscreen live server).

    wsl -e python3 fruitfly/tools/wsl_relay.py 8780 127.0.0.1 8760
"""
from __future__ import annotations

import argparse
import socket
import threading


def pipe(src: socket.socket, dst: socket.socket) -> None:
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except OSError:
        pass
    finally:
        try:
            dst.shutdown(socket.SHUT_WR)
        except OSError:
            pass


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("listen_port", type=int, nargs="?", default=8780)
    ap.add_argument("target_host", nargs="?", default="127.0.0.1")
    ap.add_argument("target_port", type=int, nargs="?", default=8760)
    a = ap.parse_args()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", a.listen_port))
    srv.listen(32)
    print(f"relay 0.0.0.0:{a.listen_port} -> {a.target_host}:{a.target_port}",
          flush=True)
    while True:
        client, _ = srv.accept()
        try:
            target = socket.create_connection((a.target_host, a.target_port),
                                              timeout=5)
        except OSError as e:
            print(f"relay: target unavailable: {e}", flush=True)
            client.close()
            continue
        client.settimeout(None)
        target.settimeout(None)
        threading.Thread(target=pipe, args=(client, target), daemon=True).start()
        threading.Thread(target=pipe, args=(target, client), daemon=True).start()


if __name__ == "__main__":
    main()
