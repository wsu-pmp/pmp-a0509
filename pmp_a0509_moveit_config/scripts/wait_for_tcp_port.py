#!/usr/bin/env python3
"""poll a tcp port until it accepts connections, or time out.

DRHWInterface's on_init makes an initial-state call in on_init, if the
endpoint is not accepting connections yet it throws, killing the whole
ros2_control_node process incl. gripper

exit 0 once the port accepts a connection
exit 1 on timeout
"""

import socket
import sys
import time


def main() -> int:
    host, port_str, timeout_str = sys.argv[1], sys.argv[2], sys.argv[3]
    port = int(port_str)
    timeout = float(timeout_str)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                # grace period past the raw accept so the server side finishes
                # its own init before DRHWInterface's first rpc
                time.sleep(1.0)
                print(f"drcf port {host}:{port} is up")
                return 0
        except OSError:
            time.sleep(0.5)
    print(
        f"timed out waiting for drcf port {host}:{port} after {timeout}s",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
