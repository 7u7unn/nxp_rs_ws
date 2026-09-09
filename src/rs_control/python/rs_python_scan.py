#!/usr/bin/env python3
"""Scan a SocketCAN channel for RobStride actuator IDs."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

# See rs_python_control.py: keep the installed sibling Python package
# importable when ros2-run invokes this renamed script.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from rs_control_py.bus import RobstrideBus, RobstrideError


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--host-id", type=int, default=0xFF)
    parser.add_argument("--timeout-ms", type=float, default=100.0)
    parser.add_argument("--start-id", type=int, default=1)
    parser.add_argument("--end-id", type=int, default=255)
    args = parser.parse_args()

    bus = RobstrideBus(
        channel=args.interface,
        host_id=args.host_id,
        bitrate=args.bitrate,
        response_timeout=args.timeout_ms / 1000.0,
    )
    try:
        bus.connect()
        motors = bus.scan_channel(args.start_id, args.end_id)
        print(f"Found {len(motors)} RobStride motor(s) on {args.interface}:")
        for motor in motors:
            print(f"  id={motor.id} uuid=0x{motor.uuid.hex()}")
        return 0
    except (RobstrideError, ValueError) as exc:
        print(f"CAN scan failed: {exc}")
        return 1
    finally:
        bus.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
