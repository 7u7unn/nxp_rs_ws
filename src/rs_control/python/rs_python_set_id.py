#!/usr/bin/env python3
"""Inspect or change one RobStride motor's private-protocol CAN ID."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

# CMake installs this wrapper beside the Python package without the .py suffix.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from rs_control_py.bus import DiscoveredMotor, RobstrideBus, RobstrideError


def _can_id(value: str) -> int:
    """Parse a decimal or 0x-prefixed CAN ID."""

    try:
        result = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid CAN ID: {value}") from exc
    if not 1 <= result <= 0xFF:
        raise argparse.ArgumentTypeError("CAN ID must be in the range 1..255")
    return result


def _host_id(value: str) -> int:
    try:
        result = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid host ID: {value}") from exc
    if not 0 <= result <= 0xFF:
        raise argparse.ArgumentTypeError("host ID must be in the range 0..255")
    return result


def _try_ping(bus: RobstrideBus, device_id: int, timeout: float) -> DiscoveredMotor | None:
    try:
        return bus.ping_by_id(device_id, timeout=timeout)
    except RobstrideError as exc:
        if "timeout waiting" in str(exc).lower():
            return None
        raise


def _print_motor(label: str, motor: DiscoveredMotor) -> None:
    print(f"{label}: id={motor.id} uuid=0x{motor.uuid.hex()}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect or change one RobStride private-protocol CAN ID. "
            "Without --old-id, the tool scans for responding motors. "
            "No write occurs unless --new-id and --yes are both supplied."
        )
    )
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--host-id", type=_host_id, default=0xFF)
    parser.add_argument("--timeout-ms", type=float, default=100.0)
    parser.add_argument(
        "--old-id",
        type=_can_id,
        help="current motor ID; omit to scan the bus first",
    )
    parser.add_argument("--new-id", type=_can_id)
    parser.add_argument("--start-id", type=_can_id, default=1)
    parser.add_argument("--end-id", type=_can_id, default=255)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="actually send the type-7 ID-change frame",
    )
    args = parser.parse_args()

    if args.bitrate <= 0:
        parser.error("--bitrate must be positive")
    if args.timeout_ms <= 0:
        parser.error("--timeout-ms must be positive")
    if args.start_id > args.end_id:
        parser.error("--start-id must not be greater than --end-id")
    if args.old_id == args.host_id:
        parser.error("--old-id must differ from --host-id")
    if args.new_id == args.host_id:
        parser.error("--new-id must differ from --host-id")
    if args.new_id is None and args.yes:
        parser.error("--yes requires --new-id")

    timeout = args.timeout_ms / 1000.0
    bus = RobstrideBus(
        channel=args.interface,
        host_id=args.host_id,
        bitrate=args.bitrate,
        response_timeout=timeout,
    )
    try:
        bus.connect()
        if args.old_id is None:
            discovered = bus.scan_channel(args.start_id, args.end_id)
            print(f"Found {len(discovered)} responding motor(s):")
            for motor in discovered:
                _print_motor("  motor", motor)
            if args.new_id is None:
                print("Read-only scan complete; no ID change was sent.")
                return 0
            if len(discovered) != 1:
                print(
                    "ID change requires exactly one responding motor. "
                    "Disconnect the others or provide --old-id explicitly."
                )
                return 1
            current = discovered[0]
            old_id = current.id
        else:
            old_id = args.old_id
            current = _try_ping(bus, old_id, timeout)
            if current is None:
                raise RobstrideError(
                    f"no motor replied at old ID {old_id}; no ID change was sent"
                )
            _print_motor("Current motor", current)

        if args.new_id is None or args.new_id == old_id:
            print("Read-only check complete; no ID change was sent.")
            return 0

        occupied = _try_ping(bus, args.new_id, timeout)
        if occupied is not None:
            _print_motor("Refusing: new ID is already occupied", occupied)
            return 1

        print(
            f"Planned change: ID {old_id} -> {args.new_id} "
            f"(host ID {args.host_id})"
        )
        if not args.yes:
            print("Dry run: add --yes to send the type-7 ID-change frame.")
            return 0

        acknowledgement_error: RobstrideError | None = None
        try:
            changed = bus.set_device_id(old_id, args.new_id, timeout=timeout)
        except RobstrideError as exc:
            if "timeout waiting" not in str(exc).lower():
                raise
            # Some firmware applies the new ID but omits or changes the
            # acknowledgement frame. The verification ping is authoritative.
            acknowledgement_error = exc

        verified = _try_ping(bus, args.new_id, timeout)
        if verified is None:
            if acknowledgement_error is not None:
                raise acknowledgement_error
            raise RobstrideError(
                f"ID change was acknowledged, but ID {args.new_id} "
                "did not answer verification"
            )
        if acknowledgement_error is not None:
            print(
                "Warning: ID-change acknowledgement was not received, "
                "but the new ID answered verification."
            )
        else:
            _print_motor("ID-change response", changed)
        _print_motor("Verified motor", verified)
        print("ID change completed successfully.")
        return 0
    except (RobstrideError, ValueError) as exc:
        print(f"RobStride ID operation failed: {exc}")
        return 1
    finally:
        try:
            bus.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
