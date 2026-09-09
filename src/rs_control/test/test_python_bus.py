#!/usr/bin/env python3
"""No-hardware transaction tests for the Python RobStride bus."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import struct
import sys
import types
import unittest


PYTHON_ROOT = Path(__file__).parents[1] / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from rs_control_py.bus import Motor, RobstrideBus  # noqa: E402
from rs_control_py.protocol import (  # noqa: E402
    CommunicationType,
    compose_extended_id,
)


class _Message:
    def __init__(self, arbitration_id, is_extended_id, data):
        self.arbitration_id = arbitration_id
        self.is_extended_id = is_extended_id
        self.data = bytes(data)


class _FakeCanBus:
    instances = []

    def __init__(self, **_kwargs):
        self.messages = deque()
        self.sent = []
        self.instances.append(self)

    def send(self, message):
        self.sent.append(message)
        target_id = message.arbitration_id & 0xFF
        communication_type = (message.arbitration_id >> 24) & 0x1F
        if communication_type == CommunicationType.SET_DEVICE_ID:
            response_id = compose_extended_id(
                CommunicationType.GET_DEVICE_ID, 0, 0xFE
            )
            self.messages.append(_Message(response_id, True, b"set-id!"))
            return
        if communication_type == CommunicationType.GET_DEVICE_ID:
            response_id = compose_extended_id(
                CommunicationType.GET_DEVICE_ID, target_id, 0xFE
            )
            self.messages.append(_Message(response_id, True, b"verified"))
            return
        if communication_type != CommunicationType.READ_PARAMETER:
            return
        parameter = message.data[:2]
        response_id = compose_extended_id(
            CommunicationType.READ_PARAMETER, target_id, 0xFF
        )
        value = struct.pack("<f", target_id + 0.25)
        self.messages.append(
            _Message(response_id, True, parameter + b"\x00\x00" + value)
        )

    def recv(self, timeout=None):
        del timeout
        return self.messages.popleft() if self.messages else None

    def shutdown(self):
        pass


class PythonBusTest(unittest.TestCase):
    def test_reads_all_seven_motors_on_one_bus(self):
        fake_can = types.SimpleNamespace(
            Message=_Message,
            interface=types.SimpleNamespace(Bus=_FakeCanBus),
        )
        original_can = sys.modules.get("can")
        sys.modules["can"] = fake_can
        try:
            motors = {
                f"joint-{index}": Motor(index, "rs-00")
                for index in range(1, 8)
            }
            bus = RobstrideBus(
                "fake",
                motors=motors,
                response_timeout=0.01,
            )
            bus.connect()
            values = [
                bus.read_parameter_state(name).position_rad
                for name in motors
            ]
            self.assertEqual(values, [index + 0.25 for index in range(1, 8)])
            self.assertEqual(len(_FakeCanBus.instances[-1].sent), 21)
            bus.disconnect()
        finally:
            if original_can is None:
                sys.modules.pop("can", None)
            else:
                sys.modules["can"] = original_can

    def test_changes_and_verifies_one_motor_id(self):
        fake_can = types.SimpleNamespace(
            Message=_Message,
            interface=types.SimpleNamespace(Bus=_FakeCanBus),
        )
        original_can = sys.modules.get("can")
        sys.modules["can"] = fake_can
        try:
            bus = RobstrideBus("fake", response_timeout=0.01)
            bus.connect()
            changed = bus.set_device_id(1, 2)
            self.assertEqual(changed.id, 2)
            self.assertEqual(changed.uuid, b"set-id!\x00")
            self.assertEqual(_FakeCanBus.instances[-1].sent[0].arbitration_id, 0x0702FF01)
            verified = bus.ping_by_id(2)
            self.assertEqual(verified.id, 2)
            self.assertEqual(verified.uuid, b"verified")
            bus.disconnect()
        finally:
            if original_can is None:
                sys.modules.pop("can", None)
            else:
                sys.modules["can"] = original_can


if __name__ == "__main__":
    unittest.main()
