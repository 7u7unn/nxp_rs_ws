#!/usr/bin/env python3
"""No-hardware tests for the Python RobStride wire format."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


PYTHON_ROOT = Path(__file__).parents[1] / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from rs_control_py.protocol import (  # noqa: E402
    CommunicationType,
    ParameterType,
    compose_extended_id,
    decode_fault_report,
    describe_fault_report,
    describe_status_flags,
    make_disable_frame,
    make_enable_frame,
    make_operation_frame,
    make_parameter_read_frame,
    make_parameter_write_frame,
    make_set_device_id_frame,
)


class PythonProtocolTest(unittest.TestCase):
    def test_extended_id_and_parameter_frames(self):
        self.assertEqual(
            compose_extended_id(CommunicationType.READ_PARAMETER, 0xFF, 7),
            0x1100FF07,
        )
        read = make_parameter_read_frame(0xFF, 7, ParameterType.MECHANICAL_POSITION)
        self.assertEqual(read.data, bytes.fromhex("19 70 00 00 00 00 00 00"))
        write = make_parameter_write_frame(0xFF, 7, ParameterType.MODE, 3)
        self.assertEqual(write.data, bytes.fromhex("05 70 00 00 03 00 00 00"))

    def test_operation_frame_uses_reference_scaling(self):
        frame = make_operation_frame(
            7,
            "rs-00",
            position=4.0 * 3.141592653589793,
            kp=500.0,
            kd=0.0,
            velocity=-50.0,
            torque=0.0,
        )
        self.assertEqual(frame.arbitration_id, 0x017FFF07)
        self.assertEqual(frame.data, bytes.fromhex("ff fe 00 00 ff ff 00 00"))

    def test_enable_and_disable_frames(self):
        self.assertEqual(make_enable_frame(0xFF, 7).arbitration_id, 0x0300FF07)
        self.assertEqual(make_disable_frame(0xFF, 7).arbitration_id, 0x0400FF07)

    def test_set_device_id_frame(self):
        frame = make_set_device_id_frame(0xFF, old_device_id=1, new_device_id=2)
        self.assertEqual(frame.arbitration_id, 0x0702FF01)
        self.assertEqual(frame.data, b"\x00" * 8)

    def test_describes_status_and_fault_bits(self):
        status = describe_status_flags(0x2500)
        self.assertIn("encoder uncalibrated", status)
        self.assertIn("overtemperature", status)
        self.assertIn("undervoltage", status)

        fault_code, warning_code = decode_fault_report(
            bytes.fromhex("80 00 01 00 01 00 00 00")
        )
        self.assertEqual(fault_code, (1 << 7) | (1 << 16))
        self.assertEqual(warning_code, 1)
        fault = describe_fault_report(fault_code, warning_code)
        self.assertIn("encoder uncalibrated", fault)
        self.assertIn("A-phase overcurrent", fault)
        self.assertIn("motor overtemperature warning", fault)


if __name__ == "__main__":
    unittest.main()
