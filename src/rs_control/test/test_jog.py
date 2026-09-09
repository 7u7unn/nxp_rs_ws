#!/usr/bin/env python3
"""No-ROS tests for complete-pose jog target bookkeeping."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


PYTHON_ROOT = Path(__file__).parents[1] / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from rs_control_py.jog import (  # noqa: E402
    CONTROL_JOINTS,
    JointLimit,
    JogMotionProfile,
    JogTargetState,
)


class JogTargetStateTest(unittest.TestCase):
    def setUp(self):
        self.limits = {
            name: JointLimit(-1.0, 1.0) for name in CONTROL_JOINTS
        }
        self.targets = JogTargetState(self.limits)
        self.start = {
            name: index / 10.0 for index, name in enumerate(CONTROL_JOINTS)
        }
        self.targets.initialize(self.start)

    def test_single_joint_jog_holds_every_other_joint(self):
        self.targets.jog("joint-3", 0.25)
        result = self.targets.targets
        self.assertAlmostEqual(result["joint-3"], self.start["joint-3"] + 0.25)
        for name in CONTROL_JOINTS:
            if name != "joint-3":
                self.assertEqual(result[name], self.start[name])

    def test_initial_pose_can_be_anywhere_inside_limits(self):
        self.assertEqual(self.targets.targets, self.start)

    def test_jog_is_clamped(self):
        _, value, clamped = self.targets.jog("joint-1", 5.0)
        self.assertTrue(clamped)
        self.assertEqual(value, 1.0)

    def test_reinitialize_captures_a_new_arbitrary_pose(self):
        new_pose = {name: -0.5 for name in CONTROL_JOINTS}
        self.targets.initialize(new_pose)
        self.assertEqual(self.targets.targets, new_pose)

    def test_explicit_target_is_clamped_to_joint_limit(self):
        self.assertEqual(self.targets.set_target("joint-1", 4.0), 1.0)
        self.assertEqual(self.targets.target("joint-1"), 1.0)


class JogMotionProfileTest(unittest.TestCase):
    def setUp(self):
        self.profile = JogMotionProfile(
            JointLimit(-2.0, 2.0),
            max_velocity=0.5,
            max_acceleration=1.0,
            max_jerk=2.0,
            max_target_lead=0.25,
        )
        self.profile.initialize(0.0)

    def test_target_lead_must_allow_braking(self):
        with self.assertRaises(ValueError):
            JogMotionProfile(
                JointLimit(-2.0, 2.0),
                max_velocity=0.5,
                max_acceleration=1.0,
                max_jerk=2.0,
                max_target_lead=0.2,
            )

    def test_target_lead_and_velocity_are_bounded(self):
        target, clamped = self.profile.set_target(1.0)
        self.assertTrue(clamped)
        self.assertEqual(target, 0.25)
        for _ in range(200):
            self.profile.advance(0.02)
            self.assertLessEqual(abs(self.profile.velocity), 0.5 + 1e-9)
            self.assertLessEqual(abs(self.profile.acceleration), 1.0 + 1e-9)
        self.assertAlmostEqual(self.profile.position, 0.25, places=7)
        self.assertAlmostEqual(self.profile.velocity, 0.0, places=7)
        self.assertFalse(self.profile.active)

    def test_preview_does_not_change_profile(self):
        self.profile.set_target(0.2)
        before = (self.profile.position, self.profile.velocity, self.profile.target)
        position, velocity = self.profile.preview(0.2)
        self.assertGreater(position, before[0])
        self.assertGreaterEqual(velocity, 0.0)
        self.assertEqual(
            (self.profile.position, self.profile.velocity, self.profile.target), before
        )

    def test_reversal_is_continuous_and_stays_inside_limits(self):
        self.profile.set_target(0.25)
        for _ in range(20):
            self.profile.advance(0.02)
        old_velocity = self.profile.velocity
        reverse_target, _ = self.profile.set_target(-0.25)
        self.profile.advance(0.02)
        self.assertLessEqual(abs(self.profile.velocity - old_velocity), 0.04 + 1e-9)
        for _ in range(200):
            self.profile.advance(0.02)
        self.assertAlmostEqual(self.profile.position, reverse_target, places=7)
        self.assertGreaterEqual(self.profile.position, -2.0)
        self.assertLessEqual(self.profile.position, 2.0)

    def test_controlled_stop_reaches_braking_point(self):
        self.profile.set_target(0.25)
        for _ in range(30):
            self.profile.advance(0.02)
        moving_position = self.profile.position
        moving_velocity = self.profile.velocity
        stop_target, _ = self.profile.request_stop()
        self.assertGreater(stop_target, moving_position)
        for _ in range(200):
            self.profile.advance(0.02)
        self.assertAlmostEqual(self.profile.position, stop_target, places=7)
        self.assertAlmostEqual(self.profile.velocity, 0.0, places=7)
        self.assertGreater(moving_velocity, 0.0)

    def test_controlled_stop_includes_jerk_transition_distance(self):
        self.profile.initialize(0.0, velocity=0.5)
        stop_target, _ = self.profile.request_stop()
        # v²/(2a) is 0.125 m; changing acceleration under the jerk limit needs
        # additional distance and must still fit inside the target-lead bound.
        self.assertGreater(stop_target, 0.125)
        for _ in range(200):
            self.profile.advance(0.02)
        self.assertAlmostEqual(self.profile.position, stop_target, places=7)
        self.assertAlmostEqual(self.profile.velocity, 0.0, places=7)

    def test_controlled_stop_brakes_a_profile_with_acceleration(self):
        self.profile.initialize(0.0, acceleration=0.5)
        self.assertGreater(self.profile.acceleration, 0.0)
        self.profile.request_stop()
        for _ in range(100):
            self.profile.advance(0.02)
        self.assertAlmostEqual(self.profile.position, 0.0, places=7)
        self.assertAlmostEqual(self.profile.velocity, 0.0, places=7)
        self.assertFalse(self.profile.active)


if __name__ == "__main__":
    unittest.main()
