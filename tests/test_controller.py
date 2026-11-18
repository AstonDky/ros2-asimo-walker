"""Offline behavior checks; these do not run ROS or the simulator."""

import ast
import importlib
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "src" / "robot_simulation_experiment" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from asimo_walker.common import D, Pose2D, RobotFeedback, WalkerParams
from asimo_walker.contact_state_machine import ContactAndStateMachine, WalkState
from asimo_walker.footstep_planner import FootstepPlanner
from asimo_walker.leg_ik import LegIK
from asimo_walker.swing_foot import SwingFootPlanner
from asimo_walker.teleop_command import TeleopCommandBuffer
from asimo_walker.teleop_profiles import get_profile, resolve_profile_from_input, walker_params_for_profile
from asimo_walker.zmp_preview import ZMPPreviewPlanner


class ControllerChecks(unittest.TestCase):
    def test_python_sources_parse(self):
        for path in SCRIPTS.rglob("*.py"):
            with self.subTest(path=path.relative_to(ROOT)):
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path), feature_version=(3, 10))

    def test_public_entry_import_does_not_require_private_autotune(self):
        # Stub only imports, not ROS execution. This reproduces the missing-module
        # failure that previously prevented even public walk/GUI mode startup.
        modules = {name: types.ModuleType(name) for name in (
            "rclpy", "rclpy.node", "rclpy.executors", "std_msgs", "std_msgs.msg"
        )}
        modules["rclpy.node"].Node = object
        modules["rclpy.executors"].ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
        modules["std_msgs.msg"].Float64MultiArray = object
        with patch.dict(sys.modules, modules):
            sys.modules.pop("asimo_walker.main", None)
            module = importlib.import_module("asimo_walker.main")
            self.assertTrue(callable(module.main))
            self.assertNotIn("asimo_walker.autotune", sys.modules)
        sys.modules.pop("asimo_walker.main", None)

    def test_keyboard_latches_switches_and_releases(self):
        buffer = TeleopCommandBuffer()
        buffer.press_key("w")
        self.assertEqual(resolve_profile_from_input(buffer.snapshot())[0].name, "forward_walk")
        buffer.press_key("s")
        self.assertFalse(buffer.snapshot().key_w)
        self.assertEqual(resolve_profile_from_input(buffer.snapshot())[0].name, "backward_walk")
        buffer.press_key("s")
        self.assertEqual(resolve_profile_from_input(buffer.snapshot())[0].name, "idle")

    def test_forward_and_backward_footsteps_reverse_direction(self):
        forward = FootstepPlanner(walker_params_for_profile(get_profile("forward_walk")))
        backward = FootstepPlanner(walker_params_for_profile(get_profile("backward_walk")))
        self.assertLess(forward.steps[0].left_target.x, 0.0)
        self.assertGreater(backward.steps[0].left_target.x, 0.0)
        self.assertEqual([s.swing for s in forward.steps], ["left", "right"] * 3)
        old_targets = [(s.left_target.copy(), s.right_target.copy()) for s in forward.steps]
        forward.ensure_steps(10)
        self.assertEqual(old_targets, [(s.left_target, s.right_target) for s in forward.steps[:6]])

    def test_turn_profiles_have_opposite_yaw(self):
        left = FootstepPlanner(walker_params_for_profile(get_profile("turn_left")))
        right = FootstepPlanner(walker_params_for_profile(get_profile("turn_right")))
        self.assertEqual(len(left.steps), 10)
        for lstep, rstep in zip(left.steps, right.steps):
            self.assertAlmostEqual(lstep.swing_pose().yaw, -rstep.swing_pose().yaw)

    def test_swing_starts_and_lands_at_requested_pose(self):
        params = WalkerParams()
        planner = SwingFootPlanner(params)
        start = Pose2D(0.0, 0.045, 0.0)
        target = Pose2D(-0.045, 0.045, 0.0, yaw=0.1)
        self.assertEqual(planner.pose(start, target, 0.0), start)
        self.assertEqual(planner.pose(start, target, 1.0), target)
        self.assertAlmostEqual(planner.pose(start, target, 0.5).z, params.foot_clearance)

    def test_com_remains_finite_and_rate_bounded_under_large_references(self):
        params = WalkerParams()
        planner = ZMPPreviewPlanner(params)
        planner.reset(params)
        for i in range(1000):
            target = (10.0, -10.0) if i < 500 else (-10.0, 10.0)
            values = planner.update(params.dt, target, target)
            self.assertTrue(all(math.isfinite(x) for x in values))
            self.assertLessEqual(max(abs(x) for x in values[2:4]), params.max_com_speed)
            self.assertLessEqual(max(abs(x) for x in values[4:]), params.max_com_accel)

    def test_ik_returns_finite_joint_targets_within_model_limits(self):
        params = WalkerParams()
        ik = LegIK(params)
        for forward in (0.0, -0.045, 0.045, 2.0):
            targets = ik.solve(Pose2D(z=params.pelvis_height), Pose2D(forward, 0.045), Pose2D(0.0, -0.045))
            for leg in targets:
                self.assertEqual(len(leg), 6)
                for angle, (lower, upper) in zip(leg, ik.limits):
                    self.assertTrue(math.isfinite(angle))
                    self.assertGreaterEqual(angle, lower)
                    self.assertLessEqual(angle, upper)

    def test_finite_walk_completes_with_phase_fallback(self):
        params = WalkerParams(total_steps=2)
        machine = ContactAndStateMachine(params)
        feedback = RobotFeedback(gyro=[0.0, 0.0, 0.0])
        machine.start()
        visited = set()
        for _ in range(2000):
            visited.add(machine.update(params.dt, feedback, 0.0, 0.0))
            if machine.state == WalkState.DONE:
                break
        self.assertEqual(machine.state, WalkState.DONE)
        self.assertEqual(machine.step_index, 2)
        self.assertIn(WalkState.LEFT_SWING, visited)
        self.assertIn(WalkState.RIGHT_SWING, visited)

    def test_force_feedback_gates_support_transfer(self):
        params = WalkerParams()
        machine = ContactAndStateMachine(params)
        feedback = RobotFeedback(gyro=[0.0, 0.0, 0.0], left_force=90.0, right_force=10.0)
        machine.start()
        machine.update(params.crouch_time, feedback, 0.0, 0.0)
        self.assertEqual(machine.state, WalkState.TRANSFER_TO_RIGHT)
        machine.update(params.transfer_time, feedback, 0.0, 0.0)
        self.assertEqual(machine.state, WalkState.TRANSFER_TO_RIGHT)
        feedback.left_force, feedback.right_force = 20.0, 80.0
        machine.update(params.dt, feedback, 0.0, 0.0)
        self.assertEqual(machine.state, WalkState.LEFT_SWING)

    def test_tilt_aborts_and_holds_state(self):
        machine = ContactAndStateMachine(WalkerParams())
        feedback = RobotFeedback(gyro=[0.0, 0.0, 0.0])
        machine.start()
        self.assertEqual(machine.update(0.02, feedback, 19.0 * D, 0.0), WalkState.ABORT)
        self.assertEqual(machine.update(0.02, feedback, 0.0, 0.0), WalkState.ABORT)


if __name__ == "__main__":
    unittest.main()
