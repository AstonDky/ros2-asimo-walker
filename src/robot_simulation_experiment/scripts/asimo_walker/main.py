#!/usr/bin/python3
import math
import os
import sys
import threading

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray

from asimo_walker.autotune import AutoTuneManager
from asimo_walker.common import D, LEG_DOF, Pose2D, RobotFeedback, WalkerParams, clamp, lerp, smoothstep
from asimo_walker.contact_state_machine import ContactAndStateMachine, WalkState
from asimo_walker.footstep_planner import FootstepPlanner
from asimo_walker.leg_ik import LegIK
from asimo_walker.stabilizer import Stabilizer
from asimo_walker.swing_foot import SwingFootPlanner
from asimo_walker.teleop_command import TeleopCommandBuffer, TeleopInputState
from asimo_walker.teleop_profiles import (
    get_profile,
    requested_profile_name,
    resolve_profile_from_input,
    walker_params_for_profile,
)
from asimo_walker.zmp_preview import ZMPPreviewPlanner
from asimo_walker.zmp_reference import ZMPReferencePlanner


class AsimoStyleZMPWalker(Node):
    def __init__(self, teleop_buffer=None):
        super().__init__("asimo_style_zmp_walker")
        self.declare_parameter("mode", "teleop_gui")
        self.mode = str(self.get_parameter("mode").value)

        # WalkerParams is created before motion state so every mode starts from the same baseline.
        self.params = WalkerParams()
        self.dt = self.params.dt
        self.feedback = RobotFeedback()
        self.ori0 = None
        self.teleop_buffer = teleop_buffer
        self.active_profile = get_profile("idle")
        self.pending_profile = get_profile("idle")
        self.teleop_status_message = "Idle / safe hold."
        self.teleop_requested_profile_name = "idle"
        self.teleop_active_profile_name = "idle"
        self.teleop_state_name = "WAIT"
        self.teleop_pitch_deg = None
        self.teleop_roll_deg = None
        self.commanded_waist_yaw = 0.0

        self.leg_pub = self.create_publisher(Float64MultiArray, "/legTargetJoints", 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, "/armTargetJoints", 10)

        self.create_subscription(Float64MultiArray, "/robot/ori", self.cb_ori, 10)
        self.create_subscription(Float64MultiArray, "/robot/angVel", self.cb_gyro, 10)
        self.create_subscription(Float64MultiArray, "/robot/pos", self.cb_pos, 10)
        self.create_subscription(Float64MultiArray, "/leftLegJoints", self.cb_left_leg, 10)
        self.create_subscription(Float64MultiArray, "/rightLegJoints", self.cb_right_leg, 10)
        self.create_subscription(Float64MultiArray, "/leftArmJoints", self.cb_left_arm, 10)
        self.create_subscription(Float64MultiArray, "/rightArmJoints", self.cb_right_arm, 10)
        self.create_subscription(Float64MultiArray, "/leftFootForce", self.cb_left_force, 10)
        self.create_subscription(Float64MultiArray, "/rightFootForce", self.cb_right_force, 10)
        self.create_subscription(Float64MultiArray, "/leftFootCOP", self.cb_left_cop, 10)
        self.create_subscription(Float64MultiArray, "/rightFootCOP", self.cb_right_cop, 10)

        self.left_arm_fb = None
        self.right_arm_fb = None
        self.left_arm_base = None
        self.right_arm_base = None
        self.left_arm_cmd = []
        self.right_arm_cmd = []

        self.footsteps = FootstepPlanner(self.params)
        self.zmp_ref = ZMPReferencePlanner(self.params)
        self.com = ZMPPreviewPlanner(self.params)
        self.swing = SwingFootPlanner(self.params)
        self.ik = LegIK(self.params)
        self.stabilizer = Stabilizer(self.params)
        self.state_machine = ContactAndStateMachine(self.params)

        self.left_pose = self.footsteps.initial_left()
        self.right_pose = self.footsteps.initial_right()
        self.swing_start = {"left": self.left_pose.copy(), "right": self.right_pose.copy()}
        self.initial_left_q = None
        self.initial_right_q = None
        self.prev_left_cmd = None
        self.prev_right_cmd = None
        self.stand_start_left_cmd = None
        self.stand_start_right_cmd = None
        self.last_state = self.state_machine.state
        self.last_debug_t = self.get_clock().now()
        self.pending_step_adjustment = None

        self.autotune = AutoTuneManager(self) if self.mode == "auto_tune" else None

        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info(f"mode={self.mode}; publishing /legTargetJoints and /armTargetJoints")

    def cb_ori(self, msg):
        self.feedback.ori = list(msg.data)

    def cb_gyro(self, msg):
        self.feedback.gyro = list(msg.data)

    def cb_pos(self, msg):
        self.feedback.pos = list(msg.data)

    def cb_left_leg(self, msg):
        if len(msg.data) >= LEG_DOF:
            self.feedback.left_leg = list(msg.data[:LEG_DOF])

    def cb_right_leg(self, msg):
        if len(msg.data) >= LEG_DOF:
            self.feedback.right_leg = list(msg.data[:LEG_DOF])

    def cb_left_arm(self, msg):
        self.left_arm_fb = list(msg.data)

    def cb_right_arm(self, msg):
        self.right_arm_fb = list(msg.data)

    def cb_left_force(self, msg):
        self.feedback.left_force = self._force_value(msg.data)

    def cb_right_force(self, msg):
        self.feedback.right_force = self._force_value(msg.data)

    def cb_left_cop(self, msg):
        if len(msg.data) >= 2:
            self.feedback.left_cop = (float(msg.data[0]), float(msg.data[1]))

    def cb_right_cop(self, msg):
        if len(msg.data) >= 2:
            self.feedback.right_cop = (float(msg.data[0]), float(msg.data[1]))

    def loop(self):
        if not self._ready():
            self._debug_waiting()
            return

        if self.ori0 is None:
            self.ori0 = list(self.feedback.ori[:3])
            self.initial_left_q = list(self.feedback.left_leg)
            self.initial_right_q = list(self.feedback.right_leg)
            self.prev_left_cmd = list(self.initial_left_q)
            self.prev_right_cmd = list(self.initial_right_q)
            self._init_arms()
            self.com.reset(self.params, 0.0, 0.0)
            if self.mode == "teleop_gui":
                self.get_logger().info("feedback ready; IMU zero initialized and teleop idle hold active")
            else:
                self.state_machine.start()
                self.get_logger().info("feedback ready; IMU zero initialized and crouch sequence started")

        if self.autotune is not None:
            self.autotune.start_if_needed()

        pitch = self.feedback.ori[0] - self.ori0[0]
        roll = self.feedback.ori[1] - self.ori0[1]
        gyro = list(self.feedback.gyro or [0.0, 0.0, 0.0])

        if self.mode == "teleop_gui" and self._teleop_hold_before_update(pitch, roll):
            return

        state = self.state_machine.update(self.dt, self.feedback, pitch, roll)
        self._handle_state_entry(state)
        if self.mode == "teleop_gui":
            self._update_teleop_runtime_status(state, pitch, roll)

        if state in (WalkState.STAND, WalkState.DONE):
            left_q, right_q = self._stand_recovery_targets(state)
            self._publish_legs(left_q, right_q)
            self._publish_arms([], [])
            self._debug(state, pitch, roll, self._foot_center(), self._foot_center())
            if self.autotune is not None:
                self.autotune.tick(pitch, roll, gyro, state)
            return

        current_step = self.footsteps.get_step(self.state_machine.step_index)
        next_step = None
        if self.mode == "teleop_gui" and self.active_profile.allow_continuous_steps:
            self.footsteps.ensure_steps(self.state_machine.step_index + 2)
        if self.state_machine.step_index + 1 < len(self.footsteps.steps):
            next_step = self.footsteps.get_step(self.state_machine.step_index + 1)

        zmp_phase = self.state_machine.double_support_phase()
        if state in (WalkState.LEFT_TOUCHDOWN, WalkState.RIGHT_TOUCHDOWN):
            zmp_phase = min(1.0, self.state_machine.state_t / max(0.1, self.params.touchdown_time))
        zmp_now = self.zmp_ref.zmp_for_state(state.name, current_step, zmp_phase, self.left_pose, self.right_pose)
        zmp_preview = self.zmp_ref.preview_zmp(current_step, next_step)
        if state in (WalkState.LEFT_SWING, WalkState.RIGHT_SWING):
            zmp_preview = zmp_now
        com_x, com_y, com_vx, com_vy, _, _ = self.com.update(self.dt, zmp_now, zmp_preview)

        left_target_pose, right_target_pose = self._foot_targets(state, current_step)
        pelvis_yaw = self._average_yaw(left_target_pose.yaw, right_target_pose.yaw)
        pelvis = Pose2D(com_x, com_y, self.params.pelvis_height, roll * 0.25, pitch * 0.25, pelvis_yaw)
        left_q, right_q = self.ik.solve(pelvis, left_target_pose, right_target_pose)

        stab = self.stabilizer.compute(
            pitch,
            roll,
            gyro,
            self.state_machine.support_foot(),
            com_vx,
            com_vy,
            len(self.left_arm_cmd),
            len(self.right_arm_cmd),
        )
        self.pending_step_adjustment = (stab.next_step_dx, stab.next_step_dy)

        left_q = self.ik.limit([left_q[i] + stab.left_add[i] for i in range(LEG_DOF)])
        right_q = self.ik.limit([right_q[i] + stab.right_add[i] for i in range(LEG_DOF)])

        if state == WalkState.CROUCH:
            u = smoothstep(self.state_machine.state_t / max(0.1, self.params.crouch_time))
            left_q = [lerp(self.initial_left_q[i], left_q[i], u) for i in range(LEG_DOF)]
            right_q = [lerp(self.initial_right_q[i], right_q[i], u) for i in range(LEG_DOF)]
        elif state == WalkState.ABORT:
            left_q = self.prev_left_cmd
            right_q = self.prev_right_cmd

        left_q = self._rate_limit(left_q, self.prev_left_cmd, self.params.max_joint_rate)
        right_q = self._rate_limit(right_q, self.prev_right_cmd, self.params.max_joint_rate)
        self.prev_left_cmd = left_q
        self.prev_right_cmd = right_q

        self._publish_legs(left_q, right_q)
        self._publish_arms(stab.left_arm_add, stab.right_arm_add)
        self._debug(state, pitch, roll, zmp_now, (com_x, com_y))
        if self.autotune is not None:
            self.autotune.tick(pitch, roll, gyro, state)

    def _teleop_hold_before_update(self, pitch: float, roll: float) -> bool:
        input_state = self.teleop_buffer.snapshot() if self.teleop_buffer else TeleopInputState()
        profile, status = resolve_profile_from_input(input_state)
        requested_name = requested_profile_name(input_state)
        self.pending_profile = profile
        self.teleop_requested_profile_name = requested_name
        self.teleop_status_message = status

        state = self.state_machine.state
        if input_state.emergency_stop:
            self.active_profile = get_profile("idle")
            self.teleop_active_profile_name = self.active_profile.name
            self._update_teleop_runtime_status(state, pitch, roll)
            self._publish_hold_commands()
            return True

        if input_state.pause:
            self.active_profile = get_profile("idle")
            self.teleop_active_profile_name = self.active_profile.name
            self._request_safe_stop_or_hold(state)
            if state in (WalkState.WAIT, WalkState.STAND, WalkState.DONE, WalkState.ABORT):
                self._update_teleop_runtime_status(state, pitch, roll)
                self._publish_hold_commands()
                return True
            return False

        if self._is_enabled_waist_profile(profile):
            if state == WalkState.ABORT:
                self.active_profile = get_profile("idle")
                self.teleop_active_profile_name = self.active_profile.name
                self.teleop_status_message = "Walker is ABORT; holding current command."
                self._update_teleop_runtime_status(state, pitch, roll)
                self._publish_hold_commands()
                return True

            self.active_profile = profile
            self.teleop_active_profile_name = profile.name
            if state in (WalkState.WAIT, WalkState.DONE):
                self._update_teleop_runtime_status(state, pitch, roll)
                self._publish_static_posture(pitch, roll, profile.waist_yaw_target or 0.0, profile.max_joint_rate)
                return True
            if state == WalkState.STAND:
                self.teleop_status_message = f"{profile.description} requested; waiting for stand recovery."
                return False
            self.teleop_status_message = f"{profile.description} requested; finishing the current safe step first."
            self._request_safe_stop_or_hold(state)
            return False

        if self._is_enabled_walk_profile(profile):
            if state == WalkState.ABORT:
                self.active_profile = get_profile("idle")
                self.teleop_active_profile_name = self.active_profile.name
                self.teleop_status_message = "Walker is ABORT; holding current command."
                self._update_teleop_runtime_status(state, pitch, roll)
                self._publish_hold_commands()
                return True

            if (
                not profile.allow_continuous_steps
                and self.active_profile.name == profile.name
                and state in (WalkState.STAND, WalkState.DONE)
            ):
                self.teleop_status_message = f"{profile.description} complete; press the key again to clear."
                if state == WalkState.DONE:
                    self._update_teleop_runtime_status(state, pitch, roll)
                    self._publish_hold_commands()
                    return True
                return False

            if state not in (WalkState.WAIT, WalkState.STAND, WalkState.DONE) and self.active_profile.name != profile.name:
                self.teleop_status_message = (
                    f"{profile.description} requested; finishing the current safe step before switching profiles."
                )
                self._request_safe_stop_or_hold(state)
                return False

            self.active_profile = profile
            self.teleop_active_profile_name = profile.name
            self.state_machine.clear_stop_request()
            if state in (WalkState.WAIT, WalkState.DONE):
                self._reset_teleop_walk_session()
                self.state_machine.start()
            elif state == WalkState.STAND:
                self._reset_teleop_walk_session()
                self.state_machine.start()
            return False

        self.active_profile = get_profile("idle")
        self.teleop_active_profile_name = self.active_profile.name
        self._request_safe_stop_or_hold(state)
        if state in (WalkState.WAIT, WalkState.STAND, WalkState.DONE, WalkState.ABORT):
            self._update_teleop_runtime_status(state, pitch, roll)
            if state != WalkState.ABORT and abs(self.commanded_waist_yaw) > 1e-3:
                self._publish_static_posture(pitch, roll, 0.0)
            else:
                self._publish_hold_commands()
            return True
        return False

    def _request_safe_stop_or_hold(self, state: WalkState) -> None:
        if state in (WalkState.CROUCH, WalkState.TRANSFER_TO_LEFT, WalkState.TRANSFER_TO_RIGHT):
            self._enter_stand_recovery()
        elif state in (
            WalkState.LEFT_SWING,
            WalkState.LEFT_TOUCHDOWN,
            WalkState.DOUBLE_SUPPORT_AFTER_LEFT,
            WalkState.RIGHT_SWING,
            WalkState.RIGHT_TOUCHDOWN,
            WalkState.DOUBLE_SUPPORT_AFTER_RIGHT,
        ):
            self.state_machine.request_stop_after_current_step()

    def _is_enabled_walk_profile(self, profile) -> bool:
        return profile.enabled and profile.name in ("forward_walk", "backward_walk", "turn_left", "turn_right", "fast_modifier")

    def _is_enabled_waist_profile(self, profile) -> bool:
        return profile.enabled and profile.name in ("waist_left", "waist_right")

    def _enter_stand_recovery(self) -> None:
        self.state_machine.clear_stop_request()
        self.state_machine._set(WalkState.STAND)
        self.stand_start_left_cmd = list(self.prev_left_cmd or self.feedback.left_leg)
        self.stand_start_right_cmd = list(self.prev_right_cmd or self.feedback.right_leg)

    def _reset_teleop_walk_session(self) -> None:
        self.params = walker_params_for_profile(self.active_profile)
        self.dt = self.params.dt
        self.footsteps.reset(self.params)
        self.zmp_ref.reset(self.params)
        self.com.reset(self.params, 0.0, 0.0)
        self.swing.reset(self.params)
        self.ik.reset(self.params)
        self.stabilizer.reset(self.params)
        self.state_machine.reset(self.params)
        self.state_machine.set_continuous_walk(self.active_profile.allow_continuous_steps)
        self.left_pose = self.footsteps.initial_left()
        self.right_pose = self.footsteps.initial_right()
        self.swing_start = {"left": self.left_pose.copy(), "right": self.right_pose.copy()}
        self.initial_left_q = list(self.feedback.left_leg)
        self.initial_right_q = list(self.feedback.right_leg)
        self.prev_left_cmd = list(self.feedback.left_leg)
        self.prev_right_cmd = list(self.feedback.right_leg)
        self.stand_start_left_cmd = None
        self.stand_start_right_cmd = None
        self.pending_step_adjustment = None
        self.last_state = self.state_machine.state
        self.commanded_waist_yaw = 0.0

    def _publish_hold_commands(self) -> None:
        left_q = list(self.prev_left_cmd or self.feedback.left_leg or [0.0] * LEG_DOF)
        right_q = list(self.prev_right_cmd or self.feedback.right_leg or [0.0] * LEG_DOF)
        self.prev_left_cmd = left_q
        self.prev_right_cmd = right_q
        self._publish_legs(left_q, right_q)

    def _publish_static_posture(self, pitch: float, roll: float, target_waist_yaw: float, max_joint_rate: float = None) -> None:
        max_rate = max_joint_rate if max_joint_rate is not None else self.params.max_joint_rate
        waist_rate = self.active_profile.waist_yaw_rate if self.active_profile.waist_yaw_rate is not None else 18.0 * D
        yaw_limit = waist_rate * self.dt
        self.commanded_waist_yaw += clamp(target_waist_yaw - self.commanded_waist_yaw, -yaw_limit, yaw_limit)

        center_x, center_y = self._foot_center()
        pelvis = Pose2D(
            center_x,
            center_y,
            self.params.pelvis_height,
            roll * 0.10,
            pitch * 0.10,
            self.commanded_waist_yaw,
        )
        left_q, right_q = self.ik.solve(pelvis, self.left_pose, self.right_pose)
        left_q = self._rate_limit(left_q, self.prev_left_cmd, max_rate)
        right_q = self._rate_limit(right_q, self.prev_right_cmd, max_rate)
        self.prev_left_cmd = left_q
        self.prev_right_cmd = right_q
        self._publish_legs(left_q, right_q)
        self._publish_arms([], [])

    def _update_teleop_runtime_status(self, state: WalkState, pitch: float, roll: float) -> None:
        self.teleop_state_name = state.name
        self.teleop_pitch_deg = pitch / D
        self.teleop_roll_deg = roll / D

    def teleop_status_snapshot(self) -> dict:
        pitch = "-" if self.teleop_pitch_deg is None else f"{self.teleop_pitch_deg:.2f} deg"
        roll = "-" if self.teleop_roll_deg is None else f"{self.teleop_roll_deg:.2f} deg"
        return {
            "requested_profile": self.teleop_requested_profile_name,
            "active_profile": self.teleop_active_profile_name,
            "status_message": self.teleop_status_message,
            "state": self.teleop_state_name,
            "pitch": pitch,
            "roll": roll,
        }

    def _foot_targets(self, state: WalkState, step) -> tuple:
        left_target = self.left_pose.copy()
        right_target = self.right_pose.copy()
        phase = self.state_machine.swing_phase()
        if state in (WalkState.LEFT_SWING, WalkState.LEFT_TOUCHDOWN):
            left_target = self.swing.pose(self.swing_start["left"], step.left_target, phase)
        elif state in (WalkState.RIGHT_SWING, WalkState.RIGHT_TOUCHDOWN):
            right_target = self.swing.pose(self.swing_start["right"], step.right_target, phase)
        return left_target, right_target

    def _handle_state_entry(self, state: WalkState) -> None:
        if state == self.last_state:
            return
        if self.last_state == WalkState.LEFT_TOUCHDOWN:
            self.left_pose = self.footsteps.get_step(self.state_machine.step_index).left_target.copy()
        if self.last_state == WalkState.RIGHT_TOUCHDOWN:
            self.right_pose = self.footsteps.get_step(self.state_machine.step_index).right_target.copy()

        if state == WalkState.LEFT_SWING:
            self.swing_start["left"] = self.left_pose.copy()
        elif state == WalkState.RIGHT_SWING:
            self.swing_start["right"] = self.right_pose.copy()
        elif state in (WalkState.DOUBLE_SUPPORT_AFTER_LEFT, WalkState.DOUBLE_SUPPORT_AFTER_RIGHT):
            self._apply_next_step_adjustment()
        elif state == WalkState.STAND:
            self.stand_start_left_cmd = list(self.prev_left_cmd or self.feedback.left_leg)
            self.stand_start_right_cmd = list(self.prev_right_cmd or self.feedback.right_leg)

        self.get_logger().info(f"state -> {state.name}, step={self.state_machine.step_index}")
        self.last_state = state

    def _stand_recovery_targets(self, state: WalkState) -> tuple:
        if self.stand_start_left_cmd is None:
            self.stand_start_left_cmd = list(self.prev_left_cmd or self.feedback.left_leg)
        if self.stand_start_right_cmd is None:
            self.stand_start_right_cmd = list(self.prev_right_cmd or self.feedback.right_leg)

        u = 1.0 if state == WalkState.DONE else smoothstep(self.state_machine.state_t / max(0.1, self.params.stand_time))
        left_target = [lerp(self.stand_start_left_cmd[i], self.initial_left_q[i], u) for i in range(LEG_DOF)]
        right_target = [lerp(self.stand_start_right_cmd[i], self.initial_right_q[i], u) for i in range(LEG_DOF)]
        left_q = self._rate_limit(left_target, self.prev_left_cmd, self.params.max_joint_rate * 0.55)
        right_q = self._rate_limit(right_target, self.prev_right_cmd, self.params.max_joint_rate * 0.55)
        self.prev_left_cmd = left_q
        self.prev_right_cmd = right_q
        return left_q, right_q

    def _foot_center(self) -> tuple:
        return (
            0.5 * (self.left_pose.x + self.right_pose.x),
            0.5 * (self.left_pose.y + self.right_pose.y),
        )

    def _average_yaw(self, left_yaw: float, right_yaw: float) -> float:
        return math.atan2(math.sin(left_yaw) + math.sin(right_yaw), math.cos(left_yaw) + math.cos(right_yaw))

    def _apply_next_step_adjustment(self) -> None:
        if not self.pending_step_adjustment:
            return
        dx, dy = self.pending_step_adjustment
        self.footsteps.modify_next_step(self.state_machine.step_index + 1, dx, dy)
        self.pending_step_adjustment = None

    def _ready(self) -> bool:
        return (
            self.feedback.ori is not None
            and self.feedback.gyro is not None
            and self.feedback.pos is not None
            and self.feedback.left_leg is not None
            and self.feedback.right_leg is not None
        )

    def _debug_waiting(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_debug_t).nanoseconds * 1e-9 > 2.0:
            missing = []
            for name, value in (
                ("ori", self.feedback.ori),
                ("angVel", self.feedback.gyro),
                ("pos", self.feedback.pos),
                ("leftLegJoints", self.feedback.left_leg),
                ("rightLegJoints", self.feedback.right_leg),
            ):
                if value is None:
                    missing.append(name)
            self.get_logger().info("waiting for feedback: " + ", ".join(missing))
            self.last_debug_t = now

    def _debug(self, state, pitch, roll, zmp, com) -> None:
        now = self.get_clock().now()
        if (now - self.last_debug_t).nanoseconds * 1e-9 < 0.8:
            return
        left_ratio, right_ratio = self.state_machine.force_ratio(self.feedback)
        force = "phase-fallback" if left_ratio is None else f"L={left_ratio:.2f} R={right_ratio:.2f}"
        self.get_logger().info(
            f"state={state.name} step={self.state_machine.step_index} support={self.state_machine.support_foot()} "
            f"pitch={pitch / D:.2f} roll={roll / D:.2f} zmp=({zmp[0]:.3f},{zmp[1]:.3f}) "
            f"com=({com[0]:.3f},{com[1]:.3f}) force={force}"
        )
        self.last_debug_t = now

    def _publish_legs(self, left_q, right_q) -> None:
        msg = Float64MultiArray()
        msg.data = [float(x) for x in left_q + right_q]
        self.leg_pub.publish(msg)

    def _publish_arms(self, left_add, right_add) -> None:
        if not self.left_arm_cmd or not self.right_arm_cmd:
            return
        left = self._rate_limit(
            [self.left_arm_base[i] + (left_add[i] if i < len(left_add) else 0.0) for i in range(len(self.left_arm_base))],
            self.left_arm_cmd,
            self.params.max_arm_rate,
        )
        right = self._rate_limit(
            [self.right_arm_base[i] + (right_add[i] if i < len(right_add) else 0.0) for i in range(len(self.right_arm_base))],
            self.right_arm_cmd,
            self.params.max_arm_rate,
        )
        self.left_arm_cmd = left
        self.right_arm_cmd = right
        msg = Float64MultiArray()
        msg.data = [float(x) for x in left + right]
        self.arm_pub.publish(msg)

    def _init_arms(self) -> None:
        self.left_arm_base = list(self.left_arm_fb or [])
        self.right_arm_base = list(self.right_arm_fb or [])
        self.left_arm_cmd = list(self.left_arm_base)
        self.right_arm_cmd = list(self.right_arm_base)

    def _rate_limit(self, target, previous, max_rate):
        if previous is None:
            return list(target)
        limit = max_rate * self.dt
        return [previous[i] + clamp(target[i] - previous[i], -limit, limit) for i in range(len(target))]

    def _force_value(self, data) -> float:
        if not data:
            return None
        if len(data) >= 3:
            return math.sqrt(data[0] ** 2 + data[1] ** 2 + data[2] ** 2)
        return abs(float(data[0]))

def main(args=None):
    if any(arg in ("-h", "--help") for arg in sys.argv[1:]):
        print("ASIMO-style ZMP walker for CoppeliaSim Asti")
        print("Usage:")
        print("  ros2 run robot_simulation_experiment main.py")
        print("  ros2 run robot_simulation_experiment main.py --ros-args -p mode:=walk")
        print("  ros2 run robot_simulation_experiment main.py --ros-args -p mode:=auto_tune")
        print("  ros2 run robot_simulation_experiment main.py --ros-args -p mode:=teleop_gui")
        return
    rclpy.init(args=args)
    teleop_buffer = TeleopCommandBuffer()
    node = AsimoStyleZMPWalker(teleop_buffer=teleop_buffer)
    try:
        if node.mode == "teleop_gui":
            _run_teleop_gui(node, teleop_buffer)
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if exc.__class__.__name__ != "RCLError":
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def _run_teleop_gui(node: AsimoStyleZMPWalker, teleop_buffer: TeleopCommandBuffer) -> None:
    spin_error = []

    def spin_node() -> None:
        try:
            rclpy.spin(node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        except Exception as exc:
            if exc.__class__.__name__ != "RCLError":
                spin_error.append(exc)

    def shutdown() -> None:
        if rclpy.ok():
            rclpy.shutdown()

    spin_thread = threading.Thread(target=spin_node, name="teleop_ros_spin", daemon=True)
    spin_thread.start()
    try:
        from asimo_walker.teleop_gui import TeleopGuiApp

        app = TeleopGuiApp(
            teleop_buffer,
            status_provider=node.teleop_status_snapshot,
            shutdown_callback=shutdown,
        )
        app.run()
    except Exception as exc:
        print(f"teleop_gui failed to start: {exc}", file=sys.stderr)
        shutdown()
    finally:
        shutdown()
        spin_thread.join(timeout=2.0)
        if spin_error:
            raise spin_error[0]


if __name__ == "__main__":
    main()
