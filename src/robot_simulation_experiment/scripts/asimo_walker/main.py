#!/usr/bin/python3
import math
import os
import sys

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
from asimo_walker.zmp_preview import ZMPPreviewPlanner
from asimo_walker.zmp_reference import ZMPReferencePlanner


class AsimoStyleZMPWalker(Node):
    def __init__(self):
        super().__init__("asimo_style_zmp_walker")
        self.declare_parameter("mode", "walk")
        self.mode = str(self.get_parameter("mode").value)

        # WalkerParams is created before any motion state so both walk and
        # auto-tune modes start from the same defaults.
        self.params = WalkerParams()
        self.dt = self.params.dt
        self.feedback = RobotFeedback()
        self.ori0 = None

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
            self.state_machine.start()
            self.get_logger().info("feedback ready; IMU zero initialized and crouch sequence started")

        if self.autotune is not None:
            self.autotune.start_if_needed()

        pitch = self.feedback.ori[0] - self.ori0[0]
        roll = self.feedback.ori[1] - self.ori0[1]
        gyro = list(self.feedback.gyro or [0.0, 0.0, 0.0])

        state = self.state_machine.update(self.dt, self.feedback, pitch, roll)
        self._handle_state_entry(state)

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

        pelvis = Pose2D(com_x, com_y, self.params.pelvis_height, roll * 0.25, pitch * 0.25, 0.0)
        left_target_pose, right_target_pose = self._foot_targets(state, current_step)
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
        print("  ros2 run robot_simulation_experiment asimo_style_zmp_walker")
        print("  ros2 run robot_simulation_experiment asimo_style_zmp_walker --ros-args -p mode:=auto_tune")
        return
    rclpy.init(args=args)
    node = AsimoStyleZMPWalker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if exc.__class__.__name__ != "RCLError":
            raise
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
