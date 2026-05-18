#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def deg(x: float) -> float:
    return x * math.pi / 180.0


def clamp(x: float, low: float, high: float) -> float:
    return max(low, min(high, x))


class HumanoidSafeControllerROS2(Node):
    """
    Safe quasi-static humanoid gait controller for the CoppeliaSim humanoid model.

    Input:
        /robot/ori          Float64MultiArray, [roll/pitch/yaw-like euler]
        /robot/angVel       Float64MultiArray
        /robot/acc          Float64MultiArray
        /robot/vel          Float64MultiArray
        /robot/pos          Float64MultiArray
        /leftLegJoints      Float64MultiArray, 6 joints
        /rightLegJoints     Float64MultiArray, 6 joints

    Output:
        /legTargetJoints    Float64MultiArray, 12 joints:
                            left 6 + right 6
    """

    def __init__(self):
        super().__init__("humanoid_safe_controller_ros2")

        # =========================
        # ROS2 communication
        # =========================
        self.pub = self.create_publisher(
            Float64MultiArray,
            "/legTargetJoints",
            10
        )

        self.create_subscription(Float64MultiArray, "/robot/ori", self.cb_ori, 10)
        self.create_subscription(Float64MultiArray, "/robot/angVel", self.cb_ang_vel, 10)
        self.create_subscription(Float64MultiArray, "/robot/acc", self.cb_acc, 10)
        self.create_subscription(Float64MultiArray, "/robot/vel", self.cb_vel, 10)
        self.create_subscription(Float64MultiArray, "/robot/pos", self.cb_pos, 10)

        # 如果你的 CoppeliaSim 腿部脚本已经发布这两个话题，这两个非常重要：
        # 用于防止程序一启动就从错误初值跳到目标值。
        self.create_subscription(Float64MultiArray, "/leftLegJoints", self.cb_left_joints, 10)
        self.create_subscription(Float64MultiArray, "/rightLegJoints", self.cb_right_joints, 10)

        # =========================
        # Sensor state
        # =========================
        self.ori = np.zeros(3)
        self.ang_vel = np.zeros(3)
        self.acc = np.zeros(3)
        self.vel = np.zeros(3)
        self.pos = np.zeros(3)

        self.ori_zero = None
        self.imu_ready = False

        self.left_joint_feedback = None
        self.right_joint_feedback = None
        self.joint_feedback_ready = False

        # =========================
        # Timing
        # =========================
        self.frequency = 20.0
        self.dt = 1.0 / self.frequency
        self.timer = self.create_timer(self.dt, self.control_loop)

        # =========================
        # Safety and smoothness
        # =========================
        self.max_joint_abs = deg(55.0)

        # 每个控制周期最大关节变化量。20Hz 下 0.7deg/tick ≈ 14deg/s。
        # 这是防止一启动就倒的关键。
        self.max_delta_per_tick = deg(0.7)

        # 站稳判定阈值
        self.stable_pitch_th = deg(4.0)
        self.stable_roll_th = deg(4.0)
        self.stable_rate_th = deg(8.0)

        # 倾倒保护阈值
        self.recover_pitch_th = deg(12.0)
        self.recover_roll_th = deg(12.0)
        self.abort_pitch_th = deg(22.0)
        self.abort_roll_th = deg(22.0)

        # IMU feedback compensation
        # 保留课件/原代码思路：[姿态误差, 角速度] -> 支撑腿补偿
        self.K_pitch = np.array([0.30, 0.08])
        self.K_roll = np.array([0.28, 0.07])
        self.max_comp = deg(2.5)

        # 如果发现越补越倒，优先改这两个符号。
        self.pitch_sign = 1.0
        self.roll_sign = 1.0

        # =========================
        # Nominal postures
        # 单腿 6 维：沿用你 CoppeliaSim 模型顺序
        # [q1, q2, q3, q4, q5, q6]
        # =========================
        self.stand_left = np.array([
            0.0,
            0.0,
            deg(-10.0),
            deg(20.0),
            0.0,
            deg(-10.0)
        ])

        self.stand_right = np.array([
            0.0,
            0.0,
            deg(-10.0),
            deg(20.0),
            0.0,
            deg(-10.0)
        ])

        # 稍微下蹲，降低重心，比直腿更稳
        self.crouch_left = np.array([
            0.0,
            0.0,
            deg(-13.0),
            deg(26.0),
            0.0,
            deg(-13.0)
        ])

        self.crouch_right = np.array([
            0.0,
            0.0,
            deg(-13.0),
            deg(26.0),
            0.0,
            deg(-13.0)
        ])

        # 当前发送目标
        self.left_target = self.crouch_left.copy()
        self.right_target = self.crouch_right.copy()

        # =========================
        # Gait parameters
        # =========================
        self.total_steps = 8          # 8 个半步，先别太多
        self.step_count = 0

        # 每个状态至少保持的 tick。20Hz 下 30 tick = 1.5s。
        self.phase_duration = 30
        self.double_support_duration = 20
        self.start_hold_duration = 60
        self.stop_hold_duration = 80
        self.twist_duration = 100

        self.phase_tick = 0
        self.global_tick = 0
        self.stable_counter = 0
        self.required_stable_ticks = 8

        self.mode = "WAIT"
        self.finished = False

        self.get_logger().info("Humanoid safe ROS2 controller started.")
        self.get_logger().info("Waiting for IMU and joint feedback...")

    # =========================
    # Callbacks
    # =========================
    def cb_ori(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.size >= 3:
            self.ori = data[:3]

            if self.ori_zero is None:
                self.ori_zero = self.ori.copy()
                self.imu_ready = True
                self.get_logger().info(f"IMU zero set to {self.ori_zero.tolist()}")

    def cb_ang_vel(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.size >= 3:
            self.ang_vel = data[:3]

    def cb_acc(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.size >= 3:
            self.acc = data[:3]

    def cb_vel(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.size >= 3:
            self.vel = data[:3]

    def cb_pos(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.size >= 3:
            self.pos = data[:3]

    def cb_left_joints(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.size >= 6:
            self.left_joint_feedback = data[:6]
            self.check_joint_feedback_ready()

    def cb_right_joints(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.size >= 6:
            self.right_joint_feedback = data[:6]
            self.check_joint_feedback_ready()

    def check_joint_feedback_ready(self):
        if self.left_joint_feedback is not None and self.right_joint_feedback is not None:
            if not self.joint_feedback_ready:
                self.joint_feedback_ready = True

                # 关键：从实际关节反馈初始化目标，防止一开始跳变。
                self.left_target = self.left_joint_feedback.copy()
                self.right_target = self.right_joint_feedback.copy()

                self.get_logger().info("Joint feedback ready. Targets initialized from current joints.")

    # =========================
    # Utility
    # =========================
    def set_mode(self, mode: str):
        if self.mode != mode:
            self.mode = mode
            self.phase_tick = 0
            self.stable_counter = 0
            self.get_logger().info(f"Switch mode -> {mode}")

    def get_pitch_roll_error(self):
        if self.ori_zero is None:
            return 0.0, 0.0

        # 根据你之前的代码，sensor_ori[0]、sensor_ori[1] 用作 x/y 倾斜控制。
        pitch = self.ori[0] - self.ori_zero[0]
        roll = self.ori[1] - self.ori_zero[1]
        return pitch, roll

    def is_stable(self):
        pitch, roll = self.get_pitch_roll_error()
        pitch_rate = self.ang_vel[0]
        roll_rate = self.ang_vel[1]

        return (
            abs(pitch) < self.stable_pitch_th
            and abs(roll) < self.stable_roll_th
            and abs(pitch_rate) < self.stable_rate_th
            and abs(roll_rate) < self.stable_rate_th
        )

    def safety_level(self):
        pitch, roll = self.get_pitch_roll_error()

        if abs(pitch) > self.abort_pitch_th or abs(roll) > self.abort_roll_th:
            return "ABORT"

        if abs(pitch) > self.recover_pitch_th or abs(roll) > self.recover_roll_th:
            return "RECOVER"

        return "OK"

    def compute_imu_compensation(self):
        """
        类 LQR / PD state feedback:
        u = -K * [angle_error, angular_rate]

        这里输出 6 维腿部补偿，主要加在支撑腿。
        """
        if not self.imu_ready:
            return np.zeros(6)

        pitch, roll = self.get_pitch_roll_error()

        pitch_rate = self.ang_vel[0]
        roll_rate = self.ang_vel[1]

        # 注意这里保守，补偿很小。
        u_pitch = -float(np.dot(self.K_pitch, np.array([pitch, pitch_rate]))) * self.pitch_sign
        u_roll = -float(np.dot(self.K_roll, np.array([roll, roll_rate]))) * self.roll_sign

        u_pitch = clamp(u_pitch, -self.max_comp, self.max_comp)
        u_roll = clamp(u_roll, -self.max_comp, self.max_comp)

        # 结合你原代码：补偿主要作用在第2、第3关节。
        # 同时给踝关节一点反向补偿，减少只靠髋关节导致的摆动。
        comp = np.array([
            0.0,
            u_roll,
            u_pitch,
            -0.20 * u_pitch,
            -0.25 * u_roll,
            -0.35 * u_pitch
        ])

        return comp

    def rate_limit(self, current, desired):
        desired = np.clip(desired, -self.max_joint_abs, self.max_joint_abs)
        delta = desired - current
        delta = np.clip(delta, -self.max_delta_per_tick, self.max_delta_per_tick)
        return current + delta

    def update_targets(self, left_desired, right_desired, support="both"):
        comp = self.compute_imu_compensation()

        if support == "left":
            left_desired = left_desired + comp
            right_desired = right_desired + 0.10 * comp
        elif support == "right":
            right_desired = right_desired + comp
            left_desired = left_desired + 0.10 * comp
        else:
            left_desired = left_desired + 0.35 * comp
            right_desired = right_desired + 0.35 * comp

        self.left_target = self.rate_limit(self.left_target, left_desired)
        self.right_target = self.rate_limit(self.right_target, right_desired)

    def publish_targets(self):
        msg = Float64MultiArray()
        msg.data = np.concatenate((self.left_target, self.right_target)).astype(float).tolist()
        self.pub.publish(msg)

    # =========================
    # Gait references
    # =========================
    def ref_shift_right(self):
        """
        重心慢慢移到右脚。左脚准备抬。
        """
        left = self.crouch_left.copy()
        right = self.crouch_right.copy()

        left[1] = deg(5.0)
        right[1] = deg(-7.0)
        right[4] = deg(-4.0)

        return left, right, "right"

    def ref_left_swing(self):
        """
        左脚小幅摆动。非常保守，不要大跨步。
        """
        left = self.crouch_left.copy()
        right = self.crouch_right.copy()

        # 左腿抬脚：髋 pitch、膝、踝协同
        left[2] = deg(-20.0)
        left[3] = deg(34.0)
        left[5] = deg(3.0)

        # 右腿支撑略微前倾/屈膝
        right[1] = deg(-6.0)
        right[2] = deg(-9.0)
        right[3] = deg(24.0)
        right[4] = deg(-4.0)
        right[5] = deg(-13.0)

        return left, right, "right"

    def ref_left_land(self):
        """
        左脚落地，双支撑。
        """
        left = self.crouch_left.copy()
        right = self.crouch_right.copy()

        left[2] = deg(-14.0)
        left[3] = deg(26.0)
        left[5] = deg(-12.0)

        return left, right, "both"

    def ref_shift_left(self):
        """
        重心移到左脚。右脚准备抬。
        """
        left = self.crouch_left.copy()
        right = self.crouch_right.copy()

        left[1] = deg(7.0)
        left[4] = deg(4.0)
        right[1] = deg(-5.0)

        return left, right, "left"

    def ref_right_swing(self):
        """
        右脚小幅摆动。
        """
        left = self.crouch_left.copy()
        right = self.crouch_right.copy()

        right[2] = deg(-20.0)
        right[3] = deg(34.0)
        right[5] = deg(3.0)

        left[1] = deg(6.0)
        left[2] = deg(-9.0)
        left[3] = deg(24.0)
        left[4] = deg(4.0)
        left[5] = deg(-13.0)

        return left, right, "left"

    def ref_right_land(self):
        """
        右脚落地，双支撑。
        """
        left = self.crouch_left.copy()
        right = self.crouch_right.copy()

        right[2] = deg(-14.0)
        right[3] = deg(26.0)
        right[5] = deg(-12.0)

        return left, right, "both"

    def ref_twist(self):
        """
        小幅扭腰。
        当前模型没有单独腰关节时，用左右腿第1关节反向模拟。
        """
        left = self.crouch_left.copy()
        right = self.crouch_right.copy()

        phase = 2.0 * math.pi * self.phase_tick / 50.0
        twist = deg(4.5) * math.sin(phase)

        left[0] = twist
        right[0] = -twist

        return left, right, "both"

    # =========================
    # Main loop
    # =========================
    def control_loop(self):
        self.global_tick += 1

        # 等待数据
        if not self.imu_ready or not self.joint_feedback_ready:
            self.publish_targets()
            return

        # 第一次进入
        if self.mode == "WAIT":
            self.set_mode("STAND_INIT")

        level = self.safety_level()

        if level == "ABORT":
            # 姿态太大，说明已经快倒或已倒，不再继续给步态。
            self.set_mode("ABORT")
        elif level == "RECOVER" and self.mode not in ["RECOVER", "ABORT"]:
            self.set_mode("RECOVER")

        # 稳定计数
        if self.is_stable():
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        # 根据状态生成目标
        if self.mode == "STAND_INIT":
            left, right, support = self.crouch_left, self.crouch_right, "both"
            self.update_targets(left, right, support)

            if self.phase_tick > self.start_hold_duration and self.stable_counter >= self.required_stable_ticks:
                self.set_mode("SHIFT_RIGHT")

        elif self.mode == "SHIFT_RIGHT":
            left, right, support = self.ref_shift_right()
            self.update_targets(left, right, support)

            if self.phase_tick > self.phase_duration and self.stable_counter >= self.required_stable_ticks:
                self.set_mode("LEFT_SWING")

        elif self.mode == "LEFT_SWING":
            left, right, support = self.ref_left_swing()
            self.update_targets(left, right, support)

            if self.phase_tick > self.phase_duration and self.stable_counter >= self.required_stable_ticks:
                self.set_mode("LEFT_LAND")

        elif self.mode == "LEFT_LAND":
            left, right, support = self.ref_left_land()
            self.update_targets(left, right, support)

            if self.phase_tick > self.double_support_duration and self.stable_counter >= self.required_stable_ticks:
                self.step_count += 1
                if self.step_count >= self.total_steps:
                    self.set_mode("TWIST")
                else:
                    self.set_mode("SHIFT_LEFT")

        elif self.mode == "SHIFT_LEFT":
            left, right, support = self.ref_shift_left()
            self.update_targets(left, right, support)

            if self.phase_tick > self.phase_duration and self.stable_counter >= self.required_stable_ticks:
                self.set_mode("RIGHT_SWING")

        elif self.mode == "RIGHT_SWING":
            left, right, support = self.ref_right_swing()
            self.update_targets(left, right, support)

            if self.phase_tick > self.phase_duration and self.stable_counter >= self.required_stable_ticks:
                self.set_mode("RIGHT_LAND")

        elif self.mode == "RIGHT_LAND":
            left, right, support = self.ref_right_land()
            self.update_targets(left, right, support)

            if self.phase_tick > self.double_support_duration and self.stable_counter >= self.required_stable_ticks:
                self.step_count += 1
                if self.step_count >= self.total_steps:
                    self.set_mode("TWIST")
                else:
                    self.set_mode("SHIFT_RIGHT")

        elif self.mode == "TWIST":
            left, right, support = self.ref_twist()
            self.update_targets(left, right, support)

            if self.phase_tick > self.twist_duration and self.stable_counter >= self.required_stable_ticks:
                self.set_mode("STOP")

        elif self.mode == "STOP":
            left, right, support = self.crouch_left, self.crouch_right, "both"
            self.update_targets(left, right, support)

            if self.phase_tick > self.stop_hold_duration:
                self.finished = True
                self.get_logger().info("Finished: walk + twist + stable stop.")

        elif self.mode == "RECOVER":
            # 恢复时只回小蹲站姿，不继续走
            left, right, support = self.crouch_left, self.crouch_right, "both"
            self.update_targets(left, right, support)

            if self.phase_tick > 80 and self.stable_counter >= self.required_stable_ticks:
                self.set_mode("STOP")

        elif self.mode == "ABORT":
            # 快倒时，不再走。继续给小蹲目标，避免关节失控。
            left, right, support = self.crouch_left, self.crouch_right, "both"
            self.update_targets(left, right, support)

        else:
            left, right, support = self.crouch_left, self.crouch_right, "both"
            self.update_targets(left, right, support)

        self.publish_targets()
        self.phase_tick += 1

        if self.global_tick % int(self.frequency) == 0:
            pitch, roll = self.get_pitch_roll_error()
            self.get_logger().info(
                f"mode={self.mode}, step={self.step_count}/{self.total_steps}, "
                f"pitch={pitch:.4f}, roll={roll:.4f}, "
                f"angVel=({self.ang_vel[0]:.4f}, {self.ang_vel[1]:.4f}), "
                f"stable_count={self.stable_counter}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidSafeControllerROS2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()