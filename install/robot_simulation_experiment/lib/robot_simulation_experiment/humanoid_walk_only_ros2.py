#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


D = math.pi / 180.0
LEG_DOF = 6


def deg(x):
    return x * D


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def smoothstep(x):
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def mix(a, b, u):
    return [a[i] + (b[i] - a[i]) * u for i in range(len(a))]


class AstiSimpleWalk(Node):
    """Small quasi-static Asti walker.

    Goal:
    left lift -> left forward -> land -> stable,
    right lift -> right forward -> land -> stable,
    repeat 4 foot placements, then stand.
    """

    def __init__(self):
        super().__init__("asti_simple_walk")

        self.leg_pub = self.create_publisher(Float64MultiArray, "/legTargetJoints", 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, "/armTargetJoints", 10)

        self.create_subscription(Float64MultiArray, "/robot/ori", self.cb_ori, 10)
        self.create_subscription(Float64MultiArray, "/robot/angVel", self.cb_gyr, 10)
        self.create_subscription(Float64MultiArray, "/robot/pos", self.cb_pos, 10)
        self.create_subscription(Float64MultiArray, "/leftLegJoints", self.cb_left_leg, 10)
        self.create_subscription(Float64MultiArray, "/rightLegJoints", self.cb_right_leg, 10)
        self.create_subscription(Float64MultiArray, "/leftArmJoints", self.cb_left_arm, 10)
        self.create_subscription(Float64MultiArray, "/rightArmJoints", self.cb_right_arm, 10)

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.loop)
        self.last_t = self.now()

        self.ori = [0.0, 0.0, 0.0]
        self.gyr = [0.0, 0.0, 0.0]
        self.pos = [0.0, 0.0, 0.0]
        self.ori0 = None
        self.forward_xy = (0.0, -1.0)

        self.left_fb = None
        self.right_fb = None
        self.left_arm_fb = None
        self.right_arm_fb = None
        self.left_arm_base = None
        self.right_arm_base = None
        self.left_arm_cmd = None
        self.right_arm_cmd = None

        self.state = "WAIT"
        self.phase_t = 0.0
        self.step_count = 0
        self.step_sign = 1.0
        self.walk_start_pos = None
        self.cycle_start_pos = None
        self.last_cycle_progress = 0.0
        self.direction_flips = 0

        self.cmd_l = self.crouch_pose()
        self.cmd_r = self.crouch_pose()
        self.phase_l0 = self.cmd_l[:]
        self.phase_r0 = self.cmd_r[:]

        # Tuned conservative parameters. Keep this block small and easy to edit.
        self.total_steps = 4
        self.step_hip = deg(5.6)
        self.lift_knee = deg(8.2)
        self.shift_roll = deg(5.2)
        self.support_ankle_roll = deg(3.0)
        self.balance_gain_pitch = 0.42
        self.balance_gain_roll = 0.38
        self.max_balance = deg(5.0)
        self.min_cycle_progress = 0.010

        self.durations = {
            "CROUCH": 2.4,
            "SHIFT_RIGHT": 1.4,
            "LIFT_LEFT": 1.0,
            "PLACE_LEFT": 1.0,
            "DOUBLE_LEFT": 0.8,
            "SHIFT_LEFT": 1.4,
            "LIFT_RIGHT": 1.0,
            "PLACE_RIGHT": 1.0,
            "DOUBLE_RIGHT": 0.8,
            "STAND": 2.0,
            "DONE": 1.0,
        }

        self.get_logger().info("Simple Asti walker started. Waiting for feedback.")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_ori(self, msg):
        if len(msg.data) >= 3:
            self.ori = list(msg.data[:3])
            if self.ori0 is None:
                if abs(self.ori[0]) > deg(6.0) or abs(self.ori[1]) > deg(6.0):
                    self.get_logger().warn("Robot is tilted at start. Reset CoppeliaSim before walking.")
                    return
                self.ori0 = self.ori[:]
                yaw = self.ori0[2]
                self.forward_xy = (math.sin(yaw), -math.cos(yaw))
                self.get_logger().info(
                    "IMU zero set. Facing forward vector=(%.3f, %.3f)"
                    % self.forward_xy
                )

    def cb_gyr(self, msg):
        if len(msg.data) >= 2:
            self.gyr = list(msg.data[:3])

    def cb_pos(self, msg):
        if len(msg.data) >= 3:
            self.pos = list(msg.data[:3])

    def cb_left_leg(self, msg):
        if len(msg.data) >= LEG_DOF:
            self.left_fb = list(msg.data[:LEG_DOF])

    def cb_right_leg(self, msg):
        if len(msg.data) >= LEG_DOF:
            self.right_fb = list(msg.data[:LEG_DOF])

    def cb_left_arm(self, msg):
        if msg.data:
            self.left_arm_fb = list(msg.data)
            self.init_arms()

    def cb_right_arm(self, msg):
        if msg.data:
            self.right_arm_fb = list(msg.data)
            self.init_arms()

    def init_arms(self):
        if self.left_arm_base is None and self.left_arm_fb and self.right_arm_fb:
            self.left_arm_base = self.left_arm_fb[:]
            self.right_arm_base = self.right_arm_fb[:]
            self.left_arm_cmd = self.left_arm_base[:]
            self.right_arm_cmd = self.right_arm_base[:]

    def ready(self):
        return self.ori0 is not None and self.left_fb is not None and self.right_fb is not None

    def crouch_pose(self):
        return [0.0, 0.0, deg(-14.0), deg(28.0), 0.0, deg(-14.0)]

    def stand_pose(self):
        return [0.0, 0.0, deg(-8.0), deg(16.0), 0.0, deg(-8.0)]

    def set_state(self, state):
        if self.state == state:
            return
        self.state = state
        self.phase_t = 0.0
        self.phase_l0 = self.cmd_l[:]
        self.phase_r0 = self.cmd_r[:]
        self.get_logger().info("State -> %s" % state)

    def pitch_roll(self):
        if self.ori0 is None:
            return 0.0, 0.0
        return self.ori[0] - self.ori0[0], self.ori[1] - self.ori0[1]

    def stable_enough(self, single_support=False):
        pitch, roll = self.pitch_roll()
        roll_limit = deg(8.8 if single_support else 5.8)
        return (
            abs(pitch) < deg(6.0)
            and abs(roll) < roll_limit
            and abs(self.gyr[0]) < deg(16.0)
            and abs(self.gyr[1]) < deg(16.0)
        )

    def forward_progress(self, start_pos):
        if start_pos is None:
            return 0.0
        dx = self.pos[0] - start_pos[0]
        dy = self.pos[1] - start_pos[1]
        return dx * self.forward_xy[0] + dy * self.forward_xy[1]

    def limit_leg(self, q):
        limits = [
            (deg(-12), deg(12)),
            (deg(-18), deg(18)),
            (deg(-38), deg(18)),
            (deg(6), deg(54)),
            (deg(-18), deg(18)),
            (deg(-36), deg(18)),
        ]
        return [clamp(q[i], limits[i][0], limits[i][1]) for i in range(LEG_DOF)]

    def rate_limit(self, current, target, dt):
        target = self.limit_leg(target)
        rates = [deg(25), deg(24), deg(30), deg(34), deg(28), deg(34)]
        out = []
        for i in range(LEG_DOF):
            out.append(current[i] + clamp(target[i] - current[i], -rates[i] * dt, rates[i] * dt))
        return out

    def balance(self, ql, qr, support):
        pitch, roll = self.pitch_roll()
        pitch_u = clamp(self.balance_gain_pitch * pitch + 0.06 * self.gyr[0], -self.max_balance, self.max_balance)
        roll_u = clamp(-(self.balance_gain_roll * roll + 0.06 * self.gyr[1]), -self.max_balance, self.max_balance)

        def add_pitch(q, w):
            q[2] += w * pitch_u
            q[5] -= 0.8 * w * pitch_u

        def add_roll(q, w):
            q[1] += w * roll_u
            q[4] -= 0.6 * w * roll_u

        if support == "left":
            add_pitch(ql, 1.0)
            add_roll(ql, 1.0)
            add_pitch(qr, 0.25)
            add_roll(qr, 0.15)
        elif support == "right":
            add_pitch(qr, 1.0)
            add_roll(qr, 1.0)
            add_pitch(ql, 0.25)
            add_roll(ql, 0.15)
        else:
            add_pitch(ql, 0.5)
            add_pitch(qr, 0.5)
            add_roll(ql, 0.45)
            add_roll(qr, 0.45)

    def shift_pose(self, support):
        ql = self.crouch_pose()
        qr = self.crouch_pose()
        if support == "right":
            ql[1] = deg(3.5)
            qr[1] = -self.shift_roll
            qr[4] = -self.support_ankle_roll
        else:
            ql[1] = self.shift_roll
            ql[4] = self.support_ankle_roll
            qr[1] = deg(-3.5)
        return ql, qr

    def lift_left_pose(self):
        ql, qr = self.shift_pose("right")
        ql[2] += self.step_sign * deg(1.2)
        ql[3] += self.lift_knee
        ql[5] += -self.step_sign * deg(1.2) - self.lift_knee
        return ql, qr

    def place_left_pose(self):
        ql, qr = self.shift_pose("right")
        ql[2] += self.step_sign * self.step_hip
        ql[5] -= self.step_sign * self.step_hip
        return ql, qr

    def lift_right_pose(self):
        ql, qr = self.shift_pose("left")
        qr[2] += self.step_sign * deg(1.2)
        qr[3] += self.lift_knee
        qr[5] += -self.step_sign * deg(1.2) - self.lift_knee
        return ql, qr

    def place_right_pose(self):
        ql, qr = self.shift_pose("left")
        # Right leg uses a shorter forward stride; this avoids undoing the left step.
        qr[2] += self.step_sign * self.step_hip * 0.35
        qr[5] -= self.step_sign * self.step_hip * 0.35
        return ql, qr

    def reference(self):
        u = smoothstep(self.phase_t / max(self.durations.get(self.state, 1.0), 1e-6))
        support = "both"

        if self.state == "CROUCH":
            ql, qr = self.crouch_pose(), self.crouch_pose()
        elif self.state == "SHIFT_RIGHT":
            ql, qr = self.shift_pose("right")
            support = "right"
        elif self.state == "LIFT_LEFT":
            ql, qr = self.lift_left_pose()
            support = "right"
        elif self.state == "PLACE_LEFT":
            ql, qr = self.place_left_pose()
            support = "right"
        elif self.state == "DOUBLE_LEFT":
            ql, qr = self.crouch_pose(), self.crouch_pose()
        elif self.state == "SHIFT_LEFT":
            ql, qr = self.shift_pose("left")
            support = "left"
        elif self.state == "LIFT_RIGHT":
            ql, qr = self.lift_right_pose()
            support = "left"
        elif self.state == "PLACE_RIGHT":
            ql, qr = self.place_right_pose()
            support = "left"
        elif self.state == "DOUBLE_RIGHT":
            ql, qr = self.crouch_pose(), self.crouch_pose()
        elif self.state in ("STAND", "DONE"):
            ql, qr = self.stand_pose(), self.stand_pose()
        else:
            ql, qr = self.crouch_pose(), self.crouch_pose()

        return mix(self.phase_l0, ql, u), mix(self.phase_r0, qr, u), support

    def update_state(self):
        t = self.phase_t
        if self.state == "CROUCH" and t > self.durations["CROUCH"] and self.stable_enough():
            self.set_state("SHIFT_RIGHT")
        elif self.state == "SHIFT_RIGHT" and t > self.durations["SHIFT_RIGHT"] and self.stable_enough(True):
            self.set_state("LIFT_LEFT")
        elif self.state == "LIFT_LEFT" and t > self.durations["LIFT_LEFT"] and self.stable_enough(True):
            self.set_state("PLACE_LEFT")
        elif self.state == "PLACE_LEFT" and t > self.durations["PLACE_LEFT"]:
            self.set_state("DOUBLE_LEFT")
        elif self.state == "DOUBLE_LEFT" and t > self.durations["DOUBLE_LEFT"] and self.stable_enough():
            self.step_count += 1
            self.set_state("SHIFT_LEFT")
        elif self.state == "SHIFT_LEFT" and t > self.durations["SHIFT_LEFT"] and self.stable_enough(True):
            self.set_state("LIFT_RIGHT")
        elif self.state == "LIFT_RIGHT" and t > self.durations["LIFT_RIGHT"] and self.stable_enough(True):
            self.set_state("PLACE_RIGHT")
        elif self.state == "PLACE_RIGHT" and t > self.durations["PLACE_RIGHT"]:
            self.set_state("DOUBLE_RIGHT")
        elif self.state == "DOUBLE_RIGHT" and t > self.durations["DOUBLE_RIGHT"] and self.stable_enough():
            self.step_count += 1
            cycle = self.forward_progress(self.cycle_start_pos)
            self.last_cycle_progress = cycle
            self.get_logger().info("Cycle progress along facing direction: %.3f m" % cycle)
            if cycle < self.min_cycle_progress and self.direction_flips < 1:
                self.step_sign *= -1.0
                self.direction_flips += 1
                self.step_count = 0
                self.walk_start_pos = self.pos[:]
                self.get_logger().warn("Cycle went backward. Flipped step_sign to %.0f and restarted count." % self.step_sign)
            self.cycle_start_pos = self.pos[:]
            if self.step_count >= self.total_steps:
                total = self.forward_progress(self.walk_start_pos)
                if total > 0.02:
                    self.get_logger().info("Completed 4 steps forward. Standing.")
                    self.set_state("STAND")
                else:
                    self.get_logger().warn("4 steps done but net progress %.3f m; continuing." % total)
                    self.step_count = 2
                    self.set_state("SHIFT_RIGHT")
            else:
                self.set_state("SHIFT_RIGHT")
        elif self.state == "STAND" and t > self.durations["STAND"] and self.stable_enough():
            self.set_state("DONE")

    def publish_arms(self, dt):
        if self.left_arm_base is None or self.right_arm_base is None:
            return
        phase = 0.0
        if self.state in ("LIFT_LEFT", "PLACE_LEFT", "DOUBLE_LEFT"):
            phase = 1.0
        elif self.state in ("LIFT_RIGHT", "PLACE_RIGHT", "DOUBLE_RIGHT"):
            phase = -1.0
        amp = deg(5.0) * phase
        ldes = self.left_arm_base[:]
        rdes = self.right_arm_base[:]
        if ldes:
            ldes[0] -= amp
        if rdes:
            rdes[0] += amp
        rate = deg(45.0) * dt
        self.left_arm_cmd = [self.left_arm_cmd[i] + clamp(ldes[i] - self.left_arm_cmd[i], -rate, rate) for i in range(len(ldes))]
        self.right_arm_cmd = [self.right_arm_cmd[i] + clamp(rdes[i] - self.right_arm_cmd[i], -rate, rate) for i in range(len(rdes))]
        msg = Float64MultiArray()
        msg.data = [float(x) for x in self.left_arm_cmd + self.right_arm_cmd]
        self.arm_pub.publish(msg)

    def loop(self):
        now = self.now()
        dt = clamp(now - self.last_t, 0.005, 0.05)
        self.last_t = now

        if not self.ready():
            return
        if self.state == "WAIT":
            self.cmd_l = self.left_fb[:]
            self.cmd_r = self.right_fb[:]
            self.walk_start_pos = self.pos[:]
            self.cycle_start_pos = self.pos[:]
            self.set_state("CROUCH")

        ql, qr, support = self.reference()
        self.balance(ql, qr, support)
        self.cmd_l = self.rate_limit(self.cmd_l, ql, dt)
        self.cmd_r = self.rate_limit(self.cmd_r, qr, dt)

        msg = Float64MultiArray()
        msg.data = [float(x) for x in self.cmd_l + self.cmd_r]
        self.leg_pub.publish(msg)
        self.publish_arms(dt)

        self.phase_t += dt
        self.update_state()


def main(args=None):
    rclpy.init(args=args)
    node = AstiSimpleWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
