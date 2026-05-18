import math

from .common import D, LEG_DOF, Pose2D, WalkerParams, clamp, wrap_pi


class LegIK:
    def __init__(self, params: WalkerParams):
        self.params = params
        self.thigh_length = 0.24
        self.shin_length = 0.24
        self.hip_width = 0.10
        self.limits = [
            (-30.0 * D, 30.0 * D),
            (-22.0 * D, 22.0 * D),
            (-38.0 * D, 30.0 * D),
            (0.0, 62.0 * D),
            (-22.0 * D, 22.0 * D),
            (-38.0 * D, 28.0 * D),
        ]

    def reset(self, params: WalkerParams) -> None:
        self.params = params

    def solve(self, pelvis: Pose2D, left_foot: Pose2D, right_foot: Pose2D) -> tuple:
        return self._solve_leg(pelvis, left_foot, "left"), self._solve_leg(pelvis, right_foot, "right")

    def _solve_leg(self, pelvis: Pose2D, foot: Pose2D, side: str) -> list:
        side_sign = 1.0 if side == "left" else -1.0
        hip_x = pelvis.x
        hip_y = pelvis.y + side_sign * self.hip_width / 2.0
        hip_z = pelvis.z

        dx = foot.x - hip_x
        dy = foot.y - hip_y
        dz = hip_z - foot.z
        sagittal = math.hypot(dx, dz)
        leg_len = clamp(math.hypot(sagittal, dy), 0.12, self.thigh_length + self.shin_length - 0.01)

        knee_cos = clamp(
            (self.thigh_length**2 + self.shin_length**2 - leg_len**2)
            / (2.0 * self.thigh_length * self.shin_length),
            -1.0,
            1.0,
        )
        knee_pitch = math.pi - math.acos(knee_cos)

        reach_cos = clamp(
            (self.thigh_length**2 + leg_len**2 - self.shin_length**2)
            / (2.0 * self.thigh_length * leg_len),
            -1.0,
            1.0,
        )
        reach = math.acos(reach_cos)
        line_angle = math.atan2(dx, max(0.04, dz))
        hip_pitch = line_angle - reach
        hip_roll = clamp(math.atan2(dy, max(0.08, dz)), -18.0 * D, 18.0 * D)
        hip_yaw = wrap_pi(foot.yaw - pelvis.yaw)

        ankle_pitch = foot.pitch - pelvis.pitch - hip_pitch - knee_pitch
        ankle_roll = foot.roll - pelvis.roll - hip_roll
        q = [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_roll, ankle_pitch]
        return [clamp(q[i], self.limits[i][0], self.limits[i][1]) for i in range(LEG_DOF)]

    def limit(self, q: list) -> list:
        return [clamp(q[i], self.limits[i][0], self.limits[i][1]) for i in range(LEG_DOF)]

