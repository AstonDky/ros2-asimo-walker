from .common import D, LEG_DOF, StabilizerOutput, WalkerParams, clamp


class Stabilizer:
    def __init__(self, params: WalkerParams):
        self.params = params

    def reset(self, params: WalkerParams) -> None:
        self.params = params

    def compute(
        self,
        pitch: float,
        roll: float,
        gyro: list,
        support_foot: str,
        com_vx: float,
        com_vy: float,
        left_arm_count: int = 0,
        right_arm_count: int = 0,
    ) -> StabilizerOutput:
        p = self.params
        gyro_pitch = gyro[0] if len(gyro) > 0 else 0.0
        gyro_roll = gyro[1] if len(gyro) > 1 else 0.0
        pitch_fb = clamp(pitch + 0.09 * gyro_pitch, -10.0 * D, 10.0 * D)
        roll_fb = clamp(roll + 0.09 * gyro_roll, -10.0 * D, 10.0 * D)

        out = StabilizerOutput(
            left_arm_add=[0.0] * left_arm_count,
            right_arm_add=[0.0] * right_arm_count,
        )

        support_gain = 1.0
        swing_gain = 0.25
        if support_foot == "both":
            left_gain = 0.55
            right_gain = 0.55
        else:
            left_gain = support_gain if support_foot == "left" else swing_gain
            right_gain = support_gain if support_foot == "right" else swing_gain

        ankle_pitch = clamp(-p.ankle_pitch_kp * pitch_fb, -5.5 * D, 5.5 * D)
        ankle_roll = clamp(-p.ankle_roll_kp * roll_fb, -5.0 * D, 5.0 * D)
        hip_pitch = clamp(-p.hip_pitch_kp * pitch_fb, -3.5 * D, 3.5 * D)
        hip_roll = clamp(-p.hip_roll_kp * roll_fb, -3.5 * D, 3.5 * D)

        for add, gain in ((out.left_add, left_gain), (out.right_add, right_gain)):
            add[2] += gain * hip_pitch
            add[3] += gain * 0.35 * abs(hip_pitch)
            add[4] += gain * ankle_roll
            add[5] += gain * ankle_pitch
            add[1] += gain * hip_roll

        arm = clamp(0.55 * pitch_fb, -8.0 * D, 8.0 * D)
        side_arm = clamp(0.45 * roll_fb, -6.0 * D, 6.0 * D)
        if out.left_arm_add:
            out.left_arm_add[0] += -arm
            if len(out.left_arm_add) > 1:
                out.left_arm_add[1] += side_arm
        if out.right_arm_add:
            out.right_arm_add[0] += arm
            if len(out.right_arm_add) > 1:
                out.right_arm_add[1] += side_arm

        out.next_step_dx = clamp(0.08 * pitch + 0.12 * com_vx, -0.018, 0.018)
        out.next_step_dy = clamp(0.06 * roll + 0.10 * com_vy, -0.014, 0.014)
        return out
