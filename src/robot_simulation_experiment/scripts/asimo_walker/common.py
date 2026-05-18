import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple


D = math.pi / 180.0
G = 9.81
LEG_DOF = 6


def deg(x: float) -> float:
    return x * D


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def smoothstep(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def lerp(a: float, b: float, u: float) -> float:
    return a + (b - a) * u


def wrap_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def vec_lerp(a: "Pose2D", b: "Pose2D", u: float) -> "Pose2D":
    return Pose2D(
        lerp(a.x, b.x, u),
        lerp(a.y, b.y, u),
        lerp(a.z, b.z, u),
        lerp(a.roll, b.roll, u),
        lerp(a.pitch, b.pitch, u),
        lerp(a.yaw, b.yaw, u),
    )


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def copy(self) -> "Pose2D":
        return Pose2D(self.x, self.y, self.z, self.roll, self.pitch, self.yaw)


@dataclass
class Footstep:
    support: str
    swing: str
    left_target: Pose2D
    right_target: Pose2D
    t_start: float
    t_end: float

    def support_pose(self) -> Pose2D:
        return self.left_target if self.support == "left" else self.right_target

    def swing_pose(self) -> Pose2D:
        return self.left_target if self.swing == "left" else self.right_target


@dataclass
class RobotFeedback:
    ori: Optional[Sequence[float]] = None
    gyro: Optional[Sequence[float]] = None
    pos: Optional[Sequence[float]] = None
    left_leg: Optional[Sequence[float]] = None
    right_leg: Optional[Sequence[float]] = None
    left_force: Optional[float] = None
    right_force: Optional[float] = None
    left_cop: Optional[Tuple[float, float]] = None
    right_cop: Optional[Tuple[float, float]] = None


@dataclass
class StabilizerOutput:
    left_add: list = field(default_factory=lambda: [0.0] * LEG_DOF)
    right_add: list = field(default_factory=lambda: [0.0] * LEG_DOF)
    left_arm_add: list = field(default_factory=list)
    right_arm_add: list = field(default_factory=list)
    next_step_dx: float = 0.0
    next_step_dy: float = 0.0


@dataclass
class WalkerParams:
    step_length: float = 0.045
    step_width: float = 0.09
    step_time: float = 1.62
    double_support_time: float = 0.48
    foot_clearance: float = 0.052
    pelvis_height: float = 0.48
    total_steps: int = 6
    sagittal_sign: float = -1.0
    support_zmp_margin: float = 0.004

    zmp_preview_time: float = 0.8
    zmp_kp: float = 1.6
    zmp_kd: float = 1.0

    ankle_pitch_kp: float = 0.35
    ankle_roll_kp: float = 0.35
    hip_pitch_kp: float = 0.18
    hip_roll_kp: float = 0.18

    crouch_time: float = 1.5
    transfer_time: float = 0.74
    touchdown_time: float = 0.25
    stand_time: float = 2.0
    swing_lift_fraction: float = 0.24
    swing_lower_fraction: float = 0.26
    dt: float = 0.02

    max_joint_rate: float = 1.55
    max_arm_rate: float = 1.2
    max_com_speed: float = 0.075
    max_com_accel: float = 0.20

    stable_pitch: float = 5.0 * D
    stable_roll: float = 6.0 * D
    abort_tilt: float = 18.0 * D

    @classmethod
    def from_profile(cls, profile: dict) -> "WalkerParams":
        params = cls()
        for key, value in profile.items():
            if hasattr(params, key):
                setattr(params, key, value)
        return params
