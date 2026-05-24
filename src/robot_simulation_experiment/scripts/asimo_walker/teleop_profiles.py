from dataclasses import dataclass
from typing import Optional

from .common import D, WalkerParams


@dataclass(frozen=True)
class MotionProfile:
    name: str
    enabled: bool
    description: str
    step_length: Optional[float] = None
    step_width: Optional[float] = None
    step_time: Optional[float] = None
    double_support_time: Optional[float] = None
    transfer_time: Optional[float] = None
    touchdown_time: Optional[float] = None
    foot_clearance: Optional[float] = None
    swing_lift_fraction: Optional[float] = None
    swing_lower_fraction: Optional[float] = None
    direction_sign: Optional[float] = None
    turn_yaw_per_step: Optional[float] = None
    waist_yaw_target: Optional[float] = None
    waist_yaw_rate: Optional[float] = None
    arm_swing_gain: Optional[float] = None
    arm_balance_gain: Optional[float] = None
    zmp_kp: Optional[float] = None
    zmp_kd: Optional[float] = None
    max_joint_rate: Optional[float] = None
    max_arm_rate: Optional[float] = None
    max_com_speed: Optional[float] = None
    max_com_accel: Optional[float] = None
    total_steps: Optional[int] = None
    allow_continuous_steps: bool = False


def _forward_profile() -> MotionProfile:
    params = WalkerParams()
    return MotionProfile(
        name="forward_walk",
        enabled=True,
        description="Verified baseline forward walk",
        step_length=params.step_length,
        step_width=params.step_width,
        step_time=params.step_time,
        double_support_time=params.double_support_time,
        transfer_time=params.transfer_time,
        touchdown_time=params.touchdown_time,
        foot_clearance=params.foot_clearance,
        swing_lift_fraction=params.swing_lift_fraction,
        swing_lower_fraction=params.swing_lower_fraction,
        direction_sign=1.0,
        turn_yaw_per_step=0.0,
        waist_yaw_target=0.0,
        zmp_kp=params.zmp_kp,
        zmp_kd=params.zmp_kd,
        max_joint_rate=params.max_joint_rate,
        max_arm_rate=params.max_arm_rate,
        max_com_speed=params.max_com_speed,
        max_com_accel=params.max_com_accel,
        total_steps=params.total_steps,
        allow_continuous_steps=True,
    )


def _backward_profile() -> MotionProfile:
    return MotionProfile(
        name="backward_walk",
        enabled=True,
        description="Conservative backward walk",
        step_length=0.032,
        step_width=0.095,
        step_time=1.95,
        double_support_time=0.68,
        transfer_time=0.96,
        touchdown_time=0.36,
        foot_clearance=0.050,
        swing_lift_fraction=0.32,
        swing_lower_fraction=0.38,
        direction_sign=-1.0,
        turn_yaw_per_step=0.0,
        waist_yaw_target=0.0,
        zmp_kp=1.45,
        zmp_kd=1.25,
        max_joint_rate=1.20,
        max_arm_rate=1.0,
        max_com_speed=0.055,
        max_com_accel=0.13,
        total_steps=6,
        allow_continuous_steps=True,
    )


def _turn_left_profile() -> MotionProfile:
    return MotionProfile(
        name="turn_left",
        enabled=True,
        description="Validated faster left turn",
        step_length=0.0,
        step_width=0.106,
        step_time=1.84,
        double_support_time=0.64,
        transfer_time=0.90,
        touchdown_time=0.36,
        foot_clearance=0.055,
        swing_lift_fraction=0.31,
        swing_lower_fraction=0.35,
        direction_sign=1.0,
        turn_yaw_per_step=5.2 * D,
        waist_yaw_target=0.0,
        zmp_kp=1.35,
        zmp_kd=1.42,
        max_joint_rate=1.08,
        max_arm_rate=0.90,
        max_com_speed=0.050,
        max_com_accel=0.115,
        total_steps=10,
        allow_continuous_steps=False,
    )


def _turn_right_profile() -> MotionProfile:
    left = _turn_left_profile()
    return MotionProfile(
        name="turn_right",
        enabled=True,
        description="Validated faster right turn",
        step_length=left.step_length,
        step_width=left.step_width,
        step_time=left.step_time,
        double_support_time=left.double_support_time,
        transfer_time=left.transfer_time,
        touchdown_time=left.touchdown_time,
        foot_clearance=left.foot_clearance,
        swing_lift_fraction=left.swing_lift_fraction,
        swing_lower_fraction=left.swing_lower_fraction,
        direction_sign=left.direction_sign,
        turn_yaw_per_step=-left.turn_yaw_per_step,
        waist_yaw_target=left.waist_yaw_target,
        zmp_kp=left.zmp_kp,
        zmp_kd=left.zmp_kd,
        max_joint_rate=left.max_joint_rate,
        max_arm_rate=left.max_arm_rate,
        max_com_speed=left.max_com_speed,
        max_com_accel=left.max_com_accel,
        total_steps=left.total_steps,
        allow_continuous_steps=False,
    )


def _waist_left_profile() -> MotionProfile:
    return MotionProfile(
        name="waist_left",
        enabled=True,
        description="Standing waist twist left",
        waist_yaw_target=10.0 * D,
        waist_yaw_rate=20.0 * D,
        max_joint_rate=0.85,
        max_arm_rate=0.8,
    )


def _waist_right_profile() -> MotionProfile:
    left = _waist_left_profile()
    return MotionProfile(
        name="waist_right",
        enabled=True,
        description="Standing waist twist right",
        waist_yaw_target=-left.waist_yaw_target,
        waist_yaw_rate=left.waist_yaw_rate,
        max_joint_rate=left.max_joint_rate,
        max_arm_rate=left.max_arm_rate,
    )


def _forward_fast_profile() -> MotionProfile:
    return MotionProfile(
        name="fast_modifier",
        enabled=True,
        description="Validated faster forward walk",
        step_length=0.048,
        step_width=0.09,
        step_time=1.62,
        double_support_time=0.50,
        transfer_time=0.78,
        touchdown_time=0.28,
        foot_clearance=0.048,
        swing_lift_fraction=0.25,
        swing_lower_fraction=0.28,
        direction_sign=1.0,
        turn_yaw_per_step=0.0,
        waist_yaw_target=0.0,
        zmp_kp=1.65,
        zmp_kd=1.08,
        max_joint_rate=1.52,
        max_arm_rate=1.22,
        max_com_speed=0.082,
        max_com_accel=0.22,
        total_steps=6,
        allow_continuous_steps=True,
    )


idle_profile = MotionProfile(
    name="idle",
    enabled=True,
    description="Idle / safe hold",
)
forward_walk_profile = _forward_profile()
backward_walk_profile = _backward_profile()
turn_left_profile = _turn_left_profile()
turn_right_profile = _turn_right_profile()
waist_left_profile = _waist_left_profile()
waist_right_profile = _waist_right_profile()
fast_modifier_profile = _forward_fast_profile()

_PROFILES = {
    profile.name: profile
    for profile in (
        idle_profile,
        forward_walk_profile,
        backward_walk_profile,
        turn_left_profile,
        turn_right_profile,
        waist_left_profile,
        waist_right_profile,
        fast_modifier_profile,
    )
}

_ENABLED_KEY_PROFILES = (
    ("key_s", "backward_walk"),
    ("key_a", "turn_left"),
    ("key_d", "turn_right"),
    ("key_q", "waist_left"),
    ("key_e", "waist_right"),
)

_DISABLED_KEY_PROFILES = (
)

_WALKER_PARAM_FIELDS = (
    "step_length",
    "step_width",
    "step_time",
    "double_support_time",
    "transfer_time",
    "touchdown_time",
    "foot_clearance",
    "swing_lift_fraction",
    "swing_lower_fraction",
    "zmp_kp",
    "zmp_kd",
    "max_joint_rate",
    "max_arm_rate",
    "max_com_speed",
    "max_com_accel",
    "total_steps",
    "turn_yaw_per_step",
)


def get_profile(name: str) -> MotionProfile:
    return _PROFILES.get(name, idle_profile)


def walker_params_for_profile(profile: MotionProfile) -> WalkerParams:
    params = WalkerParams()
    for field in _WALKER_PARAM_FIELDS:
        value = getattr(profile, field)
        if value is not None:
            setattr(params, field, value)
    if profile.direction_sign is not None:
        params.sagittal_sign *= profile.direction_sign
    return params


def requested_profile_name(input_state) -> str:
    if input_state.emergency_stop:
        return "emergency_stop"
    if input_state.pause:
        return "pause"
    if input_state.shift and not input_state.key_w:
        return "fast_modifier"
    if input_state.key_w:
        extras = [name for attr, name in _ENABLED_KEY_PROFILES if getattr(input_state, attr)]
        extras.extend(name for attr, name in _DISABLED_KEY_PROFILES if getattr(input_state, attr))
        if input_state.shift:
            return "fast_modifier"
        if extras:
            return "forward_walk+" + "+".join(extras)
        return "forward_walk"
    requested = [name for attr, name in _ENABLED_KEY_PROFILES if getattr(input_state, attr)]
    requested.extend(name for attr, name in _DISABLED_KEY_PROFILES if getattr(input_state, attr))
    if input_state.shift:
        requested.append("fast_modifier")
    return "+".join(requested) if requested else "idle"


def resolve_profile_from_input(input_state) -> tuple[MotionProfile, str]:
    if input_state.emergency_stop:
        return idle_profile, "Emergency stop requested; holding current command."
    if input_state.pause:
        return idle_profile, "Paused; walker will hold or finish the current step safely."

    requested_enabled = [name for attr, name in _ENABLED_KEY_PROFILES if getattr(input_state, attr)]
    requested_disabled = [name for attr, name in _DISABLED_KEY_PROFILES if getattr(input_state, attr)]
    if input_state.shift:
        if requested_enabled:
            enabled_text = ", ".join(f"'{name}'" for name in requested_enabled)
            return (
                fast_modifier_profile,
                f"Requested profile(s) {enabled_text} conflict with fast forward; using validated faster forward walk.",
            )
        if input_state.key_w:
            return fast_modifier_profile, "Validated faster forward walk active."
        return fast_modifier_profile, "Validated faster forward walk active."

    if input_state.key_w:
        if requested_enabled:
            enabled_text = ", ".join(f"'{name}'" for name in requested_enabled)
            return (
                forward_walk_profile,
                f"Requested profile(s) {enabled_text} conflict with forward; using forward baseline only.",
            )
        return forward_walk_profile, "Forward baseline profile active."

    if requested_enabled:
        if len(requested_enabled) == 1 and not requested_disabled:
            profile = get_profile(requested_enabled[0])
            return profile, f"{profile.description} profile active."
        enabled_text = ", ".join(f"'{name}'" for name in requested_enabled)
        if requested_disabled:
            disabled_text = ", ".join(f"'{name}'" for name in requested_disabled)
            return (
                get_profile(requested_enabled[0]),
                f"Requested profile(s) {disabled_text} are reserved but disabled; using enabled {enabled_text}.",
            )
        return get_profile(requested_enabled[0]), f"Using enabled profile {enabled_text}."

    if requested_disabled:
        if len(requested_disabled) == 1:
            return idle_profile, f"Requested profile '{requested_disabled[0]}' is reserved but disabled / 未调参，暂不执行."
        disabled_text = ", ".join(f"'{name}'" for name in requested_disabled)
        return idle_profile, f"Requested profiles {disabled_text} are reserved but disabled / 未调参，暂不执行."

    return idle_profile, "Idle / safe hold."
