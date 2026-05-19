from dataclasses import dataclass
from typing import Optional

from .common import WalkerParams


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
    foot_clearance: Optional[float] = None
    direction_sign: Optional[float] = None
    turn_yaw_per_step: Optional[float] = None
    waist_yaw_target: Optional[float] = None
    waist_yaw_rate: Optional[float] = None
    arm_swing_gain: Optional[float] = None
    arm_balance_gain: Optional[float] = None
    max_joint_rate: Optional[float] = None
    max_arm_rate: Optional[float] = None
    max_com_speed: Optional[float] = None
    max_com_accel: Optional[float] = None
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
        foot_clearance=params.foot_clearance,
        direction_sign=1.0,
        turn_yaw_per_step=0.0,
        waist_yaw_target=0.0,
        max_joint_rate=params.max_joint_rate,
        max_arm_rate=params.max_arm_rate,
        max_com_speed=params.max_com_speed,
        max_com_accel=params.max_com_accel,
        allow_continuous_steps=True,
    )


idle_profile = MotionProfile(
    name="idle",
    enabled=True,
    description="Idle / safe hold",
)
forward_walk_profile = _forward_profile()
backward_walk_profile = MotionProfile(
    name="backward_walk",
    enabled=False,
    description="Reserved for backward walking; not tuned yet",
)
turn_left_profile = MotionProfile(
    name="turn_left",
    enabled=False,
    description="Reserved for left turning; not tuned yet",
)
turn_right_profile = MotionProfile(
    name="turn_right",
    enabled=False,
    description="Reserved for right turning; not tuned yet",
)
waist_left_profile = MotionProfile(
    name="waist_left",
    enabled=False,
    description="Reserved for left waist twist; not tuned yet",
)
waist_right_profile = MotionProfile(
    name="waist_right",
    enabled=False,
    description="Reserved for right waist twist; not tuned yet",
)
fast_modifier_profile = MotionProfile(
    name="fast_modifier",
    enabled=False,
    description="Reserved for faster walking; not tuned yet",
)

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

_DISABLED_KEY_PROFILES = (
    ("key_s", "backward_walk"),
    ("key_a", "turn_left"),
    ("key_d", "turn_right"),
    ("key_q", "waist_left"),
    ("key_e", "waist_right"),
)


def get_profile(name: str) -> MotionProfile:
    return _PROFILES.get(name, idle_profile)


def requested_profile_name(input_state) -> str:
    if input_state.emergency_stop:
        return "emergency_stop"
    if input_state.pause:
        return "pause"
    if input_state.key_w:
        extras = [name for attr, name in _DISABLED_KEY_PROFILES if getattr(input_state, attr)]
        if input_state.shift:
            extras.append("fast_modifier")
        if extras:
            return "forward_walk+" + "+".join(extras)
        return "forward_walk"
    requested = [name for attr, name in _DISABLED_KEY_PROFILES if getattr(input_state, attr)]
    if input_state.shift:
        requested.append("fast_modifier")
    return "+".join(requested) if requested else "idle"


def resolve_profile_from_input(input_state) -> tuple[MotionProfile, str]:
    if input_state.emergency_stop:
        return idle_profile, "Emergency stop requested; holding current command."
    if input_state.pause:
        return idle_profile, "Paused; walker will hold or finish the current step safely."

    requested_disabled = [name for attr, name in _DISABLED_KEY_PROFILES if getattr(input_state, attr)]
    if input_state.shift:
        requested_disabled.append("fast_modifier")

    if input_state.key_w:
        if requested_disabled:
            disabled_text = ", ".join(f"'{name}'" for name in requested_disabled)
            return (
                forward_walk_profile,
                f"Requested profile(s) {disabled_text} are reserved but disabled; using forward baseline only.",
            )
        return forward_walk_profile, "Forward baseline profile active."

    if requested_disabled:
        if len(requested_disabled) == 1:
            return idle_profile, f"Requested profile '{requested_disabled[0]}' is reserved but disabled / 未调参，暂不执行."
        disabled_text = ", ".join(f"'{name}'" for name in requested_disabled)
        return idle_profile, f"Requested profiles {disabled_text} are reserved but disabled / 未调参，暂不执行."

    return idle_profile, "Idle / safe hold."
