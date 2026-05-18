from .common import Pose2D, WalkerParams, lerp, smoothstep


class SwingFootPlanner:
    def __init__(self, params: WalkerParams):
        self.params = params

    def reset(self, params: WalkerParams) -> None:
        self.params = params

    def pose(self, start: Pose2D, target: Pose2D, phase: float) -> Pose2D:
        phase = max(0.0, min(1.0, phase))
        u = smoothstep(phase)
        lift_end = max(0.10, min(0.45, self.params.swing_lift_fraction))
        lower_start = max(lift_end + 0.10, 1.0 - self.params.swing_lower_fraction)
        if phase < lift_end:
            z_scale = smoothstep(phase / lift_end)
        elif phase > lower_start:
            z_scale = 1.0 - smoothstep((phase - lower_start) / max(0.05, 1.0 - lower_start))
        else:
            z_scale = 1.0
        z = lerp(start.z, target.z, u) + z_scale * self.params.foot_clearance
        return Pose2D(
            lerp(start.x, target.x, u),
            lerp(start.y, target.y, u),
            z,
            0.0,
            0.0,
            lerp(start.yaw, target.yaw, u),
        )
