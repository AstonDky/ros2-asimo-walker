from .common import Footstep, Pose2D, WalkerParams, lerp, smoothstep


class ZMPReferencePlanner:
    def __init__(self, params: WalkerParams):
        self.params = params

    def reset(self, params: WalkerParams) -> None:
        self.params = params

    def zmp_for_phase(
        self,
        step: Footstep,
        phase: float,
        left_current: Pose2D,
        right_current: Pose2D,
        double_support: bool,
    ) -> tuple:
        support = step.support_pose()
        if not double_support:
            return support.x, support.y

        u = smoothstep(phase)
        if step.support == "right":
            old_support = right_current
            new_support = step.left_target
        else:
            old_support = left_current
            new_support = step.right_target
        return lerp(old_support.x, new_support.x, u), lerp(old_support.y, new_support.y, u)

    def zmp_for_state(
        self,
        state_name: str,
        step: Footstep,
        phase: float,
        left_current: Pose2D,
        right_current: Pose2D,
    ) -> tuple:
        u = smoothstep(phase)
        center_x = 0.5 * (left_current.x + right_current.x)
        center_y = 0.5 * (left_current.y + right_current.y)
        left_support_y = left_current.y + self.params.support_zmp_margin
        right_support_y = right_current.y - self.params.support_zmp_margin
        left_landing_center_x = 0.5 * (step.left_target.x + right_current.x)
        left_landing_center_y = 0.5 * (step.left_target.y + right_current.y)
        right_landing_center_x = 0.5 * (left_current.x + step.right_target.x)
        right_landing_center_y = 0.5 * (left_current.y + step.right_target.y)

        if state_name in ("CROUCH", "STAND", "DONE", "WAIT"):
            return center_x, center_y
        if state_name == "TRANSFER_TO_RIGHT":
            return lerp(center_x, right_current.x, u), lerp(center_y, right_support_y, u)
        if state_name == "TRANSFER_TO_LEFT":
            return lerp(center_x, left_current.x, u), lerp(center_y, left_support_y, u)
        if state_name == "LEFT_SWING":
            return right_current.x, right_support_y
        if state_name == "RIGHT_SWING":
            return left_current.x, left_support_y
        if state_name == "LEFT_TOUCHDOWN":
            return lerp(right_current.x, left_landing_center_x, u), lerp(right_support_y, left_landing_center_y, u)
        if state_name == "RIGHT_TOUCHDOWN":
            return lerp(left_current.x, right_landing_center_x, u), lerp(left_support_y, right_landing_center_y, u)
        if state_name == "DOUBLE_SUPPORT_AFTER_LEFT":
            return left_landing_center_x, left_landing_center_y
        if state_name == "DOUBLE_SUPPORT_AFTER_RIGHT":
            return right_landing_center_x, right_landing_center_y
        return self.zmp_for_phase(step, phase, left_current, right_current, True)

    def preview_zmp(self, current: Footstep, next_step: Footstep = None) -> tuple:
        if next_step is None:
            pose = current.support_pose()
            return pose.x, pose.y
        pose = next_step.support_pose()
        return pose.x, pose.y
