from .common import Footstep, Pose2D, WalkerParams, clamp


class FootstepPlanner:
    def __init__(self, params: WalkerParams):
        self.params = params
        self.steps = []
        self.reset(params)

    def reset(self, params: WalkerParams = None) -> None:
        if params is not None:
            self.params = params
        p = self.params
        self.steps = []
        self.ensure_steps(p.total_steps)

    def ensure_steps(self, count: int) -> None:
        while len(self.steps) < count:
            self._append_next_step()

    def _append_next_step(self) -> None:
        p = self.params
        index = len(self.steps)
        if self.steps:
            left = self.steps[-1].left_target.copy()
            right = self.steps[-1].right_target.copy()
            t = self.steps[-1].t_end + p.double_support_time
        else:
            left = self.initial_left()
            right = self.initial_right()
            t = 0.0

        if index % 2 == 0:
            support = "right"
            swing = "left"
            left = Pose2D(
                right.x + p.sagittal_sign * p.step_length,
                p.step_width / 2.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        else:
            support = "left"
            swing = "right"
            right = Pose2D(
                left.x + p.sagittal_sign * p.step_length,
                -p.step_width / 2.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

        self.steps.append(
            Footstep(
                support=support,
                swing=swing,
                left_target=left.copy(),
                right_target=right.copy(),
                t_start=t,
                t_end=t + p.step_time,
            )
        )

    def get_step(self, index: int) -> Footstep:
        if not self.steps:
            self.reset()
        index = max(0, int(index))
        self.ensure_steps(index + 1)
        return self.steps[index]

    def initial_left(self) -> Pose2D:
        return Pose2D(0.0, self.params.step_width / 2.0, 0.0, 0.0, 0.0, 0.0)

    def initial_right(self) -> Pose2D:
        return Pose2D(0.0, -self.params.step_width / 2.0, 0.0, 0.0, 0.0, 0.0)

    def modify_next_step(self, index: int, dx: float, dy: float) -> None:
        if index >= len(self.steps):
            return
        dx = clamp(dx, -0.018, 0.018)
        dy = clamp(dy, -0.014, 0.014)

        step = self.steps[index]
        target = step.left_target if step.swing == "left" else step.right_target
        target.x += dx
        if step.swing == "left":
            target.y = clamp(target.y + dy, 0.025, 0.085)
        else:
            target.y = clamp(target.y + dy, -0.085, -0.025)

        # Keep later nominal steps coherent with the corrected foothold.
        for later in self.steps[index + 1 :]:
            later.left_target.x += dx
            later.right_target.x += dx
