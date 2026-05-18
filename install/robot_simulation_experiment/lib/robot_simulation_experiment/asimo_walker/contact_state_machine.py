from enum import Enum, auto

from .common import WalkerParams


class WalkState(Enum):
    WAIT = auto()
    CROUCH = auto()
    TRANSFER_TO_RIGHT = auto()
    LEFT_SWING = auto()
    LEFT_TOUCHDOWN = auto()
    DOUBLE_SUPPORT_AFTER_LEFT = auto()
    TRANSFER_TO_LEFT = auto()
    RIGHT_SWING = auto()
    RIGHT_TOUCHDOWN = auto()
    DOUBLE_SUPPORT_AFTER_RIGHT = auto()
    STAND = auto()
    DONE = auto()
    ABORT = auto()


class ContactAndStateMachine:
    def __init__(self, params: WalkerParams):
        self.params = params
        self.state = WalkState.WAIT
        self.state_t = 0.0
        self.step_index = 0
        self._last_state = self.state

    def reset(self, params: WalkerParams = None) -> None:
        if params is not None:
            self.params = params
        self.state = WalkState.WAIT
        self.state_t = 0.0
        self.step_index = 0
        self._last_state = self.state

    def start(self) -> None:
        if self.state == WalkState.WAIT:
            self._set(WalkState.CROUCH)

    def update(self, dt: float, feedback, pitch: float, roll: float) -> WalkState:
        p = self.params
        if abs(pitch) > p.abort_tilt or abs(roll) > p.abort_tilt:
            self._set(WalkState.ABORT)
            return self.state

        self.state_t += dt
        stable = self._stable(feedback, pitch, roll)

        if self.state == WalkState.WAIT:
            return self.state
        if self.state == WalkState.CROUCH and self.state_t >= p.crouch_time and stable:
            self._set(WalkState.TRANSFER_TO_RIGHT)
        elif self.state == WalkState.TRANSFER_TO_RIGHT and self.state_t >= p.transfer_time and self._support_loaded(feedback, "right", stable):
            self._set(WalkState.LEFT_SWING)
        elif self.state == WalkState.LEFT_SWING and self.state_t >= p.step_time:
            self._set(WalkState.LEFT_TOUCHDOWN)
        elif self.state == WalkState.LEFT_TOUCHDOWN and self.state_t >= p.touchdown_time and self._swing_contact(feedback, "left", stable):
            self._set(WalkState.DOUBLE_SUPPORT_AFTER_LEFT)
        elif self.state == WalkState.DOUBLE_SUPPORT_AFTER_LEFT and self.state_t >= p.double_support_time and stable:
            self.step_index += 1
            if self.step_index >= p.total_steps:
                self._set(WalkState.STAND)
            else:
                self._set(WalkState.TRANSFER_TO_LEFT)
        elif self.state == WalkState.TRANSFER_TO_LEFT and self.state_t >= p.transfer_time and self._support_loaded(feedback, "left", stable):
            self._set(WalkState.RIGHT_SWING)
        elif self.state == WalkState.RIGHT_SWING and self.state_t >= p.step_time:
            self._set(WalkState.RIGHT_TOUCHDOWN)
        elif self.state == WalkState.RIGHT_TOUCHDOWN and self.state_t >= p.touchdown_time and self._swing_contact(feedback, "right", stable):
            self._set(WalkState.DOUBLE_SUPPORT_AFTER_RIGHT)
        elif self.state == WalkState.DOUBLE_SUPPORT_AFTER_RIGHT and self.state_t >= p.double_support_time and stable:
            self.step_index += 1
            if self.step_index >= p.total_steps:
                self._set(WalkState.STAND)
            else:
                self._set(WalkState.TRANSFER_TO_RIGHT)
        elif self.state == WalkState.STAND and self.state_t >= p.stand_time and stable:
            self._set(WalkState.DONE)
        return self.state

    def support_foot(self) -> str:
        if self.state in (WalkState.CROUCH, WalkState.STAND, WalkState.DONE):
            return "both"
        if self.state in (WalkState.TRANSFER_TO_LEFT, WalkState.RIGHT_SWING, WalkState.RIGHT_TOUCHDOWN):
            return "left"
        return "right"

    def swing_foot(self) -> str:
        if self.state in (WalkState.LEFT_SWING, WalkState.LEFT_TOUCHDOWN):
            return "left"
        if self.state in (WalkState.RIGHT_SWING, WalkState.RIGHT_TOUCHDOWN):
            return "right"
        return "none"

    def swing_phase(self) -> float:
        if self.state in (WalkState.LEFT_SWING, WalkState.RIGHT_SWING):
            return min(1.0, self.state_t / max(0.1, self.params.step_time))
        if self.state in (WalkState.LEFT_TOUCHDOWN, WalkState.RIGHT_TOUCHDOWN):
            return 1.0
        return 0.0

    def force_ratio(self, feedback) -> tuple:
        lf = feedback.left_force
        rf = feedback.right_force
        if lf is None or rf is None or lf + rf <= 1e-6:
            return None, None
        total = lf + rf
        return lf / total, rf / total

    def double_support_phase(self) -> float:
        if self.state in (WalkState.DOUBLE_SUPPORT_AFTER_LEFT, WalkState.DOUBLE_SUPPORT_AFTER_RIGHT):
            return min(1.0, self.state_t / max(0.1, self.params.double_support_time))
        if self.state in (WalkState.TRANSFER_TO_LEFT, WalkState.TRANSFER_TO_RIGHT):
            return min(1.0, self.state_t / max(0.1, self.params.transfer_time))
        return 0.0

    def _set(self, state: WalkState) -> None:
        if state != self.state:
            self.state = state
            self.state_t = 0.0

    def _stable(self, feedback, pitch: float, roll: float) -> bool:
        gyro = feedback.gyro or [0.0, 0.0, 0.0]
        gyro_ok = max(abs(v) for v in gyro[:2]) < 1.4
        return abs(pitch) < self.params.stable_pitch and abs(roll) < self.params.stable_roll and gyro_ok

    def _support_loaded(self, feedback, support: str, fallback: bool) -> bool:
        left_ratio, right_ratio = self.force_ratio(feedback)
        if left_ratio is None:
            return fallback
        return left_ratio > 0.56 if support == "left" else right_ratio > 0.56

    def _swing_contact(self, feedback, foot: str, fallback: bool) -> bool:
        left_ratio, right_ratio = self.force_ratio(feedback)
        if left_ratio is None:
            return fallback or self.state_t >= self.params.touchdown_time
        return left_ratio > 0.20 if foot == "left" else right_ratio > 0.20
