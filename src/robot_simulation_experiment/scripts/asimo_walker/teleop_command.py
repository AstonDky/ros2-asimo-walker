import threading
from dataclasses import dataclass


@dataclass
class TeleopInputState:
    key_w: bool = False
    key_s: bool = False
    key_a: bool = False
    key_d: bool = False
    key_q: bool = False
    key_e: bool = False
    shift: bool = False
    pause: bool = False
    emergency_stop: bool = False


class TeleopCommandBuffer:
    """Thread-safe handoff from the tkinter GUI thread to the ROS loop."""

    def __init__(self):
        self._lock = threading.Lock()
        self._state = TeleopInputState()

    def set_key(self, key: str, pressed: bool) -> None:
        key = key.lower()
        attr = {
            "w": "key_w",
            "s": "key_s",
            "a": "key_a",
            "d": "key_d",
            "q": "key_q",
            "e": "key_e",
            "shift": "shift",
        }.get(key)
        if attr is None:
            return
        with self._lock:
            setattr(self._state, attr, bool(pressed))

    def press_key(self, key: str) -> None:
        """Latch one teleop request; pressing the active key again releases it."""
        key = key.lower()
        with self._lock:
            if key == "w":
                if self._state.key_w:
                    self._clear_motion_locked()
                else:
                    self._clear_motion_locked()
                    self._state.key_w = True
                return

            attr = {
                "s": "key_s",
                "a": "key_a",
                "d": "key_d",
                "q": "key_q",
                "e": "key_e",
            }.get(key)
            if attr is not None:
                if getattr(self._state, attr):
                    self._clear_motion_locked()
                    return
                self._clear_motion_locked()
                setattr(self._state, attr, True)
                return

            if key == "shift":
                self._state.shift = not self._state.shift

    def _clear_motion_locked(self) -> None:
        self._state.key_w = False
        self._state.key_s = False
        self._state.key_a = False
        self._state.key_d = False
        self._state.key_q = False
        self._state.key_e = False
        self._state.shift = False

    def toggle_pause(self) -> None:
        with self._lock:
            self._state.pause = not self._state.pause

    def reset_keys(self) -> None:
        with self._lock:
            self._state = TeleopInputState()

    def set_emergency_stop(self) -> None:
        with self._lock:
            self._state.emergency_stop = True
            self._state.pause = False
            self._state.key_w = False
            self._state.key_s = False
            self._state.key_a = False
            self._state.key_d = False
            self._state.key_q = False
            self._state.key_e = False
            self._state.shift = False

    def snapshot(self) -> TeleopInputState:
        with self._lock:
            return TeleopInputState(**vars(self._state))
