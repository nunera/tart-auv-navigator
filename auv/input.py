from __future__ import annotations

import os
import select
import sys
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from time import monotonic

from .models import ManualInput, MotionCommand


class SharedManualInput:
    """Thread-safe latest-value seam between an input worker and the control loop."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value = ManualInput(observed_at=monotonic())

    def publish(
        self,
        *,
        forward: float | None = None,
        yaw: float | None = None,
        vertical: float | None = None,
        autonomous_requested: bool | None = None,
        emergency_stop_requested: bool | None = None,
    ) -> None:
        with self._lock:
            current = self._value
            self._value = ManualInput(
                command=MotionCommand(
                    current.command.forward if forward is None else forward,
                    current.command.yaw if yaw is None else yaw,
                    current.command.vertical if vertical is None else vertical,
                ),
                autonomous_requested=current.autonomous_requested if autonomous_requested is None else autonomous_requested,
                emergency_stop_requested=(
                    current.emergency_stop_requested if emergency_stop_requested is None else emergency_stop_requested
                ),
                observed_at=monotonic(),
            )

    def snapshot(self) -> ManualInput:
        with self._lock:
            return self._value


SPEED_LEVELS = (0.2, 0.4, 0.6, 0.8, 1.0)

# Key -> (forward, yaw, vertical) direction. Positive yaw turns clockwise (right),
# matching the Xbox stick and the waypoint controller. Q/E drive and turn at once,
# because a terminal only auto-repeats the most recently pressed key.
DRIVE_KEYS: dict[str, tuple[int, int, int]] = {
    "w": (1, 0, 0),
    "s": (-1, 0, 0),
    "a": (0, -1, 0),
    "d": (0, 1, 0),
    "q": (1, -1, 0),
    "e": (1, 1, 0),
    "r": (0, 0, 1),
    "f": (0, 0, -1),
}

KEYBOARD_HELP = (
    "Hold W/S forward/back, A/D turn left/right, Q/E forward+turn, R/F up/down. "
    "Release to stop. 1-5 speed, X stop, M manual/auto, Space E-STOP, Ctrl-C quit."
)


@dataclass
class KeyboardInputWorker:
    """Hold-to-drive terminal control for bench and water tests.

    A terminal reports key presses but not releases, so a held key is seen as
    its auto-repeat stream. A key drives while it keeps repeating and returns to
    zero once the repeats stop. The first press is held long enough to bridge
    the terminal's initial auto-repeat delay. This worker never owns motor output.
    """

    output: SharedManualInput
    stop_event: threading.Event
    level: int = 2
    first_press_hold_s: float = 0.7
    repeat_hold_s: float = 0.25
    fd: int | None = None
    clock: Callable[[], float] = monotonic
    _key: str | None = field(default=None, init=False)
    _direction: tuple[int, int, int] = field(default=(0, 0, 0), init=False)
    _deadline: float = field(default=0.0, init=False)

    @property
    def speed(self) -> float:
        return SPEED_LEVELS[self.level - 1]

    def run(self) -> None:
        fd = sys.stdin.fileno() if self.fd is None else self.fd
        while not self.stop_event.is_set():
            ready, _, _ = select.select([fd], [], [], 0.05)
            if ready:
                data = os.read(fd, 64)
                if not data:
                    return  # stdin closed; input goes stale and the runtime stops the motors
                self.handle_bytes(data, self.clock())
            self.tick(self.clock())

    def handle_bytes(self, data: bytes, now: float) -> None:
        for byte in data:
            if byte == 0x1B:
                return  # arrow/function key escape sequence; its letters are not commands
            self.handle_key(chr(byte).lower(), now)

    def handle_key(self, key: str, now: float) -> None:
        if key in DRIVE_KEYS:
            repeat = key == self._key and now <= self._deadline
            self._key = key
            self._direction = DRIVE_KEYS[key]
            self._deadline = now + (self.repeat_hold_s if repeat else self.first_press_hold_s)
        elif key in ("1", "2", "3", "4", "5"):
            self.level = int(key)
        elif key == "x":
            self._release()
        elif key == "m":
            self._release()
            self.output.publish(autonomous_requested=not self.output.snapshot().autonomous_requested)
        elif key == " ":
            self._release()
            self.output.publish(emergency_stop_requested=True)

    def tick(self, now: float) -> None:
        """Publish the held command, or neutral once the hold has expired."""
        if self._key is not None and now > self._deadline:
            self._release()
        forward, yaw, vertical = (axis * self.speed for axis in self._direction)
        self.output.publish(forward=forward, yaw=yaw, vertical=vertical)

    def _release(self) -> None:
        self._key = None
        self._direction = (0, 0, 0)


@dataclass
class XboxInputWorker:
    """Optional `inputs` gamepad worker. Press START to toggle autonomy; BACK is e-stop."""

    output: SharedManualInput
    stop_event: threading.Event
    deadzone: int = 3_000
    _left_trigger: float = field(default=0.0, init=False)
    _right_trigger: float = field(default=0.0, init=False)

    def run(self) -> None:
        try:
            from inputs import get_gamepad
        except ImportError as error:
            raise RuntimeError("install inputs to use --input xbox") from error

        while not self.stop_event.is_set():
            for event in get_gamepad():
                if self.stop_event.is_set():
                    return
                if event.code == "ABS_Y":
                    self.output.publish(forward=self._axis(-event.state))
                elif event.code == "ABS_X":
                    self.output.publish(yaw=self._axis(event.state))
                elif event.code == "ABS_RZ":
                    self._right_trigger = self._trigger(event.state)
                    self.output.publish(vertical=self._right_trigger - self._left_trigger)
                elif event.code == "ABS_Z":
                    self._left_trigger = self._trigger(event.state)
                    self.output.publish(vertical=self._right_trigger - self._left_trigger)
                elif event.code == "BTN_START" and event.state:
                    self.output.publish(autonomous_requested=not self.output.snapshot().autonomous_requested)
                elif event.code == "BTN_SELECT" and event.state:
                    self.output.publish(emergency_stop_requested=True)

    def _axis(self, value: int) -> float:
        return 0.0 if abs(value) < self.deadzone else max(-1.0, min(1.0, value / 32_767.0))

    def _trigger(self, value: int) -> float:
        return max(0.0, min(1.0, value / 255.0))
