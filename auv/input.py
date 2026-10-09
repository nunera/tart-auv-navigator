from __future__ import annotations

import select
import sys
import threading
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


@dataclass
class KeyboardInputWorker:
    """Terminal keyboard control for bench and water tests.

    W/S drive, A/D yaw, R/F vertical, M toggles autonomous, space latches e-stop,
    and X writes an explicit neutral command. This worker never owns motor output.
    """

    output: SharedManualInput
    stop_event: threading.Event
    increment: float = 0.1

    def run(self) -> None:
        while not self.stop_event.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            key = sys.stdin.read(1).lower()
            if key == "w":
                self._adjust(forward=self.increment)
            elif key == "s":
                self._adjust(forward=-self.increment)
            elif key == "a":
                self._adjust(yaw=self.increment)
            elif key == "d":
                self._adjust(yaw=-self.increment)
            elif key == "r":
                self._adjust(vertical=self.increment)
            elif key == "f":
                self._adjust(vertical=-self.increment)
            elif key == "m":
                self.output.publish(autonomous_requested=not self.output.snapshot().autonomous_requested)
            elif key == " ":
                self.output.publish(emergency_stop_requested=True)
            elif key == "x":
                self.output.publish(forward=0.0, yaw=0.0, vertical=0.0)

    def _adjust(self, *, forward: float = 0.0, yaw: float = 0.0, vertical: float = 0.0) -> None:
        current = self.output.snapshot().command
        self.output.publish(
            forward=max(-1.0, min(1.0, current.forward + forward)),
            yaw=max(-1.0, min(1.0, current.yaw + yaw)),
            vertical=max(-1.0, min(1.0, current.vertical + vertical)),
        )


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
