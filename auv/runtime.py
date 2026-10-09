from __future__ import annotations

import threading
from time import monotonic, sleep
from typing import Protocol

from .actuation import ActuatorState, NavigatorActuator
from .input import SharedManualInput
from .logging import RunLogger
from .models import HeadingSample, HealthSnapshot, MotionCommand, NavigationObservation, PositionSample
from .navigation import NavigationController
from .safety import ControlMode, SafetyController
from .sensors import gps_track_heading


class PositionReader(Protocol):
    def read(self, now: float | None = None) -> PositionSample: ...


class RuntimeHeadingReader(Protocol):
    def read(self, now: float | None = None) -> HeadingSample: ...


class ControlRuntime:
    """Single motor-output owner. Input and sensor adapters cannot command PWM directly."""

    def __init__(
        self,
        *,
        actuator: NavigatorActuator,
        manual_input: SharedManualInput,
        position_reader: PositionReader,
        heading_reader: RuntimeHeadingReader,
        navigation: NavigationController,
        safety: SafetyController,
        logger: RunLogger,
        input_timeout_s: float = 0.5,
        gps_timeout_s: float = 1.5,
        heading_timeout_s: float = 0.5,
        clock=monotonic,
    ) -> None:
        self.actuator = actuator
        self.manual_input = manual_input
        self.position_reader = position_reader
        self.heading_reader = heading_reader
        self.navigation = navigation
        self.safety = safety
        self.logger = logger
        self.input_timeout_s = input_timeout_s
        self.gps_timeout_s = gps_timeout_s
        self.heading_timeout_s = heading_timeout_s
        self._clock = clock
        self._worker_failure: str | None = None
        self._previous_position: PositionSample | None = None

    def initialize(self, *, arm: bool) -> None:
        self.logger.open()
        try:
            self.actuator.initialize()
            if arm:
                self.actuator.arm()
        except BaseException:
            self.logger.close()
            raise

    def step(self) -> MotionCommand:
        now = self._clock()
        manual = self.manual_input.snapshot()
        try:
            position = self.position_reader.read(now)
            if self.navigation.heading_source == "gps":
                heading = (
                    gps_track_heading(self._previous_position, position)
                    if self._previous_position is not None
                    else HeadingSample(None, now, False, "awaiting a second GPS fix")
                )
                self._previous_position = position
            else:
                heading = self.heading_reader.read(now)
        except Exception as error:
            self._worker_failure = str(error)
            position = PositionSample(None, None, now, False, "sensor reader exception")
            heading = HeadingSample(None, now, False, "sensor reader exception")

        mode = ControlMode.AUTONOMOUS if manual.autonomous_requested else ControlMode.MANUAL
        if mode is ControlMode.AUTONOMOUS:
            if not self.navigation.enabled and not self.navigation.complete:
                self.navigation.enable()
        else:
            self.navigation.disable()
        autonomous = self.navigation.tick(NavigationObservation(position=position, heading=heading, now=now))
        health = HealthSnapshot(
            navigator_ready=self.actuator.state is ActuatorState.ARMED,
            input_fresh=now - manual.observed_at <= self.input_timeout_s,
            gps_fresh=position.valid and now - position.observed_at <= self.gps_timeout_s,
            heading_fresh=heading.valid and now - heading.observed_at <= self.heading_timeout_s,
            worker_failure=self._worker_failure,
        )
        decision = self.safety.decide(
            mode=mode,
            manual=manual,
            autonomous=autonomous,
            health=health,
            permit_gps_heading_acquisition=self.navigation.heading_source == "gps",
        )

        if self.actuator.state is ActuatorState.ARMED:
            if manual.emergency_stop_requested or self._worker_failure:
                self.actuator.fault(decision.fault_reason or "emergency stop")
            else:
                self.actuator.command(decision.command)
        try:
            self.logger.record(
                now=now,
                position=position,
                heading=heading,
                command=decision.command,
                mode=mode.name.lower(),
                reason=decision.fault_reason or "",
            )
        except Exception as error:
            self.actuator.fault(f"logging failure: {error}")
            raise RuntimeError("logging failed; actuator faulted") from error
        return decision.command

    def run(self, stop_event: threading.Event, *, period_s: float = 0.05) -> None:
        if period_s <= 0:
            raise ValueError("period_s must be positive")
        while not stop_event.is_set():
            started = self._clock()
            self.step()
            sleep(max(0.0, period_s - (self._clock() - started)))

    def shutdown(self, reason: str = "shutdown") -> None:
        try:
            self.actuator.disarm(reason)
        finally:
            self.logger.close()
