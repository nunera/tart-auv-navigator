from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass
from time import monotonic, sleep
from typing import Protocol

from .actuation import ActuatorState, NavigatorActuator
from .input import SharedManualInput
from .logging import RunLogger
from .models import HeadingSample, HealthSnapshot, MotionCommand, NavigationObservation, PositionSample
from .navigation import NavigationController
from .safety import ControlMode, SafetyController
from .sensors import GpsTrackHeading


class PositionReader(Protocol):
    def read(self, now: float | None = None) -> PositionSample: ...


class RuntimeHeadingReader(Protocol):
    def read(self, now: float | None = None) -> HeadingSample: ...


@dataclass(frozen=True)
class RuntimeStatus:
    """What the last control step decided, for operator display."""

    mode: ControlMode
    command: MotionCommand
    reason: str
    actuator: ActuatorState


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
        self._clock = clock
        self._worker_failure: str | None = None
        self._track_heading = GpsTrackHeading() if navigation.heading_source == "gps" else None
        self._mode = ControlMode.MANUAL
        # Manual control resumes only after the operator's command passes through neutral:
        # at startup and after every autonomous-to-manual switch.
        self._manual_release_required = True
        self.status: RuntimeStatus | None = None

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
            if self._track_heading is not None:
                heading = self._track_heading.update(position)
            else:
                heading = self.heading_reader.read(now)
        except Exception as error:
            self._worker_failure = str(error)
            position = PositionSample(None, None, now, False, "sensor reader exception")
            heading = HeadingSample(None, now, False, "sensor reader exception")

        mode = ControlMode.AUTONOMOUS if manual.autonomous_requested else ControlMode.MANUAL
        if mode is not self._mode:
            if mode is ControlMode.AUTONOMOUS:
                self.navigation.enable()
            else:
                self.navigation.disable()
                self._manual_release_required = True
            self._mode = mode
        if self._manual_release_required and manual.command.is_neutral:
            self._manual_release_required = False

        autonomous = self.navigation.tick(NavigationObservation(position=position, heading=heading, now=now))
        health = HealthSnapshot(
            navigator_ready=self.actuator.state is ActuatorState.ARMED,
            input_fresh=now - manual.observed_at <= self.input_timeout_s,
            gps_fresh=position.valid and now - position.observed_at <= self.navigation.max_position_age_s,
            heading_fresh=heading.valid and now - heading.observed_at <= self.navigation.max_heading_age_s,
            worker_failure=self._worker_failure,
        )
        decision = self.safety.decide(
            mode=mode,
            manual=manual,
            autonomous=autonomous,
            health=health,
            permit_gps_heading_acquisition=self.navigation.heading_source == "gps",
            manual_release_required=self._manual_release_required,
            autonomy_stopped=self._autonomy_stopped(),
        )

        if self.actuator.state is ActuatorState.ARMED:
            if manual.emergency_stop_requested or self._worker_failure:
                self.actuator.fault(decision.fault_reason or "emergency stop")
            else:
                self.actuator.command(decision.command)
        self.status = RuntimeStatus(mode, decision.command, decision.fault_reason or "", self.actuator.state)
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

    def _autonomy_stopped(self) -> str | None:
        if self._mode is not ControlMode.AUTONOMOUS:
            return None
        if self.navigation.abort_reason:
            return f"autonomy stopped: {self.navigation.abort_reason}; switch to manual"
        if not self.navigation.waypoints:
            return "no waypoints configured"
        if self.navigation.complete:
            return "all waypoints reached"
        return None

    def run(
        self,
        stop_event: threading.Event,
        *,
        period_s: float = 0.05,
        after_step: Callable[[], None] | None = None,
    ) -> None:
        if period_s <= 0:
            raise ValueError("period_s must be positive")
        while not stop_event.is_set():
            started = self._clock()
            self.step()
            if after_step is not None:
                after_step()
            sleep(max(0.0, period_s - (self._clock() - started)))

    def shutdown(self, reason: str = "shutdown") -> None:
        try:
            self.actuator.disarm(reason)
        finally:
            self.logger.close()
