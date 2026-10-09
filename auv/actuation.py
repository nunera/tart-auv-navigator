from __future__ import annotations

from enum import Enum, auto
from time import sleep
from typing import Protocol

from .models import MotionCommand


class PwmBackend(Protocol):
    """Internal seam for Navigator PWM and the fake backend used in tests."""

    def initialize(self) -> None: ...
    def set_enabled(self, enabled: bool) -> None: ...
    def set_frequency_hz(self, frequency: float) -> None: ...
    def set_duty_cycle(self, channel: int, duty_cycle: float) -> None: ...


class NavigatorPwmBackend:
    """Production adapter. Imports the ARM-only Navigator binding only when used."""

    def __init__(self) -> None:
        try:
            import bluerobotics_navigator as navigator
            from bluerobotics_navigator import NavigatorVersion, Raspberry
        except ImportError as error:
            raise RuntimeError(
                "Navigator binding is unavailable. Install requirements-runtime.txt on a supported ARM Navigator host."
            ) from error
        self._navigator = navigator
        self._version = NavigatorVersion.Version2
        self._raspberry = Raspberry.Pi4

    def initialize(self) -> None:
        self._navigator.set_navigator_version(self._version)
        self._navigator.set_raspberry_pi_version(self._raspberry)
        self._navigator.init()

    def set_enabled(self, enabled: bool) -> None:
        self._navigator.set_pwm_enable(enabled)

    def set_frequency_hz(self, frequency: float) -> None:
        self._navigator.set_pwm_freq_hz(frequency)

    def set_duty_cycle(self, channel: int, duty_cycle: float) -> None:
        self._navigator.set_pwm_channel_duty_cycle(channel, duty_cycle)


class ActuatorState(Enum):
    OFF = auto()
    DISARMED = auto()
    ARMED = auto()
    FAULT = auto()


class NavigatorActuator:
    """Owns all Navigator PWM output for the vehicle.

    The caller sees only normalized forward/yaw/vertical commands. This module
    owns channel direction, pulse conversion, neutral ordering, and PWM state.
    """

    NEUTRAL_US = 1500.0
    MIN_US = 1000.0
    MAX_US = 2000.0

    def __init__(
        self,
        backend: PwmBackend,
        *,
        channels: tuple[int, int, int] = (0, 1, 2),
        frequency_hz: float = 50.0,
        horizontal_trim: float = 0.0,
        speed_scale: float = 1.0,
        neutral_dwell_s: float = 0.25,
        sleep_fn=sleep,
    ) -> None:
        if len(channels) != 3 or len(set(channels)) != 3 or any(not 0 <= channel <= 15 for channel in channels):
            raise ValueError("channels must be three unique Navigator channels in [0, 15]")
        if not 24.0 <= frequency_hz <= 500.0:
            raise ValueError("frequency_hz must be in [24, 500]")
        if self.MAX_US >= 1_000_000.0 / frequency_hz:
            raise ValueError("PWM period must be longer than the maximum pulse")
        if not -0.3 <= horizontal_trim <= 0.3:
            raise ValueError("horizontal_trim must be in [-0.3, 0.3]")
        if not 0.0 < speed_scale <= 1.0:
            raise ValueError("speed_scale must be in (0, 1]")

        self._backend = backend
        self.channels = channels
        self.frequency_hz = frequency_hz
        self.horizontal_trim = horizontal_trim
        self.speed_scale = speed_scale
        self.neutral_dwell_s = neutral_dwell_s
        self._sleep = sleep_fn
        self.state = ActuatorState.OFF
        self.fault_reason: str | None = None

    def initialize(self) -> None:
        if self.state is not ActuatorState.OFF:
            raise RuntimeError(f"cannot initialize actuator from {self.state.name}")
        self._backend.initialize()
        self._backend.set_enabled(False)
        self._backend.set_frequency_hz(self.frequency_hz)
        self._write_neutral()
        self.state = ActuatorState.DISARMED

    def arm(self) -> None:
        if self.state is ActuatorState.FAULT:
            raise RuntimeError(f"cannot arm faulted actuator: {self.fault_reason}")
        if self.state is not ActuatorState.DISARMED:
            raise RuntimeError(f"cannot arm actuator from {self.state.name}")
        self._write_neutral()
        self._backend.set_enabled(True)
        self._sleep(3.0)
        self.state = ActuatorState.ARMED

    def command(self, command: MotionCommand) -> None:
        if self.state is ActuatorState.FAULT:
            raise RuntimeError(f"faulted actuator refuses commands: {self.fault_reason}")
        if self.state is not ActuatorState.ARMED:
            raise RuntimeError("actuator must be armed before commanding motion")
        first, second, vertical = self._mix(command)
        self._write_pulse(self.channels[0], first)
        self._write_pulse(self.channels[1], second)
        self._write_pulse(self.channels[2], vertical)

    def disarm(self, reason: str = "operator request") -> None:
        if self.state is ActuatorState.OFF:
            return
        try:
            self._write_neutral()
            if self.state is ActuatorState.ARMED:
                self._sleep(self.neutral_dwell_s)
        finally:
            self._backend.set_enabled(False)
            if self.state is not ActuatorState.FAULT:
                self.state = ActuatorState.DISARMED

    def fault(self, reason: str) -> None:
        self.fault_reason = reason
        try:
            self._write_neutral()
            if self.state is ActuatorState.ARMED:
                self._sleep(self.neutral_dwell_s)
        finally:
            self._backend.set_enabled(False)
            self.state = ActuatorState.FAULT

    def _mix(self, command: MotionCommand) -> tuple[float, float, float]:
        if command.is_neutral:
            return self.NEUTRAL_US, self.NEUTRAL_US, self.NEUTRAL_US

        # Trim corrects straight-line travel only; it must never turn an explicit stop into thrust.
        turn = command.yaw + (self.horizontal_trim if command.forward != 0.0 else 0.0)
        turn = max(-1.0, min(1.0, turn))
        forward = command.forward * self.speed_scale
        vertical = command.vertical * self.speed_scale
        motor_one = max(-1.0, min(1.0, (forward + turn) / 2.0))
        motor_two = max(-1.0, min(1.0, -(forward - turn) / 2.0))
        return (
            self._speed_to_pulse(motor_one),
            self._speed_to_pulse(motor_two),
            self._speed_to_pulse(vertical),
        )

    def _speed_to_pulse(self, speed: float) -> float:
        return max(self.MIN_US, min(self.MAX_US, self.NEUTRAL_US + speed * 500.0))

    def _write_neutral(self) -> None:
        for channel in self.channels:
            self._write_pulse(channel, self.NEUTRAL_US)

    def _write_pulse(self, channel: int, pulse_us: float) -> None:
        period_us = 1_000_000.0 / self.frequency_hz
        self._backend.set_duty_cycle(channel, pulse_us / period_us)
