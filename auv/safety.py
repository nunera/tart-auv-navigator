from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from .models import HealthSnapshot, ManualInput, MotionCommand


class ControlMode(Enum):
    MANUAL = auto()
    AUTONOMOUS = auto()


@dataclass(frozen=True)
class SafetyDecision:
    command: MotionCommand
    allow_motion: bool
    fault_reason: str | None = None


class SafetyController:
    """Concentrates movement authorization in one place."""

    def decide(
        self,
        *,
        mode: ControlMode,
        manual: ManualInput,
        autonomous: MotionCommand,
        health: HealthSnapshot,
        permit_gps_heading_acquisition: bool = False,
    ) -> SafetyDecision:
        if manual.emergency_stop_requested:
            return SafetyDecision(MotionCommand(), False, "operator emergency stop")
        if health.worker_failure:
            return SafetyDecision(MotionCommand(), False, f"worker failure: {health.worker_failure}")
        if not health.navigator_ready:
            return SafetyDecision(MotionCommand(), False, "Navigator unavailable")

        if mode is ControlMode.MANUAL:
            if not health.input_fresh:
                return SafetyDecision(MotionCommand(), False, "manual input stale")
            return SafetyDecision(manual.command, True)

        if not health.gps_fresh:
            return SafetyDecision(MotionCommand(), False, "GPS unavailable or stale")
        if not health.heading_fresh:
            acquiring_gps_heading = (
                permit_gps_heading_acquisition
                and 0.0 < autonomous.forward <= 0.1
                and autonomous.yaw == 0.0
                and autonomous.vertical == 0.0
            )
            if not acquiring_gps_heading:
                return SafetyDecision(MotionCommand(), False, "heading unavailable or stale")
        return SafetyDecision(autonomous, True)
