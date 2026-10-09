from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class MotionCommand:
    """Normalized vehicle command. Every axis is constrained to [-1, 1]."""

    forward: float = 0.0
    yaw: float = 0.0
    vertical: float = 0.0

    def __post_init__(self) -> None:
        for name, value in (("forward", self.forward), ("yaw", self.yaw), ("vertical", self.vertical)):
            if not isfinite(value) or not -1.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and in [-1, 1], got {value!r}")

    @property
    def is_neutral(self) -> bool:
        return self.forward == 0.0 and self.yaw == 0.0 and self.vertical == 0.0


@dataclass(frozen=True)
class PositionSample:
    latitude: float | None
    longitude: float | None
    observed_at: float
    valid: bool
    error: str | None = None

    def __post_init__(self) -> None:
        coordinates_present = self.latitude is not None and self.longitude is not None
        if self.valid:
            if not coordinates_present:
                raise ValueError("valid position requires latitude and longitude")
            if not -90.0 <= self.latitude <= 90.0 or not -180.0 <= self.longitude <= 180.0:
                raise ValueError("position outside geographic bounds")
            if self.latitude == 0.0 and self.longitude == 0.0:
                raise ValueError("(0, 0) is never a valid vehicle position")


@dataclass(frozen=True)
class HeadingSample:
    degrees: float | None
    observed_at: float
    valid: bool
    error: str | None = None

    def __post_init__(self) -> None:
        if self.valid and (self.degrees is None or not isfinite(self.degrees)):
            raise ValueError("valid heading requires finite degrees")


@dataclass(frozen=True)
class NavigationObservation:
    position: PositionSample
    heading: HeadingSample
    now: float


@dataclass(frozen=True)
class ManualInput:
    command: MotionCommand = MotionCommand()
    autonomous_requested: bool = False
    emergency_stop_requested: bool = False
    observed_at: float = 0.0


@dataclass(frozen=True)
class HealthSnapshot:
    navigator_ready: bool
    input_fresh: bool
    gps_fresh: bool
    heading_fresh: bool
    worker_failure: str | None = None
