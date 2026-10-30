from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2, cos, degrees, isfinite, radians, sin, sqrt

from .models import MotionCommand, NavigationObservation, PositionSample


EARTH_RADIUS_M = 6_371_008.8


@dataclass(frozen=True)
class Waypoint:
    latitude: float
    longitude: float

    def __post_init__(self) -> None:
        if not -90.0 <= self.latitude <= 90.0 or not -180.0 <= self.longitude <= 180.0:
            raise ValueError("waypoint outside geographic bounds")


def heading_error(target_degrees: float, current_degrees: float) -> float:
    """Shortest signed target-current angle in [-180, 180). Positive is clockwise/right."""
    return (target_degrees - current_degrees + 180.0) % 360.0 - 180.0


def bearing_degrees(latitude: float, longitude: float, target: Waypoint) -> float:
    lat1, lon1, lat2, lon2 = map(radians, (latitude, longitude, target.latitude, target.longitude))
    delta_lon = lon2 - lon1
    y = sin(delta_lon) * cos(lat2)
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)
    return degrees(atan2(y, x)) % 360.0


def distance_m(latitude: float, longitude: float, target: Waypoint) -> float:
    lat1, lon1, lat2, lon2 = map(radians, (latitude, longitude, target.latitude, target.longitude))
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = sin(dlat / 2.0) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * asin(sqrt(a))


@dataclass(frozen=True)
class AutonomyLimits:
    """Optional bounds on one autonomous run. `None` disables that limit."""

    max_duration_s: float | None = None
    max_waypoint_s: float | None = None
    max_distance_m: float | None = None

    def __post_init__(self) -> None:
        for name in ("max_duration_s", "max_waypoint_s", "max_distance_m"):
            value = getattr(self, name)
            if value is not None and (not isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be a positive number or omitted")


class NavigationController:
    """Pure waypoint controller; invalid observations always produce neutral output.

    Each enable() starts a new autonomous run. A run that breaches its limits
    stops itself and records `abort_reason`; only a new enable() clears it.
    """

    def __init__(
        self,
        waypoints: list[Waypoint],
        *,
        heading_source: str = "amg",
        arrival_radius_m: float = 5.0,
        drive_gain: float = 0.05,
        turn_gain: float = 0.02,
        max_heading_acquisition_forward: float = 0.1,
        max_position_age_s: float = 1.5,
        max_heading_age_s: float | None = None,
        limits: AutonomyLimits = AutonomyLimits(),
    ) -> None:
        if heading_source not in {"amg", "gps"}:
            raise ValueError("heading_source must be 'amg' or 'gps'")
        self._initial_waypoints = tuple(waypoints)
        self.heading_source = heading_source
        self.arrival_radius_m = arrival_radius_m
        self.drive_gain = drive_gain
        self.turn_gain = turn_gain
        self.max_heading_acquisition_forward = max_heading_acquisition_forward
        # GPS fixes typically arrive at 1 Hz, so a position stays usable between fixes.
        self.max_position_age_s = max_position_age_s
        # GPS track heading only updates after the vehicle has moved a few metres.
        self.max_heading_age_s = (
            max_heading_age_s if max_heading_age_s is not None else (5.0 if heading_source == "gps" else 0.5)
        )
        self.limits = limits
        self.enabled = False
        self.complete = not self._initial_waypoints
        self.abort_reason: str | None = None
        self._waypoint_index = 0
        self._run_started_at: float | None = None
        self._waypoint_started_at: float | None = None
        self._run_origin: PositionSample | None = None

    @property
    def waypoints(self) -> tuple[Waypoint, ...]:
        return self._initial_waypoints

    @property
    def target(self) -> Waypoint:
        return self._initial_waypoints[self._waypoint_index]

    def enable(self) -> None:
        self.complete = not self._initial_waypoints
        self.enabled = not self.complete
        self.abort_reason = None
        self._run_started_at = None
        self._waypoint_started_at = None
        self._run_origin = None

    def disable(self) -> None:
        self.enabled = False

    def reset(self) -> None:
        self.disable()
        self.complete = not self._initial_waypoints
        self.abort_reason = None
        self._waypoint_index = 0

    def tick(self, observation: NavigationObservation) -> MotionCommand:
        if not self.enabled or self.complete:
            return MotionCommand()
        now = observation.now
        if self._run_started_at is None:
            self._run_started_at = now
            self._waypoint_started_at = now
        limits = self.limits
        if limits.max_duration_s is not None and now - self._run_started_at > limits.max_duration_s:
            return self._abort(f"autonomous run exceeded {limits.max_duration_s:g} s")
        if limits.max_waypoint_s is not None and now - self._waypoint_started_at > limits.max_waypoint_s:
            return self._abort(f"waypoint {self._waypoint_index + 1} not reached within {limits.max_waypoint_s:g} s")

        position = observation.position
        if not position.valid or position.latitude is None or position.longitude is None:
            return MotionCommand()
        if now - position.observed_at > self.max_position_age_s:
            return MotionCommand()

        if self._run_origin is None:
            self._run_origin = position
        if limits.max_distance_m is not None:
            assert self._run_origin.latitude is not None and self._run_origin.longitude is not None
            travelled = distance_m(
                self._run_origin.latitude, self._run_origin.longitude, Waypoint(position.latitude, position.longitude)
            )
            if travelled > limits.max_distance_m:
                return self._abort(f"{travelled:.0f} m from autonomy start exceeds {limits.max_distance_m:g} m")

        distance = distance_m(position.latitude, position.longitude, self.target)
        if distance <= self.arrival_radius_m:
            if self._waypoint_index == len(self._initial_waypoints) - 1:
                self.complete = True
                self.enabled = False
                return MotionCommand()
            self._waypoint_index += 1
            self._waypoint_started_at = now
            return MotionCommand()

        heading = observation.heading
        if not heading.valid or heading.degrees is None or now - heading.observed_at > self.max_heading_age_s:
            if self.heading_source == "gps":
                return MotionCommand(forward=self.max_heading_acquisition_forward)
            return MotionCommand()

        target_bearing = bearing_degrees(position.latitude, position.longitude, self.target)
        error = heading_error(target_bearing, heading.degrees)
        forward = min(1.0, max(0.0, self.drive_gain * distance))
        yaw = min(1.0, max(-1.0, self.turn_gain * error))
        return MotionCommand(forward=forward, yaw=yaw)

    def _abort(self, reason: str) -> MotionCommand:
        self.abort_reason = reason
        self.enabled = False
        return MotionCommand()
