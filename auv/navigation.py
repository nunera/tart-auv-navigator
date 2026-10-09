from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2, cos, degrees, radians, sin, sqrt

from .models import MotionCommand, NavigationObservation


EARTH_RADIUS_M = 6_371_008.8


@dataclass(frozen=True)
class Waypoint:
    latitude: float
    longitude: float

    def __post_init__(self) -> None:
        if not -90.0 <= self.latitude <= 90.0 or not -180.0 <= self.longitude <= 180.0:
            raise ValueError("waypoint outside geographic bounds")


def heading_error(target_degrees: float, current_degrees: float) -> float:
    """Shortest signed target-current angle in [-180, 180). Positive is CCW/left."""
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


class NavigationController:
    """Pure waypoint controller; invalid observations always produce neutral output."""

    def __init__(
        self,
        waypoints: list[Waypoint],
        *,
        heading_source: str = "amg",
        arrival_radius_m: float = 5.0,
        drive_gain: float = 0.05,
        turn_gain: float = 0.02,
        max_heading_acquisition_forward: float = 0.1,
        max_command_age_s: float = 0.5,
    ) -> None:
        if heading_source not in {"amg", "gps"}:
            raise ValueError("heading_source must be 'amg' or 'gps'")
        self._initial_waypoints = tuple(waypoints)
        self.heading_source = heading_source
        self.arrival_radius_m = arrival_radius_m
        self.drive_gain = drive_gain
        self.turn_gain = turn_gain
        self.max_heading_acquisition_forward = max_heading_acquisition_forward
        self.max_command_age_s = max_command_age_s
        self.enabled = False
        self.complete = not self._initial_waypoints
        self._waypoint_index = 0
        self._last_command = MotionCommand()

    @property
    def target(self) -> Waypoint:
        return self._initial_waypoints[self._waypoint_index]

    def enable(self) -> None:
        self.complete = not self._initial_waypoints
        self.enabled = not self.complete
        self._last_command = MotionCommand()

    def disable(self) -> None:
        self.enabled = False
        self._last_command = MotionCommand()

    def reset(self) -> None:
        self.disable()
        self.complete = not self._initial_waypoints
        self._waypoint_index = 0

    def tick(self, observation: NavigationObservation) -> MotionCommand:
        if not self.enabled or self.complete:
            return MotionCommand()
        if not observation.position.valid or observation.position.latitude is None or observation.position.longitude is None:
            return MotionCommand()
        if observation.now - observation.position.observed_at > self.max_command_age_s:
            return MotionCommand()

        distance = distance_m(observation.position.latitude, observation.position.longitude, self.target)
        if distance <= self.arrival_radius_m:
            if self._waypoint_index == len(self._initial_waypoints) - 1:
                self.complete = True
                self.enabled = False
                return MotionCommand()
            self._waypoint_index += 1
            return MotionCommand()

        heading = observation.heading
        if not heading.valid or heading.degrees is None or observation.now - heading.observed_at > self.max_command_age_s:
            if self.heading_source == "gps":
                return MotionCommand(forward=self.max_heading_acquisition_forward)
            return MotionCommand()

        target_bearing = bearing_degrees(observation.position.latitude, observation.position.longitude, self.target)
        error = heading_error(target_bearing, heading.degrees)
        forward = min(1.0, max(0.0, self.drive_gain * distance))
        yaw = min(1.0, max(-1.0, self.turn_gain * error))
        command = MotionCommand(forward=forward, yaw=yaw)
        self._last_command = command
        return command
