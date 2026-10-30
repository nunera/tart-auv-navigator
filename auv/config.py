from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .navigation import AutonomyLimits, Waypoint


@dataclass(frozen=True)
class RuntimeConfig:
    channels: tuple[int, int, int]
    frequency_hz: float
    speed_scale: float
    horizontal_trim: float
    input_kind: str
    gps_device: str | None
    gps_baudrate: int
    heading_calibration: Path | None
    sensor_to_body: tuple[tuple[float, float, float], ...]
    heading_source: str
    waypoints: list[Waypoint]
    log_directory: Path
    autonomy_limits: AutonomyLimits

    @classmethod
    def load(cls, path: Path) -> "RuntimeConfig":
        data = json.loads(path.read_text(encoding="utf-8"))
        channels = tuple(int(value) for value in data.get("channels", [0, 1, 2]))
        if len(channels) != 3:
            raise ValueError("channels must contain exactly three values")
        sensor_to_body = tuple(tuple(float(value) for value in row) for row in data.get("sensor_to_body", [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        if len(sensor_to_body) != 3 or any(len(row) != 3 for row in sensor_to_body):
            raise ValueError("sensor_to_body must be 3x3")
        return cls(
            channels=channels,  # type: ignore[arg-type]
            frequency_hz=float(data.get("frequency_hz", 50)),
            speed_scale=float(data.get("speed_scale", 1)),
            horizontal_trim=float(data.get("horizontal_trim", 0)),
            input_kind=data.get("input", "keyboard"),
            gps_device=data.get("gps_device"),
            gps_baudrate=int(data.get("gps_baudrate", 9_600)),
            heading_calibration=Path(data["heading_calibration"]) if data.get("heading_calibration") else None,
            sensor_to_body=sensor_to_body,
            heading_source=data.get("heading_source", "amg"),
            waypoints=[Waypoint(latitude=float(item["latitude"]), longitude=float(item["longitude"])) for item in data.get("waypoints", [])],
            log_directory=Path(data.get("log_directory", "Data/GPSlogs")),
            autonomy_limits=_autonomy_limits(data.get("autonomy_limits", {})),
        )


def _autonomy_limits(data: dict) -> AutonomyLimits:
    """Each limit is optional; an omitted or null value disables that limit."""
    known = {"max_duration_s", "max_waypoint_s", "max_distance_m"}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"unknown autonomy_limits keys: {', '.join(sorted(unknown))}")
    return AutonomyLimits(**{key: None if data.get(key) is None else float(data[key]) for key in known})
