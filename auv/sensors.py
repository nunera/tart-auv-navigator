from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
from math import atan2, cos, degrees, radians, sin
from time import monotonic
from typing import Protocol

from .models import HeadingSample, PositionSample


class LineSource(Protocol):
    def readline(self) -> bytes: ...


@dataclass(frozen=True)
class SensorFailure:
    message: str


def _nmea_degrees(raw: str, direction: str) -> float:
    value = float(raw)
    whole_degrees = int(value // 100)
    result = whole_degrees + (value - whole_degrees * 100.0) / 60.0
    return -result if direction in {"S", "W"} else result


def _valid_checksum(line: str) -> bool:
    if not line.startswith("$") or "*" not in line:
        return False
    payload, checksum = line[1:].rsplit("*", 1)
    if len(checksum) != 2:
        return False
    calculated = 0
    for character in payload:
        calculated ^= ord(character)
    try:
        return calculated == int(checksum, 16)
    except ValueError:
        return False


class NmeaGpsReader:
    """Reads GPRMC positions; an invalid/void fix is never published as current."""

    def __init__(self, source: LineSource, *, max_fix_age_s: float = 1.5) -> None:
        self._source = source
        self.max_fix_age_s = max_fix_age_s
        self._last_position = PositionSample(None, None, 0.0, False, "no fix yet")
        self._recent_positions: deque[PositionSample] = deque(maxlen=5)

    def read(self, now: float | None = None) -> PositionSample:
        now = monotonic() if now is None else now
        raw = self._source.readline().decode("ascii", errors="ignore").strip()
        if not raw:
            return self._expired_or_last(now, "no NMEA sentence")
        if not raw.startswith("$GPRMC"):
            return self._expired_or_last(now, "no RMC sentence")
        if not _valid_checksum(raw):
            return self._expired_or_last(now, "invalid NMEA checksum")

        fields = raw[1:].split("*", 1)[0].split(",")
        if len(fields) < 7 or fields[2] != "A":
            self._last_position = PositionSample(None, None, now, False, "GPS reports void fix")
            return self._last_position
        try:
            latitude = _nmea_degrees(fields[3], fields[4])
            longitude = _nmea_degrees(fields[5], fields[6])
            sample = PositionSample(latitude, longitude, now, True)
        except (IndexError, ValueError):
            self._last_position = PositionSample(None, None, now, False, "invalid RMC coordinates")
            return self._last_position

        self._recent_positions.append(sample)
        latitude = sum(item.latitude for item in self._recent_positions if item.latitude is not None) / len(self._recent_positions)
        longitude = sum(item.longitude for item in self._recent_positions if item.longitude is not None) / len(self._recent_positions)
        self._last_position = PositionSample(latitude, longitude, now, True)
        return self._last_position

    def _expired_or_last(self, now: float, error: str) -> PositionSample:
        if not self._last_position.valid or now - self._last_position.observed_at > self.max_fix_age_s:
            self._last_position = PositionSample(None, None, now, False, error)
        return self._last_position


class HeadingReader(Protocol):
    def read(self, now: float | None = None) -> HeadingSample: ...


class VectorHeadingReader:
    """Converts injected calibrated horizontal magnetic vectors into timestamped heading samples."""

    def __init__(self, vector_source, *, max_age_s: float = 0.5) -> None:
        self._vector_source = vector_source
        self.max_age_s = max_age_s
        self._last = HeadingSample(None, 0.0, False, "no heading yet")

    def read(self, now: float | None = None) -> HeadingSample:
        now = monotonic() if now is None else now
        try:
            x, y = self._vector_source()
            if x == 0 and y == 0:
                raise ValueError("zero magnetic vector")
            self._last = HeadingSample(degrees(atan2(y, x)) % 360.0, now, True)
        except (OSError, ValueError, TypeError) as error:
            if now - self._last.observed_at > self.max_age_s:
                self._last = HeadingSample(None, now, False, str(error))
        return self._last


def gps_track_heading(previous: PositionSample, current: PositionSample) -> HeadingSample:
    """Return a valid heading only when both fixes are current and distinct."""
    if not previous.valid or not current.valid or None in (previous.latitude, previous.longitude, current.latitude, current.longitude):
        return HeadingSample(None, current.observed_at, False, "insufficient valid GPS fixes")
    if previous.latitude == current.latitude and previous.longitude == current.longitude:
        return HeadingSample(None, current.observed_at, False, "GPS position unchanged")
    lat1, lat2 = radians(previous.latitude), radians(current.latitude)
    delta_lon = radians(current.longitude - previous.longitude)
    y = sin(delta_lon) * cos(lat2)
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)
    return HeadingSample(degrees(atan2(y, x)) % 360.0, current.observed_at, True)


def tds_ppm(voltage: float, temperature_c: float = 25.0, *, calibration_factor: float = 1.0) -> float:
    """Convert a compensated analogue TDS voltage to ppm.

    The middle term is voltage squared. The legacy expression used voltage cubed
    twice and therefore materially distorted every non-zero reading.
    """
    if temperature_c <= -25.0:
        raise ValueError("temperature compensation would divide by zero or invert")
    compensated_voltage = voltage / (1.0 + 0.02 * (temperature_c - 25.0))
    ppm = (
        133.42 * compensated_voltage**3
        - 255.86 * compensated_voltage**2
        + 857.39 * compensated_voltage
    ) * calibration_factor * 0.5
    return float(Decimal(str(max(0.0, ppm))).quantize(Decimal("0.01"), rounding=ROUND_HALF_UP))
