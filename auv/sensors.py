from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
from math import atan2, degrees
from time import monotonic
from typing import Protocol

from .models import HeadingSample, PositionSample
from .navigation import Waypoint, bearing_degrees, distance_m


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
            return self._invalidate(now, "GPS reports void fix")
        try:
            latitude = _nmea_degrees(fields[3], fields[4])
            longitude = _nmea_degrees(fields[5], fields[6])
            sample = PositionSample(latitude, longitude, now, True)
        except (IndexError, ValueError):
            return self._invalidate(now, "invalid RMC coordinates")

        self._recent_positions.append(sample)
        latitude = sum(item.latitude for item in self._recent_positions if item.latitude is not None) / len(self._recent_positions)
        longitude = sum(item.longitude for item in self._recent_positions if item.longitude is not None) / len(self._recent_positions)
        self._last_position = PositionSample(latitude, longitude, now, True)
        return self._last_position

    def _expired_or_last(self, now: float, error: str) -> PositionSample:
        if not self._last_position.valid or now - self._last_position.observed_at > self.max_fix_age_s:
            return self._invalidate(now, error)
        return self._last_position

    def _invalidate(self, now: float, error: str) -> PositionSample:
        # Fixes from before a gap must not be averaged into the next valid fix.
        self._recent_positions.clear()
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


class GpsTrackHeading:
    """Course over ground from GPS fixes at least `min_distance_m` apart.

    The control loop runs much faster than GPS fixes arrive, so consecutive
    ticks usually repeat one fix. Heading therefore comes from accumulated
    displacement, and the last heading is kept with its fix time so consumers
    can apply their own age limit.
    """

    def __init__(self, *, min_distance_m: float = 2.0) -> None:
        if min_distance_m <= 0:
            raise ValueError("min_distance_m must be positive")
        self.min_distance_m = min_distance_m
        self._anchor: PositionSample | None = None
        self._last = HeadingSample(None, 0.0, False, "awaiting GPS movement")

    def update(self, position: PositionSample) -> HeadingSample:
        if not position.valid or position.latitude is None or position.longitude is None:
            self._anchor = None
            self._last = HeadingSample(None, position.observed_at, False, "GPS fix lost")
            return self._last
        anchor = self._anchor
        if anchor is None or anchor.latitude is None or anchor.longitude is None:
            self._anchor = position
            return self._last
        current = Waypoint(position.latitude, position.longitude)
        if distance_m(anchor.latitude, anchor.longitude, current) < self.min_distance_m:
            return self._last
        self._last = HeadingSample(bearing_degrees(anchor.latitude, anchor.longitude, current), position.observed_at, True)
        self._anchor = position
        return self._last


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
