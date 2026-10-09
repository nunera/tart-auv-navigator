from __future__ import annotations

from math import atan2, cos, degrees, sin, sqrt
from pathlib import Path
from time import monotonic

from .calibration import MagnetometerCalibration
from .models import HeadingSample
from .sensors import NmeaGpsReader


class SerialGpsReader(NmeaGpsReader):
    def __init__(self, device: str, *, baudrate: int = 9_600, timeout_s: float = 0.1) -> None:
        try:
            import serial
        except ImportError as error:
            raise RuntimeError("install pyserial to use GPS hardware") from error
        self._serial = serial.Serial(device, baudrate=baudrate, timeout=timeout_s)
        super().__init__(self._serial)

    def close(self) -> None:
        self._serial.close()


class Lsm9ds1HeadingReader:
    """Tilt-compensated, calibrated LSM9DS1 heading adapter.

    `sensor_to_body` makes PCB mounting explicit instead of burying an axis swap
    inside navigation math. Each row selects/reorients one body axis.
    """

    def __init__(
        self,
        *,
        calibration: MagnetometerCalibration,
        sensor_to_body: tuple[tuple[float, float, float], ...] = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    ) -> None:
        if len(sensor_to_body) != 3 or any(len(row) != 3 for row in sensor_to_body):
            raise ValueError("sensor_to_body must be a 3x3 matrix")
        try:
            import adafruit_lsm9ds1
            import board
            import busio
        except ImportError as error:
            raise RuntimeError("install adafruit-blinka and adafruit-circuitpython-lsm9ds1 for AMG hardware") from error
        self._sensor = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))
        self._calibration = calibration
        self._sensor_to_body = sensor_to_body
        self._last = HeadingSample(None, 0.0, False, "no heading yet")

    def read(self, now: float | None = None) -> HeadingSample:
        now = monotonic() if now is None else now
        try:
            acceleration = self._to_body(tuple(float(value) for value in self._sensor.acceleration))
            magnetic = self._to_body(self._calibration.apply(tuple(float(value) for value in self._sensor.magnetic)))
            ax, ay, az = acceleration
            mx, my, mz = magnetic
            if sqrt(ax * ax + ay * ay + az * az) == 0.0 or sqrt(mx * mx + my * my + mz * mz) == 0.0:
                raise ValueError("zero sensor vector")
            roll = atan2(ay, az)
            pitch = atan2(-ax, ay * sin(roll) + az * cos(roll))
            horizontal_x = mx * cos(pitch) + mz * sin(pitch)
            horizontal_y = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch)
            self._last = HeadingSample(degrees(atan2(horizontal_y, horizontal_x)) % 360.0, now, True)
        except (OSError, ValueError, TypeError) as error:
            self._last = HeadingSample(None, now, False, str(error))
        return self._last

    def _to_body(self, vector: tuple[float, float, float]) -> tuple[float, float, float]:
        return tuple(sum(row[column] * vector[column] for column in range(3)) for row in self._sensor_to_body)  # type: ignore[return-value]


class Ads1115TdsReader:
    """Production ADS1115 adapter for the corrected TDS conversion."""

    def __init__(self, analog_channel: int, *, calibration_factor: float = 1.0) -> None:
        if not 0 <= analog_channel <= 3:
            raise ValueError("ADS1115 analog_channel must be in [0, 3]")
        try:
            import board
            import busio
            from adafruit_ads1x15.ads1115 import ADS1115, P0, P1, P2, P3
            from adafruit_ads1x15.analog_in import AnalogIn
        except ImportError as error:
            raise RuntimeError("install adafruit-circuitpython-ads1x15 for TDS hardware") from error
        pin = (P0, P1, P2, P3)[analog_channel]
        self._channel = AnalogIn(ADS1115(busio.I2C(board.SCL, board.SDA)), pin)
        self._calibration_factor = calibration_factor

    def read_ppm(self, temperature_c: float = 25.0) -> float:
        from .sensors import tds_ppm

        return tds_ppm(
            float(self._channel.voltage),
            temperature_c,
            calibration_factor=self._calibration_factor,
        )


def load_heading_reader(calibration_path: Path, sensor_to_body: tuple[tuple[float, float, float], ...]) -> Lsm9ds1HeadingReader:
    return Lsm9ds1HeadingReader(calibration=MagnetometerCalibration.load(calibration_path), sensor_to_body=sensor_to_body)
