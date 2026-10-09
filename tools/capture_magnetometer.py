#!/usr/bin/env python3
"""Capture raw LSM9DS1 magnetic vectors while rotating the powered vehicle through all orientations."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from time import monotonic, sleep


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    arguments = parser.parse_args()
    if arguments.seconds <= 0 or arguments.rate_hz <= 0:
        parser.error("--seconds and --rate-hz must be positive")
    try:
        import adafruit_lsm9ds1
        import board
        import busio
    except ImportError as error:
        parser.error(f"AMG dependencies unavailable: {error}")

    sensor = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    deadline = monotonic() + arguments.seconds
    period = 1.0 / arguments.rate_hz
    with arguments.output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("x", "y", "z"))
        while monotonic() < deadline:
            writer.writerow(tuple(float(value) for value in sensor.magnetic))
            file.flush()
            sleep(period)
    print(f"wrote raw magnetometer samples to {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
