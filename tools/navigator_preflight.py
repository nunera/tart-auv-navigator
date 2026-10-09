#!/usr/bin/env python3
"""Probe the Navigator native binding in a disposable process before arming the AUV."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from auv.actuation import NavigatorActuator, NavigatorPwmBackend


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channels", default="0,1,2", help="three comma-separated PWM channels")
    parser.add_argument("--frequency-hz", type=float, default=50.0)
    arguments = parser.parse_args()
    channels = tuple(int(value.strip()) for value in arguments.channels.split(","))
    if len(channels) != 3:
        parser.error("--channels must contain exactly three values")

    actuator = NavigatorActuator(
        NavigatorPwmBackend(),
        channels=channels,  # type: ignore[arg-type]
        frequency_hz=arguments.frequency_hz,
    )
    actuator.initialize()
    actuator.disarm("preflight complete")
    print("Navigator native initialization and neutral PWM write succeeded.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
