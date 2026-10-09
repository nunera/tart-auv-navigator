from __future__ import annotations

import argparse
import signal
import sys
import threading
from contextlib import contextmanager
from datetime import UTC, datetime
from pathlib import Path

from .actuation import NavigatorActuator, NavigatorPwmBackend
from .config import RuntimeConfig
from .hardware import SerialGpsReader, load_heading_reader
from .input import KeyboardInputWorker, SharedManualInput, XboxInputWorker
from .logging import RunLogger
from .models import HeadingSample, PositionSample
from .navigation import NavigationController
from .runtime import ControlRuntime
from .safety import SafetyController


class UnavailablePositionReader:
    def read(self, now=None) -> PositionSample:
        return PositionSample(None, None, now or 0.0, False, "GPS not configured")


class UnavailableHeadingReader:
    def read(self, now=None) -> HeadingSample:
        return HeadingSample(None, now or 0.0, False, "AMG not configured")


@contextmanager
def terminal_raw_mode(enabled: bool):
    if not enabled or not sys.stdin.isatty():
        yield
        return
    import termios
    import tty

    attributes = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, attributes)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="Navigator AUV runtime")
    result.add_argument("--config", type=Path, default=Path("config/vehicle.json"))
    result.add_argument("--arm", action="store_true", help="enable PWM after initialization; omitted means telemetry-only")
    result.add_argument("--period", type=float, default=0.05, help="control period in seconds")
    return result


def main() -> int:
    arguments = parser().parse_args()
    config = RuntimeConfig.load(arguments.config)

    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())
    manual = SharedManualInput()
    worker = {
        "keyboard": KeyboardInputWorker(manual, stop_event),
        "xbox": XboxInputWorker(manual, stop_event),
    }.get(config.input_kind)
    if worker is None:
        print("input must be 'keyboard' or 'xbox'", file=sys.stderr)
        return 2
    input_thread = threading.Thread(target=worker.run, name=f"{config.input_kind}-input", daemon=True)

    position_reader = (
        SerialGpsReader(config.gps_device, baudrate=config.gps_baudrate) if config.gps_device else UnavailablePositionReader()
    )
    heading_reader = (
        load_heading_reader(config.heading_calibration, config.sensor_to_body)
        if config.heading_calibration
        else UnavailableHeadingReader()
    )
    log_name = datetime.now(UTC).strftime("run-%Y%m%dT%H%M%SZ.csv")
    runtime = ControlRuntime(
        actuator=NavigatorActuator(
            NavigatorPwmBackend(),
            channels=config.channels,
            frequency_hz=config.frequency_hz,
            speed_scale=config.speed_scale,
            horizontal_trim=config.horizontal_trim,
        ),
        manual_input=manual,
        position_reader=position_reader,
        heading_reader=heading_reader,
        navigation=NavigationController(config.waypoints, heading_source=config.heading_source),
        safety=SafetyController(),
        logger=RunLogger(config.log_directory / log_name),
    )

    try:
        runtime.initialize(arm=arguments.arm)
        input_thread.start()
        with terminal_raw_mode(config.input_kind == "keyboard"):
            runtime.run(stop_event, period_s=arguments.period)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        runtime.shutdown()
        close = getattr(position_reader, "close", None)
        if close is not None:
            close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
