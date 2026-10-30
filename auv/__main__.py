from __future__ import annotations

import argparse
import shutil
import signal
import sys
import threading
from contextlib import contextmanager
from datetime import UTC, datetime
from pathlib import Path
from time import monotonic

from .actuation import NavigatorActuator, NavigatorPwmBackend
from .config import RuntimeConfig
from .failsafe import FailsafeWatchdog
from .hardware import SerialGpsReader, load_heading_reader
from .input import KEYBOARD_HELP, KeyboardInputWorker, SharedManualInput, XboxInputWorker
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


class StatusLine:
    """Rewrites one terminal line with the runtime's latest decision, at most `rate_hz`."""

    def __init__(self, runtime: ControlRuntime, keyboard: KeyboardInputWorker | None, *, rate_hz: float = 5.0) -> None:
        self._runtime = runtime
        self._keyboard = keyboard
        self._period_s = 1.0 / rate_hz
        self._next_at = 0.0
        self._enabled = sys.stdout.isatty()

    def __call__(self) -> None:
        status = self._runtime.status
        now = monotonic()
        if not self._enabled or status is None or now < self._next_at:
            return
        self._next_at = now + self._period_s
        command = status.command
        parts = [
            status.mode.name,
            f"fwd {command.forward:+.1f} turn {command.yaw:+.1f} vert {command.vertical:+.1f}",
        ]
        if self._keyboard is not None:
            parts.append(f"speed {self._keyboard.level}/5")
        parts.append(status.actuator.name)
        if status.reason:
            parts.append(status.reason)
        line = " | ".join(parts)[: shutil.get_terminal_size().columns - 1]
        self._write("\r\x1b[K" + line)

    def finish(self) -> None:
        self._write("\n")

    def _write(self, text: str) -> None:
        if not self._enabled:
            return
        try:
            sys.stdout.write(text)
            sys.stdout.flush()
        except OSError:
            self._enabled = False  # terminal went away (e.g. SSH dropped); keep controlling


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
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda *_: stop_event.set())
    manual = SharedManualInput()
    keyboard = KeyboardInputWorker(manual, stop_event) if config.input_kind == "keyboard" else None
    worker = keyboard if keyboard is not None else XboxInputWorker(manual, stop_event) if config.input_kind == "xbox" else None
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
        navigation=NavigationController(
            config.waypoints, heading_source=config.heading_source, limits=config.autonomy_limits
        ),
        safety=SafetyController(),
        logger=RunLogger(config.log_directory / log_name),
    )
    failsafe = FailsafeWatchdog(channels=config.channels, frequency_hz=config.frequency_hz) if arguments.arm else None
    status_line = StatusLine(runtime, keyboard)

    def after_step() -> None:
        if failsafe is not None:
            failsafe.beat()
        status_line()

    if not config.gps_device:
        print("GPS not configured; autonomy is unavailable.", file=sys.stderr)
    if config.heading_source == "amg" and not config.heading_calibration:
        print("AMG heading not configured; autonomy is unavailable.", file=sys.stderr)
    if keyboard is not None:
        print(KEYBOARD_HELP)

    try:
        runtime.initialize(arm=arguments.arm)
        if failsafe is not None:
            failsafe.start()
        input_thread.start()
        with terminal_raw_mode(config.input_kind == "keyboard"):
            runtime.run(stop_event, period_s=arguments.period, after_step=after_step)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        try:
            runtime.shutdown()
            # Only a successful neutral shutdown releases the watcher; otherwise it trips when we exit.
            if failsafe is not None:
                failsafe.close()
        finally:
            close = getattr(position_reader, "close", None)
            if close is not None:
                close()
            status_line.finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
