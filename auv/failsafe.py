"""Out-of-process motor failsafe.

The Navigator keeps emitting its last PWM duty cycle after the control process
dies, so a crash, native abort, or hang would leave the thrusters running. The
control process starts this watcher after arming and feeds it a heartbeat over
the watcher's stdin. If the heartbeat stops (hang) or the pipe closes without a
clean-exit byte (crash), the watcher kills the control process if it is still
alive and writes neutral, disabled PWM through a fresh Navigator handle.

The watcher runs in its own session and ignores SIGINT/SIGHUP/SIGTERM, so a
terminal Ctrl-C, SSH disconnect, or service stop cannot take it down before the
control process has shut down.
"""

from __future__ import annotations

import argparse
import os
import select
import signal
import subprocess
import sys
import time
from collections.abc import Callable
from pathlib import Path

from .actuation import NavigatorActuator, NavigatorPwmBackend, PwmBackend

HEARTBEAT = b"."
CLEAN_EXIT = b"q"


def wait_for_trip(fd: int, timeout_s: float) -> str | None:
    """Block until the failsafe must act; return why, or None after a clean exit."""
    while True:
        ready, _, _ = select.select([fd], [], [], timeout_s)
        if not ready:
            return f"no heartbeat for {timeout_s:g} s"
        data = os.read(fd, 4096)
        if not data:
            return "control process exited without shutting down"
        if CLEAN_EXIT in data:
            return None


def force_neutral(
    reason: str,
    *,
    parent_pid: int,
    backend: PwmBackend,
    channels: tuple[int, int, int],
    frequency_hz: float,
    getppid: Callable[[], int] = os.getppid,
    kill: Callable[[int, int], None] = os.kill,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    # Once the control process is gone this watcher is reparented, so a matching
    # parent PID proves the original process is still alive (not a reused PID).
    if getppid() == parent_pid:
        kill(parent_pid, signal.SIGKILL)
        for _ in range(100):
            if getppid() != parent_pid:
                break
            sleep(0.01)
    actuator = NavigatorActuator(backend, channels=channels, frequency_hz=frequency_hz, sleep_fn=sleep)
    actuator.initialize()
    actuator.disarm(reason)


class FailsafeWatchdog:
    """Control-process side: starts the watcher and feeds its heartbeat."""

    def __init__(self, *, channels: tuple[int, int, int], frequency_hz: float, timeout_s: float = 1.0) -> None:
        self.channels = channels
        self.frequency_hz = frequency_hz
        self.timeout_s = timeout_s
        self._process: subprocess.Popen[bytes] | None = None

    def start(self) -> None:
        self._process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "auv.failsafe",
                "--parent-pid",
                str(os.getpid()),
                "--channels",
                ",".join(str(channel) for channel in self.channels),
                "--frequency-hz",
                str(self.frequency_hz),
                "--timeout",
                str(self.timeout_s),
            ],
            stdin=subprocess.PIPE,
            cwd=Path(__file__).resolve().parents[1],
            start_new_session=True,
        )

    def beat(self) -> None:
        self._send(HEARTBEAT)

    def close(self) -> None:
        """Tell the watcher the motors are already neutral and disabled."""
        process, self._process = self._process, None
        if process is None or process.stdin is None:
            return
        try:
            process.stdin.write(CLEAN_EXIT)
            process.stdin.close()
        except (OSError, ValueError):
            pass  # watcher already gone; nothing left to tell it
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()

    def _send(self, data: bytes) -> None:
        if self._process is None or self._process.stdin is None:
            raise RuntimeError("motor failsafe is not running")
        try:
            self._process.stdin.write(data)
            self._process.stdin.flush()
        except (BrokenPipeError, ValueError) as error:
            raise RuntimeError("motor failsafe process exited") from error


def main() -> int:
    parser = argparse.ArgumentParser(description="Navigator motor failsafe watcher (started by `python -m auv --arm`).")
    parser.add_argument("--parent-pid", type=int, required=True)
    parser.add_argument("--channels", required=True)
    parser.add_argument("--frequency-hz", type=float, required=True)
    parser.add_argument("--timeout", type=float, required=True)
    arguments = parser.parse_args()
    channels = tuple(int(value) for value in arguments.channels.split(","))
    if len(channels) != 3:
        parser.error("--channels must contain exactly three values")

    for signum in (signal.SIGINT, signal.SIGHUP, signal.SIGTERM):
        signal.signal(signum, signal.SIG_IGN)
    # Import the native binding now: if it is unavailable the watcher exits at
    # once and the control process stops on its next heartbeat instead of
    # discovering the problem during an emergency.
    backend = NavigatorPwmBackend()

    reason = wait_for_trip(sys.stdin.fileno(), arguments.timeout)
    if reason is None:
        return 0
    print(f"\r\nmotor failsafe: {reason}; forcing neutral PWM", file=sys.stderr, flush=True)
    force_neutral(
        reason,
        parent_pid=arguments.parent_pid,
        backend=backend,
        channels=channels,  # type: ignore[arg-type]
        frequency_hz=arguments.frequency_hz,
    )
    print("motor failsafe: PWM neutral and disabled", file=sys.stderr, flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
