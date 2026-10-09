from __future__ import annotations

import csv
from pathlib import Path

from .models import HeadingSample, MotionCommand, PositionSample


class RunLogger:
    """Append-only telemetry writer. Failure is reported to the safety owner."""

    _FIELDS = (
        "monotonic_s",
        "latitude",
        "longitude",
        "gps_valid",
        "heading_degrees",
        "heading_valid",
        "forward",
        "yaw",
        "vertical",
        "mode",
        "reason",
    )

    def __init__(self, path: Path) -> None:
        self.path = path
        self._file = None
        self._writer = None

    def open(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        existed = self.path.exists() and self.path.stat().st_size > 0
        self._file = self.path.open("a", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self._FIELDS)
        if not existed:
            self._writer.writeheader()
            self._file.flush()

    def record(
        self,
        *,
        now: float,
        position: PositionSample,
        heading: HeadingSample,
        command: MotionCommand,
        mode: str,
        reason: str = "",
    ) -> None:
        if self._writer is None or self._file is None:
            raise RuntimeError("logger is not open")
        self._writer.writerow(
            {
                "monotonic_s": now,
                "latitude": position.latitude,
                "longitude": position.longitude,
                "gps_valid": position.valid,
                "heading_degrees": heading.degrees,
                "heading_valid": heading.valid,
                "forward": command.forward,
                "yaw": command.yaw,
                "vertical": command.vertical,
                "mode": mode,
                "reason": reason,
            }
        )
        self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
