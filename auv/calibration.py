from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True)
class MagnetometerCalibration:
    offset: Vector3 = (0.0, 0.0, 0.0)
    soft_iron: Matrix3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    def apply(self, raw: Vector3) -> Vector3:
        centered = tuple(value - offset for value, offset in zip(raw, self.offset, strict=True))
        return tuple(sum(row[index] * centered[index] for index in range(3)) for row in self.soft_iron)  # type: ignore[return-value]

    @classmethod
    def load(cls, path: Path) -> "MagnetometerCalibration":
        raw = json.loads(path.read_text(encoding="utf-8"))
        offset = tuple(float(value) for value in raw["offset"])
        soft_iron = tuple(tuple(float(value) for value in row) for row in raw["soft_iron"])
        if len(offset) != 3 or len(soft_iron) != 3 or any(len(row) != 3 for row in soft_iron):
            raise ValueError("magnetometer calibration requires offset[3] and soft_iron[3][3]")
        return cls(offset=offset, soft_iron=soft_iron)  # type: ignore[arg-type]

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps({"offset": self.offset, "soft_iron": self.soft_iron}, indent=2) + "\n",
            encoding="utf-8",
        )


def fit_axis_aligned_magnetometer(samples: Iterable[Vector3]) -> MagnetometerCalibration:
    """Fit hard-iron offset and per-axis scale from a full-orientation sample set.

    This is intentionally not an unconstrained ellipsoid fit: the constrained fit
    is inspectable and does not invent a rotation for an unknown vehicle mounting.
    """
    points = list(samples)
    if len(points) < 20:
        raise ValueError("at least 20 orientation-diverse magnetometer samples are required")
    minima = tuple(min(point[index] for point in points) for index in range(3))
    maxima = tuple(max(point[index] for point in points) for index in range(3))
    radii = tuple((maxima[index] - minima[index]) / 2.0 for index in range(3))
    if any(radius <= 0.0 for radius in radii):
        raise ValueError("magnetometer samples do not span all three axes")
    average_radius = sum(radii) / 3.0
    return MagnetometerCalibration(
        offset=tuple((maxima[index] + minima[index]) / 2.0 for index in range(3)),  # type: ignore[arg-type]
        soft_iron=tuple(
            tuple((average_radius / radii[row] if row == column else 0.0) for column in range(3))
            for row in range(3)
        ),  # type: ignore[arg-type]
    )
