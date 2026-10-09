#!/usr/bin/env python3
"""Fit and write an inspectable magnetometer calibration from capture_magnetometer CSV output."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from auv.calibration import fit_axis_aligned_magnetometer


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("samples", type=Path)
    parser.add_argument("--output", type=Path, default=Path("Data/calibration/magnetometer.json"))
    arguments = parser.parse_args()
    with arguments.samples.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        samples = [(float(row["x"]), float(row["y"]), float(row["z"])) for row in reader]
    calibration = fit_axis_aligned_magnetometer(samples)
    calibration.save(arguments.output)
    print(f"wrote {arguments.output} from {len(samples)} samples")
    print(f"offset={calibration.offset}")
    print(f"soft_iron={calibration.soft_iron}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
