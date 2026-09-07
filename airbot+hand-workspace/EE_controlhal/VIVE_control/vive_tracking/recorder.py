"""CSV writer for raw tracker poses. Does not filter or overwrite raw data."""

from __future__ import annotations

import csv
import logging
import time
from pathlib import Path
from typing import Optional, TextIO, Union

from vive_tracking.types import TrackerPose

LOGGER = logging.getLogger(__name__)

CSV_FIELDS = (
    "sequence_id",
    "logical_name",
    "serial",
    "sensor_timestamp",
    "host_monotonic_ns",
    "x",
    "y",
    "z",
    "qw",
    "qx",
    "qy",
    "qz",
    "tracking_valid",
    "source",
)


class PoseRecorder:
    def __init__(self, output: Union[str, Path]) -> None:
        self._path = Path(output)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._handle: Optional[TextIO] = None
        self._writer: Optional[csv.DictWriter[str]] = None
        self._count = 0
        self._t_start = 0.0

    def open(self) -> None:
        self._handle = self._path.open("w", newline="")
        self._writer = csv.DictWriter(self._handle, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._handle.flush()
        self._count = 0
        self._t_start = time.monotonic()
        LOGGER.info("Recording poses to %s", self._path)

    def write(self, pose: TrackerPose) -> None:
        if self._writer is None or self._handle is None:
            raise RuntimeError("PoseRecorder is not open")
        self._writer.writerow(
            {
                "sequence_id": pose.sequence_id,
                "logical_name": pose.logical_name,
                "serial": pose.serial,
                "sensor_timestamp": "" if pose.sensor_timestamp is None else pose.sensor_timestamp,
                "host_monotonic_ns": pose.host_monotonic_ns,
                "x": pose.position_m.x,
                "y": pose.position_m.y,
                "z": pose.position_m.z,
                "qw": pose.orientation.w,
                "qx": pose.orientation.x,
                "qy": pose.orientation.y,
                "qz": pose.orientation.z,
                "tracking_valid": int(pose.tracking_valid),
                "source": pose.source,
            }
        )
        self._handle.flush()
        self._count += 1

    def close(self) -> None:
        if self._handle is not None:
            self._handle.flush()
            self._handle.close()
            self._handle = None
            self._writer = None
        LOGGER.info("Recorder closed frames=%d path=%s", self._count, self._path)

    @property
    def count(self) -> int:
        return self._count

    @property
    def elapsed_s(self) -> float:
        if self._t_start == 0.0:
            return 0.0
        return time.monotonic() - self._t_start

    @property
    def average_rate_hz(self) -> float:
        elapsed = self.elapsed_s
        if elapsed <= 0.0:
            return 0.0
        return self._count / elapsed

    def __enter__(self) -> PoseRecorder:
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()
