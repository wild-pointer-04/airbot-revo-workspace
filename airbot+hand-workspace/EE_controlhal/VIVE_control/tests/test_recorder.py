"""CSV recorder writes header, rows, and survives close."""

from __future__ import annotations

import csv
from pathlib import Path

from vive_tracking.recorder import CSV_FIELDS, PoseRecorder
from vive_tracking.types import IDENTITY_QUATERNION, TrackerPose, Vec3


def _pose(seq: int) -> TrackerPose:
    return TrackerPose(
        serial="MOCK-HAND-001",
        logical_name="dexterous_hand",
        position_m=Vec3(0.4, 0.0, 0.5),
        orientation=IDENTITY_QUATERNION,
        sensor_timestamp=None,
        host_monotonic_ns=1_000_000 + seq,
        tracking_valid=True,
        sequence_id=seq,
        source="mock",
    )


def test_recorder_writes_header_and_rows(tmp_path: Path) -> None:
    path = tmp_path / "out.csv"
    with PoseRecorder(path) as rec:
        rec.write(_pose(1))
        rec.write(_pose(2))
        assert rec.count == 2
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert list(rows[0].keys()) == list(CSV_FIELDS)
    assert len(rows) == 2
    assert rows[0]["serial"] == "MOCK-HAND-001"
    assert rows[0]["qw"] == "1.0"
    assert rows[0]["sensor_timestamp"] == ""
    assert rows[1]["sequence_id"] == "2"
