"""Live pose quality stats. Thresholds come from config, not hardcoded."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

from vive_tracking.pose_utils import quaternion_angle, quaternion_dot
from vive_tracking.timestamp import monotonic_ns_to_seconds
from vive_tracking.types import TrackerPose

_RAD2DEG = 180.0 / math.pi


@dataclass
class DiagnosticsReport:
    duration_s: float = 0.0
    total_samples: int = 0
    valid_samples: int = 0
    invalid_samples: int = 0
    average_rate_hz: float = 0.0
    min_interval_ms: Optional[float] = None
    max_interval_ms: Optional[float] = None
    position_mean: tuple[float, float, float] = (0.0, 0.0, 0.0)
    position_std_x: float = 0.0
    position_std_y: float = 0.0
    position_std_z: float = 0.0
    position_range: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_jitter_deg: float = 0.0
    dropout_count: int = 0
    longest_dropout_ms: float = 0.0
    queue_drop_count: int = 0
    timestamp_backward_count: int = 0
    verdict: str = "FAIL"
    notes: list[str] = field(default_factory=list)

    def format(self) -> str:
        mean = self.position_mean
        rng = self.position_range
        lines = [
            f"duration: {self.duration_s:.3f} s",
            f"total_samples: {self.total_samples}",
            f"valid_samples: {self.valid_samples}",
            f"invalid_samples: {self.invalid_samples}",
            f"average_rate_hz: {self.average_rate_hz:.2f}",
            f"min_interval_ms: {_fmt(self.min_interval_ms)}",
            f"max_interval_ms: {_fmt(self.max_interval_ms)}",
            f"position_mean: x={mean[0]:.6f} y={mean[1]:.6f} z={mean[2]:.6f}",
            f"position_std_x: {self.position_std_x:.6f}",
            f"position_std_y: {self.position_std_y:.6f}",
            f"position_std_z: {self.position_std_z:.6f}",
            f"position_range: x={rng[0]:.6f} y={rng[1]:.6f} z={rng[2]:.6f}",
            f"orientation_jitter_deg: {self.orientation_jitter_deg:.4f}",
            f"dropout_count: {self.dropout_count}",
            f"longest_dropout_ms: {self.longest_dropout_ms:.2f}",
            f"queue_drop_count: {self.queue_drop_count}",
            f"timestamp_backward_count: {self.timestamp_backward_count}",
            f"verdict: {self.verdict}",
        ]
        if self.notes:
            lines.append("notes:")
            lines.extend(f"  - {note}" for note in self.notes)
        return "\n".join(lines)


class PoseDiagnostics:
    def __init__(self, thresholds: dict[str, Any]) -> None:
        self._thresholds = thresholds
        self._poses: list[TrackerPose] = []
        self._queue_drop_count = 0

    def add(self, pose: TrackerPose) -> None:
        self._poses.append(pose)

    def set_queue_drop_count(self, count: int) -> None:
        self._queue_drop_count = count

    def summarize(self) -> DiagnosticsReport:
        report = DiagnosticsReport(queue_drop_count=self._queue_drop_count)
        if not self._poses:
            report.notes.append("No samples collected")
            report.verdict = "FAIL"
            return report

        times_ns = [p.host_monotonic_ns for p in self._poses]
        report.duration_s = monotonic_ns_to_seconds(times_ns[-1] - times_ns[0])
        report.total_samples = len(self._poses)
        report.valid_samples = sum(1 for p in self._poses if p.tracking_valid)
        report.invalid_samples = report.total_samples - report.valid_samples
        if report.duration_s > 0.0:
            report.average_rate_hz = (report.total_samples - 1) / report.duration_s

        intervals_ms = [
            monotonic_ns_to_seconds(times_ns[i] - times_ns[i - 1]) * 1000.0
            for i in range(1, len(times_ns))
        ]
        if intervals_ms:
            report.min_interval_ms = min(intervals_ms)
            report.max_interval_ms = max(intervals_ms)
        report.timestamp_backward_count = sum(1 for dt in intervals_ms if dt < 0.0)

        valid = [p for p in self._poses if p.tracking_valid]
        if valid:
            xyz = np.array([p.position_m.as_tuple() for p in valid], dtype=np.float64)
            report.position_mean = (
                float(xyz[:, 0].mean()),
                float(xyz[:, 1].mean()),
                float(xyz[:, 2].mean()),
            )
            std = xyz.std(axis=0)
            report.position_std_x = float(std[0])
            report.position_std_y = float(std[1])
            report.position_std_z = float(std[2])
            report.position_range = (
                float(xyz[:, 0].max() - xyz[:, 0].min()),
                float(xyz[:, 1].max() - xyz[:, 1].min()),
                float(xyz[:, 2].max() - xyz[:, 2].min()),
            )
            report.orientation_jitter_deg = _orientation_jitter_deg(valid)

        report.dropout_count, report.longest_dropout_ms = _dropout_stats(self._poses)
        report.verdict, report.notes = _verdict(report, self._thresholds)
        return report


def _orientation_jitter_deg(valid: list[TrackerPose]) -> float:
    if len(valid) < 2:
        return 0.0
    ref = valid[0].orientation
    for pose in valid[1:]:
        if quaternion_dot(pose.orientation, ref) > 0.0:
            ref = pose.orientation
            break
    angles = [quaternion_angle(ref, pose.orientation) * _RAD2DEG for pose in valid]
    return float(np.std(np.array(angles, dtype=np.float64)))


def _dropout_stats(poses: list[TrackerPose]) -> tuple[int, float]:
    dropouts = 0
    longest_ms = 0.0
    in_drop = False
    drop_start_ns = 0
    prev_valid = True
    for pose in poses:
        if prev_valid and not pose.tracking_valid:
            dropouts += 1
            in_drop = True
            drop_start_ns = pose.host_monotonic_ns
        if in_drop and pose.tracking_valid:
            longest_ms = max(
                longest_ms,
                monotonic_ns_to_seconds(pose.host_monotonic_ns - drop_start_ns) * 1000.0,
            )
            in_drop = False
        prev_valid = pose.tracking_valid
    if in_drop and poses:
        longest_ms = max(
            longest_ms,
            monotonic_ns_to_seconds(poses[-1].host_monotonic_ns - drop_start_ns) * 1000.0,
        )
    return dropouts, longest_ms


def _verdict(report: DiagnosticsReport, thresholds: dict[str, Any]) -> tuple[str, list[str]]:
    notes: list[str] = []
    fail = False
    warn = False
    min_rate = float(thresholds.get("min_rate_hz", 80.0))
    expected = float(thresholds.get("expected_rate_hz", 100.0))
    max_dropout_ms = float(thresholds.get("max_dropout_ms", 100.0))
    max_std = thresholds.get("max_position_std_m")
    max_jitter = thresholds.get("max_orientation_jitter_deg")
    check_static = bool(thresholds.get("check_static_stability", False))
    max_backward = int(thresholds.get("max_timestamp_backward", 0))

    if report.total_samples < 2:
        return "FAIL", ["Need at least 2 samples"]
    if report.average_rate_hz < min_rate:
        fail = True
        notes.append(f"average_rate_hz {report.average_rate_hz:.2f} < min_rate_hz {min_rate}")
    elif report.average_rate_hz < 0.9 * expected:
        warn = True
        notes.append(
            f"average_rate_hz {report.average_rate_hz:.2f} below expected {expected}"
        )
    if report.timestamp_backward_count > max_backward:
        fail = True
        notes.append(f"timestamp_backward_count={report.timestamp_backward_count}")
    if report.longest_dropout_ms > max_dropout_ms and report.dropout_count > 0:
        if check_static:
            fail = True
        else:
            warn = True
        notes.append(
            f"longest_dropout_ms {report.longest_dropout_ms:.2f} > max_dropout_ms {max_dropout_ms}"
        )
    if check_static and max_std is not None:
        limit = float(max_std)
        peak_std = max(report.position_std_x, report.position_std_y, report.position_std_z)
        if peak_std > limit:
            fail = True
            notes.append(f"position std {peak_std:.6f} m > {limit}")
    if check_static and max_jitter is not None:
        if report.orientation_jitter_deg > float(max_jitter):
            fail = True
            notes.append(
                f"orientation_jitter_deg {report.orientation_jitter_deg:.4f} > {max_jitter}"
            )
    if report.queue_drop_count > 0:
        warn = True
        notes.append(f"queue_drop_count={report.queue_drop_count}")

    if fail:
        return "FAIL", notes
    if warn:
        return "WARNING", notes
    return "PASS", notes or ["All configured checks passed"]


def _fmt(value: Optional[float]) -> str:
    if value is None:
        return "n/a"
    return f"{value:.3f}"
