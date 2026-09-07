"""Workbench frame from two table landmarks + one moving tracker.

TrackerManager still emits raw T_world_*. This module is the consumer that
builds T_bench_hand. It never invents a pose from last_valid.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from vive_tracking.pose_utils import make_bench_frame, world_to_bench
from vive_tracking.types import Pose6DoF, TrackerPose, Vec3

ROLE_HAND = "hand"
ROLE_BENCH_ORIGIN = "bench_origin"
ROLE_BENCH_X = "bench_x"


@dataclass(frozen=True, slots=True)
class BenchHandResult:
    pose: Optional[Pose6DoF]
    tracking_valid: bool
    hand_valid: bool
    bench_valid: bool
    baseline_m: Optional[float]
    reason: str


def parse_up_hint(value: Any) -> Vec3:
    if isinstance(value, Vec3):
        return value
    if isinstance(value, (list, tuple)) and len(value) == 3:
        return Vec3(float(value[0]), float(value[1]), float(value[2]))
    return Vec3(0.0, 0.0, 1.0)


def role_of(tracker: dict[str, Any]) -> str:
    return str(tracker.get("role") or ROLE_HAND).lower()


def logical_name_for_role(config: dict[str, Any], role: str) -> Optional[str]:
    for tracker in config.get("trackers") or []:
        if not tracker.get("enabled", True):
            continue
        if role_of(tracker) == role:
            return str(tracker["logical_name"])
    return None


def compute_bench_hand(
    hand: Optional[TrackerPose],
    origin: Optional[TrackerPose],
    axis: Optional[TrackerPose],
    *,
    up_hint: Vec3,
    baseline_m: Optional[float],
    baseline_tol_m: float,
) -> BenchHandResult:
    """Return T_bench_hand or an invalid result. Never substitutes last_valid."""
    hand_ok = hand is not None and hand.tracking_valid
    origin_ok = origin is not None and origin.tracking_valid
    axis_ok = axis is not None and axis.tracking_valid
    bench_ok = origin_ok and axis_ok

    if hand is None:
        return BenchHandResult(None, False, False, bench_ok, None, "missing_hand")
    if origin is None or axis is None:
        return BenchHandResult(None, False, hand_ok, False, None, "missing_bench")
    if not hand.tracking_valid:
        measured = _baseline(origin, axis)
        return BenchHandResult(None, False, False, bench_ok, measured, "invalid_hand")
    if not origin.tracking_valid or not axis.tracking_valid:
        measured = _baseline(origin, axis) if origin is not None and axis is not None else None
        return BenchHandResult(None, False, True, False, measured, "invalid_bench")

    measured = _baseline(origin, axis)
    if baseline_m is not None and measured is not None:
        if abs(measured - float(baseline_m)) > float(baseline_tol_m):
            return BenchHandResult(None, False, True, False, measured, "baseline")

    try:
        bench = make_bench_frame(origin.position_m, axis.position_m, up_hint)
    except ValueError:
        return BenchHandResult(None, False, True, False, measured, "degenerate")

    return BenchHandResult(
        pose=world_to_bench(hand, bench),
        tracking_valid=True,
        hand_valid=True,
        bench_valid=True,
        baseline_m=measured,
        reason="ok",
    )


def _baseline(origin: TrackerPose, axis: TrackerPose) -> float:
    dx = origin.position_m.x - axis.position_m.x
    dy = origin.position_m.y - axis.position_m.y
    dz = origin.position_m.z - axis.position_m.z
    return float((dx * dx + dy * dy + dz * dz) ** 0.5)
