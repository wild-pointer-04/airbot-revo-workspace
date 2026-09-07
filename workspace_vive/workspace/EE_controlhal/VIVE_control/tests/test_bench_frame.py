"""Workbench-frame validity: missing landmarks, baseline, no last_valid reuse."""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from vive_tracking.bench_frame import compute_bench_hand, parse_up_hint
from vive_tracking.timestamp import host_monotonic_ns, mark_stale_if_old
from vive_tracking.tracker_manager import TrackerManager, create_backend
from vive_tracking.types import IDENTITY_QUATERNION, TrackerPose, Vec3

ROOT = Path(__file__).resolve().parents[1]


def _pose(
    serial: str,
    x: float,
    y: float,
    z: float,
    *,
    valid: bool = True,
    seq: int = 1,
) -> TrackerPose:
    return TrackerPose(
        serial=serial,
        logical_name=serial,
        position_m=Vec3(x, y, z),
        orientation=IDENTITY_QUATERNION,
        sensor_timestamp=None,
        host_monotonic_ns=host_monotonic_ns(),
        tracking_valid=valid,
        sequence_id=seq,
        source="test",
    )


def test_compute_bench_hand_ok() -> None:
    hand = _pose("H", 0.40, 0.00, 0.50)
    origin = _pose("A", 0.10, -0.20, 0.02)
    axis = _pose("B", 0.60, -0.20, 0.02)
    result = compute_bench_hand(
        hand,
        origin,
        axis,
        up_hint=parse_up_hint([0.0, 0.0, 1.0]),
        baseline_m=0.50,
        baseline_tol_m=0.02,
    )
    assert result.reason == "ok"
    assert result.tracking_valid is True
    assert result.pose is not None
    assert result.pose.position_m.x == pytest.approx(0.30, abs=1e-8)
    assert result.pose.position_m.y == pytest.approx(0.20, abs=1e-8)
    assert result.pose.position_m.z == pytest.approx(0.48, abs=1e-8)
    assert result.baseline_m == pytest.approx(0.50, abs=1e-8)


def test_invalid_bench_does_not_return_last_hand() -> None:
    hand = _pose("H", 0.40, 0.00, 0.50)
    origin = _pose("A", 0.10, -0.20, 0.02, valid=False)
    axis = _pose("B", 0.60, -0.20, 0.02)
    result = compute_bench_hand(
        hand,
        origin,
        axis,
        up_hint=Vec3(0.0, 0.0, 1.0),
        baseline_m=0.50,
        baseline_tol_m=0.02,
    )
    assert result.tracking_valid is False
    assert result.pose is None
    assert result.hand_valid is True
    assert result.bench_valid is False
    assert result.reason == "invalid_bench"


def test_baseline_mismatch() -> None:
    hand = _pose("H", 0.40, 0.00, 0.50)
    origin = _pose("A", 0.10, -0.20, 0.02)
    axis = _pose("B", 0.80, -0.20, 0.02)
    result = compute_bench_hand(
        hand,
        origin,
        axis,
        up_hint=Vec3(0.0, 0.0, 1.0),
        baseline_m=0.50,
        baseline_tol_m=0.02,
    )
    assert result.reason == "baseline"
    assert result.tracking_valid is False
    assert result.pose is None


def test_missing_hand() -> None:
    origin = _pose("A", 0.10, -0.20, 0.02)
    axis = _pose("B", 0.60, -0.20, 0.02)
    result = compute_bench_hand(
        None,
        origin,
        axis,
        up_hint=Vec3(0.0, 0.0, 1.0),
        baseline_m=None,
        baseline_tol_m=0.02,
    )
    assert result.reason == "missing_hand"
    assert result.pose is None


def test_mark_stale_if_old() -> None:
    pose = _pose("H", 0.0, 0.0, 0.0)
    now = pose.host_monotonic_ns + 400_000_000
    stale = mark_stale_if_old(pose, 0.25, now_ns=now)
    assert stale.tracking_valid is False
    assert stale.position_m.x == pose.position_m.x
    fresh = mark_stale_if_old(pose, 0.25, now_ns=pose.host_monotonic_ns + 10_000_000)
    assert fresh.tracking_valid is True


def test_inline_three_trackers_and_static_landmarks() -> None:
    cfg = {
        "backend": "mock",
        "mock": {
            "mode": "static",
            "update_rate_hz": 80,
            "static": {"x": 0.40, "y": 0.00, "z": 0.50},
        },
        "queue": {"max_size": 200},
        "trackers": [
            {"logical_name": "dexterous_hand", "serial": "MOCK-HAND-001", "enabled": True},
            {
                "logical_name": "bench_origin",
                "serial": "MOCK-BENCH-A",
                "role": "bench_origin",
                "enabled": True,
            },
            {
                "logical_name": "bench_x",
                "serial": "MOCK-BENCH-B",
                "role": "bench_x",
                "enabled": True,
            },
        ],
    }
    manager = TrackerManager(create_backend(cfg), cfg)
    manager.start()
    try:
        origin = axis = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            origin = manager.get_pose("bench_origin")
            axis = manager.get_pose("bench_x")
            if origin is not None and axis is not None:
                break
            time.sleep(0.02)
        assert origin is not None
        assert axis is not None
        time.sleep(0.05)
        origin2 = manager.get_pose("bench_origin")
        assert origin2 is not None
        assert origin2.position_m.x == pytest.approx(origin.position_m.x, abs=1e-8)
    finally:
        manager.stop()
