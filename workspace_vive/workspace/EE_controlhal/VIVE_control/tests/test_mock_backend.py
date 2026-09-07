"""Mock backend modes, dropout, and multi-tracker emission."""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from vive_tracking.mock_backend import KNOWN_MOCK_SERIALS, MockBackend
from vive_tracking.tracker_manager import TrackerManager

ROOT = Path(__file__).resolve().parents[1]


def _cfg(mode: str, extra_mock: dict | None = None, trackers: list | None = None) -> dict:
    mock = {
        "mode": mode,
        "update_rate_hz": 80,
        "playback_speed": 1.0,
        "dropout": {"enabled": False, "duration_ms": 80, "period_s": 0.4},
    }
    if extra_mock:
        mock.update(extra_mock)
    return {
        "backend": "mock",
        "mock": mock,
        "queue": {"max_size": 200},
        "trackers": trackers
        or [
            {
                "logical_name": "dexterous_hand",
                "serial": "MOCK-HAND-001",
                "enabled": True,
            }
        ],
    }


def _collect(backend: MockBackend, n: int, timeout: float = 2.0) -> list:
    backend.start()
    poses = []
    deadline = time.monotonic() + timeout
    try:
        while len(poses) < n and time.monotonic() < deadline:
            pose = backend.wait_for_pose(timeout=0.2)
            if pose is not None:
                poses.append(pose)
    finally:
        backend.stop()
    return poses


def test_known_mock_serials_include_future_trackers() -> None:
    assert "MOCK-HAND-001" in KNOWN_MOCK_SERIALS
    assert "MOCK-BENCH-A" in KNOWN_MOCK_SERIALS
    assert "MOCK-BENCH-B" in KNOWN_MOCK_SERIALS
    assert "MOCK-LEFT-001" in KNOWN_MOCK_SERIALS
    assert "MOCK-RIGHT-001" in KNOWN_MOCK_SERIALS
    assert "MOCK-HEAD-001" in KNOWN_MOCK_SERIALS


def test_static_pose() -> None:
    backend = MockBackend(_cfg("static"), _cfg("static")["trackers"])
    backend.initialize()
    poses = _collect(backend, 20)
    assert len(poses) >= 20
    pose = poses[-1]
    assert pose.serial == "MOCK-HAND-001"
    assert pose.source == "mock"
    assert pose.sensor_timestamp is None
    assert pose.host_monotonic_ns > 0
    assert pytest.approx(pose.position_m.x, abs=1e-9) == 0.40
    assert pytest.approx(pose.orientation.w, abs=1e-9) == 1.0
    assert pose.tracking_valid is True
    assert poses[-1].sequence_id > poses[0].sequence_id


def test_sinusoidal_is_smooth() -> None:
    backend = MockBackend(_cfg("sinusoidal"), _cfg("sinusoidal")["trackers"])
    backend.initialize()
    poses = _collect(backend, 40)
    xs = [p.position_m.x for p in poses]
    assert max(xs) - min(xs) > 0.001
    deltas = [abs(xs[i] - xs[i - 1]) for i in range(1, len(xs))]
    assert max(deltas) < 0.05
    dots = []
    for i in range(1, len(poses)):
        a = poses[i - 1].orientation
        b = poses[i].orientation
        dots.append(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z)
    assert min(dots) > 0.9


def test_trajectory_playback(tmp_path: Path) -> None:
    csv_path = tmp_path / "traj.csv"
    csv_path.write_text(
        "timestamp,x,y,z,qw,qx,qy,qz\n"
        "0.0,0.0,0.0,0.0,1,0,0,0\n"
        "1.0,0.2,0.0,0.0,1,0,0,0\n"
    )
    cfg = _cfg("trajectory", {"trajectory_csv": str(csv_path), "update_rate_hz": 50})
    backend = MockBackend(cfg, cfg["trackers"])
    backend.initialize()
    poses = _collect(backend, 15)
    assert len(poses) >= 15
    xs = [p.position_m.x for p in poses]
    assert min(xs) >= -0.001
    assert max(xs) <= 0.201


def test_dropout_marks_invalid() -> None:
    cfg = _cfg(
        "dropout",
        {"dropout": {"enabled": True, "duration_ms": 100, "period_s": 0.3}, "update_rate_hz": 100},
    )
    backend = MockBackend(cfg, cfg["trackers"])
    backend.initialize()
    poses = _collect(backend, 80, timeout=3.0)
    flags = [p.tracking_valid for p in poses]
    assert True in flags
    assert False in flags
    latest = backend.get_latest_pose("MOCK-HAND-001")
    assert latest is not None


def test_multiple_trackers() -> None:
    trackers = [
        {"logical_name": "dexterous_hand", "serial": "MOCK-HAND-001", "enabled": True},
        {"logical_name": "left_wrist", "serial": "MOCK-LEFT-001", "enabled": True},
    ]
    cfg = _cfg("static", trackers=trackers)
    manager = TrackerManager(MockBackend(cfg, trackers), cfg)
    manager.start()
    try:
        seen = set()
        deadline = time.monotonic() + 2.0
        while len(seen) < 2 and time.monotonic() < deadline:
            pose = manager.wait_for_pose(timeout=0.2)
            if pose is not None:
                seen.add(pose.serial)
        assert seen == {"MOCK-HAND-001", "MOCK-LEFT-001"}
        hand = manager.get_pose("dexterous_hand")
        assert hand is not None
        assert hand.logical_name == "dexterous_hand"
    finally:
        manager.stop()


def test_dropout_can_target_one_serial() -> None:
    trackers = [
        {
            "logical_name": "dexterous_hand",
            "serial": "MOCK-HAND-001",
            "role": "hand",
            "enabled": True,
        },
        {
            "logical_name": "bench_origin",
            "serial": "MOCK-BENCH-A",
            "role": "bench_origin",
            "enabled": True,
        },
    ]
    cfg = _cfg(
        "sinusoidal",
        extra_mock={
            "dropout": {
                "enabled": True,
                "duration_ms": 500,
                "period_s": 0.5,
                "serials": ["MOCK-BENCH-A"],
            },
            "update_rate_hz": 100,
        },
        trackers=trackers,
    )
    backend = MockBackend(cfg, trackers)
    backend.initialize()
    poses = _collect(backend, 40, timeout=2.0)
    by_serial: dict[str, list[bool]] = {"MOCK-HAND-001": [], "MOCK-BENCH-A": []}
    for pose in poses:
        by_serial[pose.serial].append(pose.tracking_valid)
    assert by_serial["MOCK-HAND-001"]
    assert by_serial["MOCK-BENCH-A"]
    assert all(by_serial["MOCK-HAND-001"])
    assert not any(by_serial["MOCK-BENCH-A"])
