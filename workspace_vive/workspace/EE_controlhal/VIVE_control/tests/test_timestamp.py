"""Host monotonic timestamps must not be wall-clock and must not go backward."""

from __future__ import annotations

import time

from vive_tracking.mock_backend import MockBackend
from vive_tracking.timestamp import host_monotonic_ns, monotonic_ns_to_seconds


def test_host_monotonic_ns_matches_time_monotonic_ns() -> None:
    a = time.monotonic_ns()
    b = host_monotonic_ns()
    c = time.monotonic_ns()
    assert a <= b <= c


def test_host_monotonic_is_not_wall_clock() -> None:
    host = host_monotonic_ns()
    wall_ns = int(time.time() * 1_000_000_000)
    assert abs(host - wall_ns) > 1_000_000_000


def test_mock_sensor_timestamp_is_none() -> None:
    cfg = {
        "mock": {"mode": "static", "update_rate_hz": 50},
        "queue": {"max_size": 50},
        "trackers": [{"logical_name": "dexterous_hand", "serial": "MOCK-HAND-001", "enabled": True}],
    }
    backend = MockBackend(cfg, cfg["trackers"])
    backend.initialize()
    backend.start()
    try:
        pose = backend.wait_for_pose(timeout=1.0)
        assert pose is not None
        assert pose.sensor_timestamp is None
        assert pose.host_monotonic_ns > 0
    finally:
        backend.stop()


def test_mock_host_timestamps_increase() -> None:
    cfg = {
        "mock": {"mode": "static", "update_rate_hz": 80},
        "queue": {"max_size": 200},
        "trackers": [{"logical_name": "dexterous_hand", "serial": "MOCK-HAND-001", "enabled": True}],
    }
    backend = MockBackend(cfg, cfg["trackers"])
    backend.initialize()
    backend.start()
    stamps = []
    try:
        while len(stamps) < 30:
            pose = backend.wait_for_pose(timeout=0.5)
            if pose is not None:
                stamps.append(pose.host_monotonic_ns)
    finally:
        backend.stop()
    assert stamps == sorted(stamps)
    assert monotonic_ns_to_seconds(stamps[-1] - stamps[0]) > 0.0
