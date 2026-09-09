from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
for path in (str(ROOT), str(MIDDLE)):
    if path not in sys.path:
        sys.path.insert(0, path)

from scripts import calibrate_body_mount as calibration
from scripts.calibrate_body_mount import (
    _read_samples,
    _sample,
    _solve,
    _wait_for_trackers,
    _write_config,
    _write_sample,
)
from processing.vr_normalize import OPENXR_TO_TELEOP_R
from vive_tracking.types import IDENTITY_QUATERNION, TrackerPose, Vec3


def _rotation_z(angle: float) -> np.ndarray:
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _rotation_y(angle: float) -> np.ndarray:
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array(
        [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]],
        dtype=np.float64,
    )


def _recorded_for_mount(tracker_to_body: np.ndarray) -> dict[str, dict]:
    r_world_body = _rotation_z(0.55)
    r_world_tracker = r_world_body @ tracker_to_body
    baseline = np.array([0.32, -0.18, 0.74])
    recorded = {
        "baseline": {
            "count": 90,
            "relative_position": baseline,
            "body_rotation": r_world_tracker,
            "body_jitter_rad": 0.01,
        }
    }
    for index, axis in enumerate(("forward", "left", "up")):
        recorded[axis] = {
            "count": 90,
            "relative_position": baseline + 0.35 * r_world_body[:, index],
            "body_rotation": r_world_tracker,
            "body_jitter_rad": 0.01,
        }
    return recorded


def test_body_mount_solver_recovers_a_full_rotation_matrix() -> None:
    tracker_to_body = _rotation_z(-0.37) @ _rotation_y(0.18)

    solved, stats = _solve(_recorded_for_mount(tracker_to_body))

    np.testing.assert_allclose(solved, tracker_to_body, atol=1e-9)
    assert stats["ok"]
    assert stats["checks"]["world_up"]
    assert stats["checks"]["runtime_axes"]
    for index, axis in enumerate(calibration.AXES):
        np.testing.assert_allclose(
            stats["runtime_deltas"][axis], 0.35 * np.eye(3)[:, index], atol=1e-9
        )


def test_solver_rejects_wrong_world_up_even_with_perfect_orthogonal_axes() -> None:
    recorded = {
        "baseline": {
            "relative_position": np.zeros(3),
            "body_rotation": np.eye(3),
            "body_jitter_rad": 0.0,
        }
    }
    for index, axis in enumerate(calibration.AXES):
        recorded[axis] = {
            "relative_position": 0.3 * OPENXR_TO_TELEOP_R[:, index],
            "body_rotation": np.eye(3),
            "body_jitter_rad": 0.0,
        }

    _mount, stats = _solve(recorded)

    assert stats["checks"]["orthogonal"]
    assert stats["checks"]["right_handed"]
    assert stats["checks"]["axis_alignment"]
    assert not stats["checks"]["world_up"]
    assert not stats["checks"]["runtime_axes"]
    assert not stats["ok"]
    np.testing.assert_allclose(stats["runtime_deltas"]["up"], [0.0, -0.3, 0.0])


def test_solver_checks_actual_head_base_for_each_motion_pose() -> None:
    recorded = _recorded_for_mount(np.eye(3))
    baseline = np.array([2.0, 0.0, 0.7])
    previous = recorded["baseline"]["relative_position"].copy()
    for sample in recorded.values():
        sample["relative_position"] = sample["relative_position"] - previous + baseline
    recorded["forward"]["body_rotation"] = (
        _rotation_z(np.deg2rad(11.0)) @ recorded["forward"]["body_rotation"]
    )

    _mount, stats = _solve(recorded)

    assert stats["checks"]["world_up"]
    assert stats["checks"]["body_still"]
    assert stats["checks"]["orthogonal"]
    assert not stats["checks"]["runtime_axes"]
    assert not stats["ok"]


def test_body_mount_solver_rejects_a_left_handed_motion_set() -> None:
    tracker_to_body = _rotation_z(0.24)
    recorded = _recorded_for_mount(tracker_to_body)
    baseline = recorded["baseline"]["relative_position"]
    # The user labelled this as "up", but performed the opposite movement.
    recorded["up"]["relative_position"] = baseline - (
        recorded["up"]["relative_position"] - baseline
    )

    _solved, stats = _solve(recorded)

    assert not stats["ok"]
    assert not stats["checks"]["right_handed"]


@pytest.mark.parametrize(
    "field,value",
    [("count", 1), ("position_jitter_m", 0.05), ("valid_fraction", 0.2)],
)
def test_solver_rejects_saved_poor_sample_quality(field: str, value: float) -> None:
    recorded = _recorded_for_mount(np.eye(3))
    recorded["forward"][field] = value

    _mount, stats = _solve(recorded)

    assert not stats["checks"]["sample_quality"]
    assert not stats["ok"]


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def monotonic(self) -> float:
        return self.now

    def monotonic_ns(self) -> int:
        return round(self.now * 1e9)

    def sleep(self, duration: float) -> None:
        self.now += duration


class FakeManager:
    def __init__(
        self,
        clock: FakeClock,
        *,
        delayed_body_s: float = 0.0,
        frozen: bool = False,
        jitter: bool = False,
        stale: bool = False,
    ) -> None:
        self.clock = clock
        self.started = clock.now
        self.delayed_body_s = delayed_body_s
        self.frozen = frozen
        self.jitter = jitter
        self.stale = stale

    def get_pose(self, name: str):
        elapsed = self.clock.now - self.started
        if name == "body" and elapsed < self.delayed_body_s:
            return None
        sequence = 1 if self.frozen else round(elapsed / calibration.SAMPLE_DT) + 1
        timestamp = (
            round(self.started * 1e9) if self.frozen else self.clock.monotonic_ns()
        )
        if self.stale:
            timestamp -= 1_000_000_000
        position_x = 0.3 if name == "hand" else 0.0
        if self.jitter and name == "hand":
            position_x += 0.05 if sequence % 2 else -0.05
        return TrackerPose(
            serial=name,
            logical_name=name,
            position_m=Vec3(position_x, 0.0, 0.0),
            orientation=IDENTITY_QUATERNION,
            sensor_timestamp=None,
            host_monotonic_ns=timestamp,
            tracking_valid=True,
            sequence_id=sequence,
            source="mock",
        )


@pytest.fixture
def fake_clock(monkeypatch: pytest.MonkeyPatch) -> FakeClock:
    clock = FakeClock()
    monkeypatch.setattr(calibration.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(calibration.time, "monotonic_ns", clock.monotonic_ns)
    monkeypatch.setattr(calibration.time, "sleep", clock.sleep)
    return clock


def test_wait_requires_both_fresh_trackers(fake_clock: FakeClock) -> None:
    manager = FakeManager(fake_clock, delayed_body_s=0.3)

    _wait_for_trackers(manager, "hand", "body", timeout=0.5)

    assert fake_clock.now - manager.started >= 0.3


def test_wait_rejects_stale_but_tracking_valid_poses(fake_clock: FakeClock) -> None:
    manager = FakeManager(fake_clock, stale=True)

    with pytest.raises(RuntimeError, match="未同时提供有效新位姿"):
        _wait_for_trackers(manager, "hand", "body", timeout=0.1)


def test_sample_waits_for_body_then_collects_full_window(fake_clock: FakeClock) -> None:
    manager = FakeManager(fake_clock, delayed_body_s=0.3)

    sample = _sample(manager, "hand", "body", "teleop", settle_s=0.5)

    assert fake_clock.now - manager.started >= 0.8
    assert sample["count"] >= calibration.MIN_SAMPLE_COUNT
    assert sample["position_jitter_m"] < 1e-12
    assert sample["sample_span_s"] >= 0.4
    np.testing.assert_allclose(sample["relative_position"], [0.3, 0.0, 0.0])


def test_sample_does_not_count_repeated_cached_poses(fake_clock: FakeClock) -> None:
    with pytest.raises(RuntimeError, match="新帧不足"):
        _sample(
            FakeManager(fake_clock, frozen=True), "hand", "body", "teleop", settle_s=0.5
        )


def test_sample_rejects_motion_hidden_by_position_average(fake_clock: FakeClock) -> None:
    with pytest.raises(RuntimeError, match="位置未停稳"):
        _sample(
            FakeManager(fake_clock, jitter=True), "hand", "body", "teleop", settle_s=0.5
        )


def test_new_baseline_clears_previously_recorded_motions(tmp_path: Path) -> None:
    path = tmp_path / "samples.yaml"
    recorded = _recorded_for_mount(np.eye(3))
    for key, sample in recorded.items():
        _write_sample(path, key, sample)

    _write_sample(path, "baseline", recorded["baseline"])

    assert set(yaml.safe_load(path.read_text(encoding="utf-8"))) == {"baseline"}
    with pytest.raises(RuntimeError, match="缺少步骤"):
        _read_samples(path)


def test_sample_quality_metadata_survives_saved_samples(tmp_path: Path) -> None:
    path = tmp_path / "samples.yaml"
    recorded = _recorded_for_mount(np.eye(3))
    for key, sample in recorded.items():
        sample.update(
            position_jitter_m=0.001,
            valid_fraction=0.99,
            sample_span_s=1.98,
            sample_duration_s=2.0,
        )
        _write_sample(path, key, sample)

    loaded = _read_samples(path)
    _mount, stats = _solve(loaded)

    assert stats["ok"]
    assert loaded["baseline"]["position_jitter_m"] == 0.001
    assert loaded["baseline"]["valid_fraction"] == 0.99


def test_write_config_replaces_only_body_mount_block(tmp_path: Path) -> None:
    config_path = tmp_path / "teleop.yaml"
    original = """teleop:
  head_base: true
  head_tracker_to_body:
    x: [\"x\", 1.0]
    y: [\"y\", 1.0]
    z: [\"z\", 1.0]
  delay_vr_baseline: true
transform:
  axis_map:
    x: [\"z\", -1.0]
    y: [\"y\", 1.0]
    z: [\"x\", 1.0]
"""
    config_path.write_text(original, encoding="utf-8")
    expected = _rotation_z(0.31)

    _write_config(config_path, expected)

    updated_text = config_path.read_text(encoding="utf-8")
    updated = yaml.safe_load(updated_text)
    np.testing.assert_allclose(
        updated["teleop"]["head_tracker_to_body"]["rotation_matrix"], expected
    )
    assert updated["teleop"]["delay_vr_baseline"] is True
    assert updated["transform"] == yaml.safe_load(original)["transform"]
