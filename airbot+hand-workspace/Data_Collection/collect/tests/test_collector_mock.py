from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import pytest

COLLECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COLLECT))

from collector.collector import Collector, load_collect_config


def _cfg(tmp_path: Path) -> dict:
    cfg = load_collect_config(COLLECT / "config" / "collect.yaml")
    cfg["mock"] = True
    cfg["state_source"] = "local"
    cfg["task"] = "mock_smoke"
    cfg["data_root"] = str(tmp_path)
    cfg.setdefault("dataset", {})
    cfg["dataset"]["repo_id"] = "mock_smoke"
    cfg["dataset"]["root"] = str(tmp_path / "lerobot")
    cfg["dataset"]["use_videos"] = False
    return cfg


def _run_ticks(collector: Collector, n: int = 5) -> None:
    time.sleep(0.15)
    collector.start_recording()
    for _ in range(n):
        collector._record_tick()
        time.sleep(0.02)
    collector.stop_recording()
    collector.save_episode()


def test_mock_episode_writes_camera_and_arm(tmp_path: Path) -> None:
    collector = Collector(_cfg(tmp_path), mock=True)
    collector.start_session()
    try:
        _run_ticks(collector, 5)
    finally:
        collector.shutdown()

    frames_path = (
        tmp_path / "lerobot" / "mock_smoke" / "episodes" / "episode-000000" / "frames.jsonl"
    )
    if not frames_path.is_file():
        info = tmp_path / "lerobot" / "mock_smoke" / "meta" / "info.json"
        assert info.is_file()
        return

    rows = [json.loads(line) for line in frames_path.read_text().splitlines() if line.strip()]
    assert len(rows) >= 5
    row = rows[-1]
    assert "observation.images.cam_global" in row
    assert "observation.images.cam_high" not in row
    assert "observation.state.hand_angle" not in row
    assert "observation.state.gripper_pos" not in row
    assert "observation.state.vive_position" not in row
    assert "observation.state.hand_right_joint" in row
    assert len(row["observation.state.hand_right_joint"]) == 6
    assert len(row["observation.state.hand_right_tactile"]) == 5
    assert row["observation.state.hand_right_valid"][0] == pytest.approx(1.0)
    valid_rows = [r for r in rows if r["observation.state.arm_valid"][0] == 1.0]
    assert valid_rows, "expected at least one valid arm frame"
    row = valid_rows[-1]
    assert "observation.host_monotonic_ns" in row
    assert len(row["observation.host_monotonic_ns"]) == 1
    assert row["observation.host_monotonic_ns"][0] > 0
    assert len(row["observation.state.arm_joint"]) == 6
    assert len(row["observation.state.arm_ee_position"]) == 3
    assert len(row["observation.state.arm_ee_orientation"]) == 4
    quat = row["observation.state.arm_ee_orientation"]
    assert abs((quat[0] ** 2 + quat[1] ** 2 + quat[2] ** 2 + quat[3] ** 2) ** 0.5 - 1.0) < 0.05
    assert quat[3] == pytest.approx(1.0, abs=0.05)
    pos = row["observation.state.arm_ee_position"]
    assert row["observation.state.arm_valid"][0] == pytest.approx(1.0)
    assert pos[0] == pytest.approx(0.30, abs=0.05)
    assert pos[2] == pytest.approx(0.20, abs=0.05)
    assert len(row["action"]) == 6
    stamps = [r["observation.host_monotonic_ns"][0] for r in rows]
    assert stamps == sorted(stamps)
    assert not (tmp_path / "lerobot" / "mock_smoke" / "audio").exists()


def test_arm_ros_sidecar_keys_are_not_written(tmp_path: Path) -> None:
    collector = Collector(_cfg(tmp_path), mock=True)
    collector.start_session()
    try:
        extra = {
            "host_monotonic_ns": 123456,
            "sequence_id": 7,
            "observation.state.arm_valid": [1.0],
        }
        collector._buffer.update("arm", extra)
        _run_ticks(collector, 2)
    finally:
        collector.shutdown()
    frames_path = (
        tmp_path / "lerobot" / "mock_smoke" / "episodes" / "episode-000000" / "frames.jsonl"
    )
    if frames_path.is_file():
        row = json.loads(frames_path.read_text().splitlines()[0])
        assert "host_monotonic_ns" not in row
        assert "sequence_id" not in row
