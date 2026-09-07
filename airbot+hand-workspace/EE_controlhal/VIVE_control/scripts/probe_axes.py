#!/usr/bin/env python3
"""Measure which libsurvive world axis is up / forward / right.

Move the hand tracker as prompted; each step averages a short window so a
single noisy sample cannot decide an axis. Reports the dominant axis and sign
per motion, which is what vr_normalize needs to map into teleop_standard.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_tracking.tracker_manager import TrackerManager  # noqa: E402

TRACKER = "hand_controller"
SETTLE_S = 2.0

STEPS = [
    ("放在起始位置（腰高，手臂自然前伸）", "baseline"),
    ("把 tracker 举高约 50cm，保持不动", "up"),
    ("回到起始高度，向机器人方向平移约 40cm", "forward"),
    ("回到起始位置，向你的右手边平移约 40cm", "right"),
]


def mean_pose(manager: TrackerManager) -> np.ndarray:
    """Average position over SETTLE_S so jitter does not pick the axis."""
    samples: list[list[float]] = []
    deadline = time.time() + SETTLE_S
    while time.time() < deadline:
        pose = manager.get_pose(TRACKER)
        if pose is not None and pose.tracking_valid:
            samples.append(pose.position_m.as_list())
        time.sleep(0.02)
    if not samples:
        raise RuntimeError(f"{TRACKER} 没有有效位姿 —— 检查基站视野")
    return np.asarray(samples, dtype=np.float64).mean(axis=0)


def describe(delta: np.ndarray) -> str:
    idx = int(np.argmax(np.abs(delta)))
    axis = "xyz"[idx]
    sign = "+" if delta[idx] > 0 else "-"
    return f"主轴 = {sign}{axis}  (Δ={delta[idx]:+.3f} m)"


def main() -> int:
    manager = TrackerManager.from_config(ROOT / "config" / "tracker.yaml")
    manager.start()
    print(f"\n等待 {TRACKER} 出位姿…")
    if manager.wait_for_pose(timeout=20.0) is None:
        print("超时：没有位姿。检查 tracker 开机和基站视野。")
        manager.stop()
        return 1

    readings: dict[str, np.ndarray] = {}
    try:
        for prompt, key in STEPS:
            input(f"\n>>> {prompt}，摆好后按回车: ")
            print(f"    采样 {SETTLE_S:.0f}s，别动…")
            readings[key] = mean_pose(manager)
            print(f"    raw = {[round(v, 3) for v in readings[key]]}")
    finally:
        manager.stop()

    base = readings["baseline"]
    print("\n" + "=" * 52)
    print("相对起始位置的位移（libsurvive 世界系）")
    print("=" * 52)
    labels = {"up": "举高   (teleop +Z 上)", "forward": "朝机器人 (teleop +X 前)", "right": "朝右   (teleop -Y 右)"}
    for key, label in labels.items():
        delta = readings[key] - base
        print(f"\n{label}")
        print(f"  Δ = {[round(v, 3) for v in delta]}")
        print(f"  {describe(delta)}")
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
