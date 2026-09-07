#!/usr/bin/env python3
"""Real-time axis verification: move hand tracker and see which axis responds.

Press Ctrl-C to exit.
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


def main() -> int:
    manager = TrackerManager.from_config(ROOT / "config" / "tracker.yaml")
    manager.start()
    print(f"\n等待 {TRACKER} 出位姿…")
    if manager.wait_for_pose(timeout=20.0) is None:
        print("超时：没有位姿")
        manager.stop()
        return 1

    print("\n实时轴验证 — 移动 tracker 观察哪个轴变化")
    print("=" * 70)
    print("预期: 朝前→x变大  朝左→y变大  举高→z变大")
    print("=" * 70)
    print("\n      libsurvive raw (cm)          变化量 Δ (cm)")
    print("      x       y       z          Δx     Δy     Δz")

    baseline = None
    try:
        while True:
            pose = manager.get_pose(TRACKER)
            if pose is None or not pose.tracking_valid:
                time.sleep(0.05)
                continue
            pos = np.array(pose.position_m.as_list()) * 100  # 转厘米
            if baseline is None:
                baseline = pos
            delta = pos - baseline
            print(
                f"\r  {pos[0]:7.1f} {pos[1]:7.1f} {pos[2]:7.1f}      "
                f"{delta[0]:+6.1f} {delta[1]:+6.1f} {delta[2]:+6.1f}   ",
                end="",
                flush=True,
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n停止")
    finally:
        manager.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
