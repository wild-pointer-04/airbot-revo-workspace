#!/usr/bin/env python3
"""Record hand tracker motion for 20s, then save a 3D plot to PNG.

Usage:
  python3 scripts/record_and_plot.py

Output: tracker_motion.png (open it to see the 3D trajectory)
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(MIDDLE))

from vive_tracking.tracker_manager import TrackerManager  # noqa: E402
from processing.vr_normalize import libsurvive_to_teleop_position  # noqa: E402

TRACKER = "hand_controller"
DURATION = 20


def main() -> int:
    manager = TrackerManager.from_config(ROOT / "config" / "tracker.yaml")
    manager.start()
    print(f"等待 {TRACKER} 出位姿…")
    if manager.wait_for_pose(timeout=20.0) is None:
        print("超时")
        manager.stop()
        return 1

    print(f"\n开始录制 {DURATION} 秒，移动 tracker 测试三个方向:")
    print("  1. 往前伸")
    print("  2. 往左移")
    print("  3. 举高")
    print("\n录制中", end="", flush=True)

    trail = []
    t0 = time.time()
    while time.time() - t0 < DURATION:
        pose = manager.get_pose(TRACKER)
        if pose and pose.tracking_valid:
            pos_teleop = libsurvive_to_teleop_position(np.array(pose.position_m.as_list())) * 100
            trail.append(pos_teleop.tolist())
        if int(time.time() - t0) % 2 == 0:
            print(".", end="", flush=True)
        time.sleep(0.05)

    manager.stop()
    print(f"\n\n录制完成，共 {len(trail)} 个点")

    if len(trail) < 10:
        print("数据点太少，无法绘图")
        return 1

    # 绘图
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
    except ImportError:
        print("matplotlib 未安装，无法绘图")
        return 1

    trail_arr = np.array(trail)
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    # 轨迹线，颜色按时间渐变
    colors = plt.cm.viridis(np.linspace(0, 1, len(trail_arr)))
    for i in range(len(trail_arr) - 1):
        ax.plot(
            trail_arr[i : i + 2, 0],
            trail_arr[i : i + 2, 1],
            trail_arr[i : i + 2, 2],
            color=colors[i],
            linewidth=2,
        )

    # 起点和终点
    ax.scatter(*trail_arr[0], color="green", s=100, marker="o", label="起点", edgecolor="black", linewidth=1.5)
    ax.scatter(*trail_arr[-1], color="red", s=100, marker="o", label="终点", edgecolor="black", linewidth=1.5)

    # 坐标轴箭头
    origin = trail_arr.mean(axis=0)
    length = 20
    ax.quiver(*origin, length, 0, 0, color="red", arrow_length_ratio=0.2, linewidth=2.5, label="X(前)")
    ax.quiver(*origin, 0, length, 0, color="green", arrow_length_ratio=0.2, linewidth=2.5, label="Y(左)")
    ax.quiver(*origin, 0, 0, length, color="blue", arrow_length_ratio=0.2, linewidth=2.5, label="Z(上)")

    ax.set_xlabel("X (前, cm)", fontsize=12, fontweight="bold", color="red")
    ax.set_ylabel("Y (左, cm)", fontsize=12, fontweight="bold", color="green")
    ax.set_zlabel("Z (上, cm)", fontsize=12, fontweight="bold", color="blue")
    ax.set_title(f"VIVE Tracker 运动轨迹 (teleop 坐标系, {DURATION}s)", fontsize=14, fontweight="bold")
    ax.legend(loc="upper left", fontsize=10)
    ax.grid(True, alpha=0.3)

    # 统计
    span = trail_arr.max(axis=0) - trail_arr.min(axis=0)
    stats_text = (
        f"运动范围 (cm):\n"
        f"  X(前): {span[0]:.1f}\n"
        f"  Y(左): {span[1]:.1f}\n"
        f"  Z(上): {span[2]:.1f}"
    )
    ax.text2D(0.02, 0.98, stats_text, transform=ax.transAxes, va="top", fontsize=10, bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

    out_path = ROOT / "tracker_motion.png"
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\n已保存: {out_path}")
    print("用图片查看器打开，验证:")
    print("  - 往前伸时轨迹沿红色 X 轴(前)")
    print("  - 往左移时轨迹沿绿色 Y 轴(左)")
    print("  - 举高时轨迹沿蓝色 Z 轴(上)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
