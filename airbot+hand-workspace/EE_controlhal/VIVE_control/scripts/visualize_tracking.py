#!/usr/bin/env python3
"""Real-time 3D visualization of hand tracker position in teleop frame.

Shows:
- Current hand position as a red sphere
- Trajectory trail (last 300 points)
- Coordinate axes (X=red forward, Y=green left, Z=blue up)
- Position text readout

Move the hand tracker and watch it move in the plot. Close the window to exit.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(MIDDLE))

from vive_tracking.tracker_manager import TrackerManager  # noqa: E402
from processing.vr_normalize import libsurvive_to_teleop_position  # noqa: E402

TRACKER = "hand_controller"
TRAIL_LEN = 300


class TrackerVisualizer:
    def __init__(self):
        self.manager = TrackerManager.from_config(ROOT / "config" / "tracker.yaml")
        self.manager.start()
        print(f"等待 {TRACKER} 出位姿…")
        if self.manager.wait_for_pose(timeout=20.0) is None:
            raise RuntimeError(f"{TRACKER} 没有位姿")
        print("已连接，启动可视化窗口…\n")

        self.trail = []
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self._setup_plot()

        # 图形元素
        (self.point,) = self.ax.plot([], [], [], "ro", markersize=12, label="当前位置")
        (self.trail_line,) = self.ax.plot([], [], [], "b-", linewidth=1, alpha=0.6, label="轨迹")
        self.text = self.ax.text2D(0.02, 0.98, "", transform=self.ax.transAxes, va="top", fontsize=10)

    def _setup_plot(self):
        self.ax.set_xlabel("X (前, cm)", fontsize=11, color="red")
        self.ax.set_ylabel("Y (左, cm)", fontsize=11, color="green")
        self.ax.set_zlabel("Z (上, cm)", fontsize=11, color="blue")
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylim(-50, 50)
        self.ax.set_zlim(-20, 80)
        self.ax.legend(loc="upper right")
        self.ax.set_title("VIVE Tracker 实时位置 (teleop 坐标系)", fontsize=13, fontweight="bold")
        # 画坐标轴箭头
        origin = [0, 0, 0]
        self.ax.quiver(*origin, 15, 0, 0, color="red", arrow_length_ratio=0.15, linewidth=2, alpha=0.7)
        self.ax.quiver(*origin, 0, 15, 0, color="green", arrow_length_ratio=0.15, linewidth=2, alpha=0.7)
        self.ax.quiver(*origin, 0, 0, 15, color="blue", arrow_length_ratio=0.15, linewidth=2, alpha=0.7)

    def update(self, frame):
        pose = self.manager.get_pose(TRACKER)
        if pose is None or not pose.tracking_valid:
            return self.point, self.trail_line, self.text

        # libsurvive raw → teleop
        pos_teleop = libsurvive_to_teleop_position(np.array(pose.position_m.as_list())) * 100  # cm
        x, y, z = pos_teleop

        # 更新轨迹
        self.trail.append([x, y, z])
        if len(self.trail) > TRAIL_LEN:
            self.trail.pop(0)

        # 更新图形
        self.point.set_data([x], [y])
        self.point.set_3d_properties([z])

        if len(self.trail) > 1:
            trail_arr = np.array(self.trail)
            self.trail_line.set_data(trail_arr[:, 0], trail_arr[:, 1])
            self.trail_line.set_3d_properties(trail_arr[:, 2])

        # 更新文字
        self.text.set_text(
            f"X(前): {x:+6.1f} cm\n"
            f"Y(左): {y:+6.1f} cm\n"
            f"Z(上): {z:+6.1f} cm\n"
            f"\n往前伸 → X 变大\n往左移 → Y 变大\n举高   → Z 变大"
        )
        return self.point, self.trail_line, self.text

    def run(self):
        try:
            anim = FuncAnimation(self.fig, self.update, interval=50, blit=False, cache_frame_data=False)
            plt.show()
        except KeyboardInterrupt:
            print("\n停止")
        finally:
            self.manager.stop()


def main() -> int:
    try:
        viz = TrackerVisualizer()
        viz.run()
        return 0
    except Exception as exc:
        print(f"错误: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
