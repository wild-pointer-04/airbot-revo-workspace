#!/usr/bin/env python3
"""Capture static AIRBOT states for payload identification.

The operator manually places the arm in several safe poses. This tool only
reads state and writes CSV; it never switches modes or sends motion commands.
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="payload_static.csv")
    parser.add_argument("--poses", type=int, default=8)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()
    if args.poses < 4 or args.seconds <= 0 or args.hz <= 0:
        raise ValueError("use at least 4 poses and positive sampling settings")

    from airbot_py.arm import AIRBOTPlay

    robot = AIRBOTPlay(url="localhost", port=50051)
    if not robot.connect():
        raise ConnectionError("cannot connect to AIRBOT runtime")
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = ["timestamp"] + [f"q{i}" for i in range(6)]
    fields += [f"tau{i}" for i in range(6)]
    fields += [f"x{i}" for i in range(3)] + [f"quat{i}" for i in range(4)]
    try:
        with output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for pose_index in range(args.poses):
                input(f"将机械臂手动放到安全静态姿态 {pose_index + 1}/{args.poses}，确认稳定后按 Enter...")
                end = time.monotonic() + args.seconds
                while time.monotonic() < end:
                    q = list(robot.get_joint_pos())
                    tau = list(robot.get_joint_eff())
                    pose = robot.get_end_pose()
                    if len(q) != 6 or len(tau) != 6 or pose is None or len(pose) != 2:
                        raise RuntimeError("invalid AIRBOT state dimensions")
                    position, quaternion = pose
                    if len(position) != 3 or len(quaternion) != 4:
                        raise RuntimeError("invalid end pose dimensions")
                    row = {"timestamp": time.time_ns()}
                    row.update({f"q{i}": q[i] for i in range(6)})
                    row.update({f"tau{i}": tau[i] for i in range(6)})
                    row.update({f"x{i}": position[i] for i in range(3)})
                    row.update({f"quat{i}": quaternion[i] for i in range(4)})
                    writer.writerow(row)
                    handle.flush()
                    time.sleep(1.0 / args.hz)
                print(f"已保存姿态 {pose_index + 1}/{args.poses}")
    finally:
        robot.disconnect()
    print(f"采集完成：{output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
