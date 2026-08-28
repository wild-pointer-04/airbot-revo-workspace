#!/usr/bin/env python3
"""Read-only AIRBOT Play SDK smoke test."""

from __future__ import annotations

import argparse
import json
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read AIRBOT Play state without motion")
    parser.add_argument("--url", default="localhost")
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--joint-count", type=int, default=6)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        from airbot_py.arm import AIRBOTPlay
    except ImportError as exc:
        raise RuntimeError(
            "airbot_py is unavailable; rebuild airbot-hand-workspace:jazzy"
        ) from exc

    robot = AIRBOTPlay(url=args.url, port=args.port)
    if not robot.connect():
        raise ConnectionError(f"cannot connect to {args.url}:{args.port}")
    try:
        joint_pos = robot.get_joint_pos()
        if joint_pos is None or len(joint_pos) != args.joint_count:
            raise RuntimeError(f"expected {args.joint_count} joints, got {joint_pos!r}")
        output = {
            "product_info": robot.get_product_info(),
            "joint_position": joint_pos,
            "joint_velocity": robot.get_joint_vel(),
            "joint_effort": robot.get_joint_eff(),
            "end_pose": robot.get_end_pose(),
        }
        print(json.dumps(output, ensure_ascii=False, indent=2, default=str))
    finally:
        robot.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
