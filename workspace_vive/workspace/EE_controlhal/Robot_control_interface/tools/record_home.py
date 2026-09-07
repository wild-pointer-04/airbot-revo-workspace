#!/usr/bin/env python3
"""Record the current AIRBOT joint position as the configured home pose."""

from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path

import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/airbot_sdk.yaml")
    args = parser.parse_args()
    path = Path(args.config).expanduser().resolve()

    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    airbot = config["airbot"]

    from airbot_py.arm import AIRBOTPlay

    robot = AIRBOTPlay(
        url=str(airbot.get("url", "localhost")),
        port=int(airbot.get("port", 50051)),
    )
    if not robot.connect():
        raise ConnectionError("cannot connect to AIRBOT runtime")
    try:
        joints = [float(value) for value in robot.get_joint_pos()]
    finally:
        robot.disconnect()
    if len(joints) != int(airbot.get("joint_count", 6)):
        raise RuntimeError(f"unexpected joint state: {joints!r}")

    print("Current joints (rad):")
    print("[" + ", ".join(f"{value:.8f}" for value in joints) + "]")
    if input("输入 SAVE 将此姿态保存为 home_joints，其他输入取消: ").strip() != "SAVE":
        print("Cancelled; config unchanged")
        return 1

    backup = path.with_name(path.name + time.strftime(".bak-%Y%m%d-%H%M%S"))
    shutil.copy2(path, backup)
    airbot["home_joints"] = joints
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(path)
    print(f"Saved: {path}")
    print(f"Backup: {backup}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
