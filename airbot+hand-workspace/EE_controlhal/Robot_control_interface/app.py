"""Command-layer entry: connect to the arm backend and print TCP."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import yaml

from factory import create_controller
from arm_types import MotionMode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robot command layer")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "config" / "default.yaml"),
    )
    parser.add_argument(
        "--mode",
        choices=["status", "protect", "home", "gravity"],
        default="status",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with open(args.config, encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError(f"config {args.config} must be a mapping")
    logging.basicConfig(
        level=getattr(logging, str((config.get("logging") or {}).get("level", "INFO")).upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    arm = create_controller(config)
    logging.warning("ARM BACKEND: %s", str(config.get("backend", "dummy")).upper())
    arm.connect()
    try:
        if args.mode == "protect":
            arm.stop()
        elif args.mode == "home":
            arm.set_mode(MotionMode.GO_HOME)
        elif args.mode == "gravity":
            arm.set_mode(MotionMode.GRAVITY)
        state = arm.get_state()
        print(
            f"tcp pos={state.position.round(4).tolist()}  "
            f"rpy={state.xyzrpy[3:].round(4).tolist()}  "
            f"q={state.joint_pos.round(4).tolist()}",
            flush=True,
        )
        if args.mode == "status":
            time.sleep(0.2)
    finally:
        if args.mode == "status":
            arm.disconnect()
        else:
            time.sleep(0.1)
            arm.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
