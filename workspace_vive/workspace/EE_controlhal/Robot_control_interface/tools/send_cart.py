"""Send one absolute Cartesian target (base frame, meters + xyzw)."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from factory import create_controller
from arm_types import ArmCommand, MotionMode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send one ARX Cartesian command")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[1] / "config" / "arx_ros2.yaml"),
    )
    parser.add_argument("--xyz", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument(
        "--xyzw",
        nargs=4,
        type=float,
        default=[0.0, 0.0, 0.0, 1.0],
        metavar=("QX", "QY", "QZ", "QW"),
    )
    parser.add_argument("--seconds", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    with open(args.config, encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    arm = create_controller(config)
    arm.connect()
    try:
        cmd = ArmCommand(
            position=np.asarray(args.xyz, dtype=np.float64),
            rotation=np.asarray(args.xyzw, dtype=np.float64),
            mode=MotionMode.END_CONTROL,
            source="send_cart",
        )
        arm.send_command(cmd)
        time.sleep(max(args.seconds, 0.05))
        arm.stop()
        state = arm.get_state()
        print(f"held pos={state.position.round(4).tolist()}", flush=True)
    finally:
        arm.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
