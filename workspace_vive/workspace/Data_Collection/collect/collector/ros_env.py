"""Source ROS 2 Jazzy so rclpy can be imported."""

from __future__ import annotations

import os
import shlex
import sys
from pathlib import Path

JAZZY_SETUP = Path("/opt/ros/jazzy/setup.bash")
_READY = "COLLECT_ROS_READY"


def rclpy_importable() -> bool:
    try:
        import rclpy  # noqa: F401
    except ImportError:
        return False
    return True


def ensure_rclpy() -> None:
    if rclpy_importable():
        return
    if os.environ.get(_READY) == "1":
        raise RuntimeError("rclpy still missing after sourcing ROS 2 Jazzy")
    if not JAZZY_SETUP.is_file():
        raise RuntimeError(f"ROS 2 Jazzy not found: {JAZZY_SETUP}")
    if not sys.argv or sys.argv[0] in {"-c", "-m"}:
        raise RuntimeError(
            "rclpy is not importable. Source /opt/ros/jazzy/setup.bash first."
        )
    quoted = " ".join(shlex.quote(part) for part in [sys.executable, *sys.argv])
    script = (
        f"source {shlex.quote(str(JAZZY_SETUP))} && "
        f"export {_READY}=1 && "
        f"exec {quoted}"
    )
    os.execvp("bash", ["bash", "-c", script])
