"""Source ROS 2 so rclpy can be imported."""

from __future__ import annotations

import os
import shlex
import sys
from pathlib import Path

# The combined AIRBOT + Revo2 container runs Humble; the original bench ran Jazzy.
PREFERRED_DISTROS = ("humble", "jazzy")
_READY = "COLLECT_ROS_READY"


def rclpy_importable() -> bool:
    try:
        import rclpy  # noqa: F401
    except ImportError:
        return False
    return True


def find_ros_setup() -> Path | None:
    """Locate setup.bash: $ROS_DISTRO first, then the preferred list, then any install."""
    candidates = []
    distro = os.environ.get("ROS_DISTRO")
    if distro:
        candidates.append(distro)
    candidates.extend(PREFERRED_DISTROS)
    for name in candidates:
        setup = Path("/opt/ros") / name / "setup.bash"
        if setup.is_file():
            return setup
    return next(iter(sorted(Path("/opt/ros").glob("*/setup.bash"))), None)


def ensure_rclpy() -> None:
    if rclpy_importable():
        return
    if os.environ.get(_READY) == "1":
        raise RuntimeError("rclpy still missing after sourcing ROS 2")
    setup = find_ros_setup()
    if setup is None:
        raise RuntimeError(
            "No ROS 2 installation under /opt/ros. Expected one of: "
            + ", ".join(PREFERRED_DISTROS)
        )
    if not sys.argv or sys.argv[0] in {"-c", "-m"}:
        raise RuntimeError(f"rclpy is not importable. Source {setup} first.")
    quoted = " ".join(shlex.quote(part) for part in [sys.executable, *sys.argv])
    script = (
        f"source {shlex.quote(str(setup))} && "
        f"export {_READY}=1 && "
        f"exec {quoted}"
    )
    os.execvp("bash", ["bash", "-c", script])
