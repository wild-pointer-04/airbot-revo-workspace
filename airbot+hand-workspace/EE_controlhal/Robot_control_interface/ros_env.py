"""Make rclpy importable. If the shell did not source ROS, re-exec through setup.bash.

The AIRBOT backend only needs rclpy (to publish /arm/state). The ARX X5 backend
additionally needs arx5_arm_msg from its own colcon workspace, so X5_SETUP is
sourced when present but never required.
"""

from __future__ import annotations

import os
import shlex
import sys
from pathlib import Path

# The combined AIRBOT + Revo2 container runs Humble; the original bench ran Jazzy.
PREFERRED_DISTROS = ("humble", "jazzy")
X5_SETUP = Path("/home/dt/ego_ARX/ARX/ARX_X5-main/ROS2/X5_ws/install/setup.bash")
_READY = "ROBOT_CMD_ROS_READY"


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


def rclpy_importable() -> bool:
    try:
        import rclpy  # noqa: F401
    except ImportError:
        return False
    return True


def arx_msgs_importable() -> bool:
    try:
        from arx5_arm_msg.msg import RobotCmd  # noqa: F401
    except ImportError:
        return False
    return True


def ros_importable() -> bool:
    return rclpy_importable()


def ensure_ros(*, require_arx: bool = False) -> None:
    if rclpy_importable() and (not require_arx or arx_msgs_importable()):
        return
    if os.environ.get(_READY) == "1":
        missing = "rclpy" if not rclpy_importable() else "arx5_arm_msg"
        raise RuntimeError(f"{missing} still missing after sourcing ROS 2")
    setup = find_ros_setup()
    if setup is None:
        raise RuntimeError(
            "No ROS 2 installation under /opt/ros. Expected one of: "
            + ", ".join(PREFERRED_DISTROS)
        )
    if require_arx and not X5_SETUP.is_file():
        raise RuntimeError(f"X5 workspace not built: {X5_SETUP}")
    if not sys.argv or sys.argv[0] in {"-c", "-m"}:
        raise RuntimeError(
            "rclpy is not importable in this process. Use ./run.sh "
            f"or: source {setup}"
        )

    quoted = " ".join(shlex.quote(part) for part in [sys.executable, *sys.argv])
    sources = f"source {shlex.quote(str(setup))} && "
    if X5_SETUP.is_file():
        sources += f"source {shlex.quote(str(X5_SETUP))} && "
    os.execvp("bash", ["bash", "-c", f"{sources}export {_READY}=1 && exec {quoted}"])
