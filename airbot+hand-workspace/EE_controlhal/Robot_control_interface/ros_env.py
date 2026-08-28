"""Make rclpy importable. If the shell did not source ROS, re-exec through setup.bash."""

from __future__ import annotations

import os
import shlex
import sys
from pathlib import Path

JAZZY_SETUP = Path("/opt/ros/jazzy/setup.bash")
X5_SETUP = Path("/home/dt/ego_ARX/ARX/ARX_X5-main/ROS2/X5_ws/install/setup.bash")
_READY = "ROBOT_CMD_ROS_READY"


def ros_importable() -> bool:
    try:
        import rclpy  # noqa: F401
        from arx5_arm_msg.msg import RobotCmd  # noqa: F401
    except ImportError:
        return False
    return True


def ensure_ros() -> None:
    if ros_importable():
        return
    if os.environ.get(_READY) == "1":
        raise RuntimeError(
            "rclpy / arx5_arm_msg still missing after sourcing ROS 2. "
            f"Check {JAZZY_SETUP} and {X5_SETUP}"
        )
    if not JAZZY_SETUP.is_file():
        raise RuntimeError(f"ROS 2 Jazzy not found: {JAZZY_SETUP}")
    if not X5_SETUP.is_file():
        raise RuntimeError(f"X5 workspace not built: {X5_SETUP}")
    if not sys.argv or sys.argv[0] in {"-c", "-m"}:
        raise RuntimeError(
            "rclpy is not importable in this process. Use ./run.sh "
            "or: source /opt/ros/jazzy/setup.bash && "
            "source /home/dt/ego_ARX/ARX/ARX_X5-main/ROS2/X5_ws/install/setup.bash"
        )

    quoted = " ".join(shlex.quote(part) for part in [sys.executable, *sys.argv])
    script = (
        f"source {shlex.quote(str(JAZZY_SETUP))} && "
        f"source {shlex.quote(str(X5_SETUP))} && "
        f"export {_READY}=1 && "
        f"exec {quoted}"
    )
    os.execvp("bash", ["bash", "-c", script])
