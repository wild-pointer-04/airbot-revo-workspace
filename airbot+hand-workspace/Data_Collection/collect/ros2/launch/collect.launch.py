#!/usr/bin/env python3
"""Start the LeRobot collector. Arm state comes from /arm/state (VIVE/PICO teleop)."""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction

COLLECT_ROOT = Path(__file__).resolve().parents[1]
COLLECT = COLLECT_ROOT / "collect.py"

sys.path.insert(0, str(COLLECT_ROOT))
from collector.ros_env import find_ros_setup  # noqa: E402

ROS_SETUP = find_ros_setup()


def _bash(inner: str) -> list[str]:
    if ROS_SETUP is None:
        return ["bash", "-lc", inner]
    return ["bash", "-lc", f"source {ROS_SETUP} && {inner}"]


def launch_setup(context, *args, **kwargs):
    mock = context.launch_configurations.get("mock", "false").lower() in ("true", "1")
    task = context.launch_configurations.get("task", "pick_cup")
    collect_mock = " --mock" if mock else ""
    return [
        ExecuteProcess(
            cmd=_bash(f"python3 {COLLECT} --task {task}{collect_mock}"),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("task", default_value="pick_cup"),
            DeclareLaunchArgument("mock", default_value="false"),
            LogInfo(msg=["LeRobot collector subscribes /arm/state"]),
            OpaqueFunction(function=launch_setup),
        ]
    )
