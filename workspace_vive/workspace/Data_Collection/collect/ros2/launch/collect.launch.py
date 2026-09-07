#!/usr/bin/env python3
"""Start the LeRobot collector. Arm state comes from /arm/state (VIVE/PICO teleop)."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction

COLLECT_ROOT = Path(__file__).resolve().parents[1]
COLLECT = COLLECT_ROOT / "collect.py"
JAZZY = "/opt/ros/jazzy/setup.bash"


def _bash(inner: str) -> list[str]:
    return ["bash", "-lc", f"source {JAZZY} && {inner}"]


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
