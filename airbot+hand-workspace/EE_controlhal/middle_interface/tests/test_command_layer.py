"""middle_interface can drive Robot_control_interface without ROS."""

from __future__ import annotations

import numpy as np

from protocol.robot_command import RobotCommand
from robot import create_adapter


def test_command_layer_dummy_roundtrip():
    config = {
        "robot": {
            "adapter": "arx",
            "command_layer": {
                "backend": "dummy",
                "dummy": {
                    "position": [0.30, 0.0, 0.20],
                    "rotation": [0.0, 0.0, 0.0, 1.0],
                },
            },
        }
    }
    adapter = create_adapter(config)
    adapter.connect()
    start = adapter.get_ee_pose()
    np.testing.assert_allclose(start.position, [0.30, 0.0, 0.20])
    cmd = RobotCommand(
        timestamp_ns=1,
        position=np.array([0.32, 0.01, 0.21]),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
        gripper=0.0,
        enabled=True,
        source="test",
    )
    adapter.send_ee_target(cmd)
    now = adapter.get_ee_pose()
    np.testing.assert_allclose(now.position, cmd.position)
    adapter.stop()
    adapter.disconnect()
