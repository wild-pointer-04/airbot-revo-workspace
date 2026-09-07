from __future__ import annotations

import numpy as np
import pytest

from backends.airbot_sdk import AirbotSdkController


class FakeRobot:
    pass


def make_controller() -> AirbotSdkController:
    return AirbotSdkController(
        FakeRobot(),
        {
            "joint_count": 6,
            "joint_limits": [
                [-3.151, 2.089],
                [-2.963, 0.181],
                [-0.094, 3.161],
                [-3.012, 3.012],
                [-1.859, 1.859],
                [-3.017, 3.017],
            ],
        },
    )


def test_joint_limit_validation_accepts_valid_state():
    controller = make_controller()
    controller._validate_joint_limits("state", np.zeros(6))


def test_joint_limit_validation_reports_each_invalid_joint():
    controller = make_controller()
    with pytest.raises(RuntimeError, match=r"joint2=1\.396.*joint3=-0\.777"):
        controller._validate_joint_limits(
            "current state",
            np.array([0.0, 1.396, -0.777, 0.0, 0.0, 0.0]),
        )
