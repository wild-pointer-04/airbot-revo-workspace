from __future__ import annotations

import numpy as np
import pytest

from backends.airbot_sdk import AirbotSdkController


class FakeRobot:
    pass


class RecordingRobot:
    def __init__(self) -> None:
        self.profiles: list[object] = []
        self.params: list[dict] = []
        self.modes: list[object] = []

    def connect(self) -> bool:
        return True

    def disconnect(self) -> bool:
        return True

    def get_joint_pos(self) -> list[float]:
        return [0.0] * 6

    def get_end_pose(self) -> list:
        return [[0.126, 0.0, 0.21], [0.0, 0.0, 0.0, 1.0]]

    def set_speed_profile(self, profile: object) -> None:
        self.profiles.append(profile)

    def set_params(self, params: dict) -> None:
        self.params.append(dict(params))

    def switch_mode(self, mode: object) -> bool:
        self.modes.append(mode)
        return True


JOINT_LIMITS = [
    [-3.151, 2.089],
    [-2.963, 0.181],
    [-0.094, 3.161],
    [-3.012, 3.012],
    [-1.859, 1.859],
    [-3.017, 3.017],
]


def make_controller(robot: object | None = None, **overrides) -> AirbotSdkController:
    config = {
        "joint_count": 6,
        "joint_limits": JOINT_LIMITS,
    }
    config.update(overrides)
    return AirbotSdkController(robot if robot is not None else FakeRobot(), config)


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


def test_servo_scale_defaults_to_none():
    controller = make_controller()
    assert controller._servo_scale_linear is None
    assert controller._servo_scale_rotational is None
    assert controller._servo_scale_joint is None


def test_servo_scale_reads_config():
    controller = make_controller(
        servo_scale_linear=1.5,
        servo_scale_rotational=3.0,
        servo_scale_joint=0.6,
    )
    assert controller._servo_scale_linear == pytest.approx(1.5)
    assert controller._servo_scale_rotational == pytest.approx(3.0)
    assert controller._servo_scale_joint == pytest.approx(0.6)


def test_connect_overrides_default_servo_scale_after_speed_profile():
    pytest.importorskip("airbot_py.arm")
    robot = RecordingRobot()
    controller = make_controller(
        robot,
        servo_scale_linear=1.5,
        servo_scale_rotational=3.0,
        servo_scale_joint=0.6,
    )
    try:
        controller.connect()
        assert robot.profiles
        assert robot.params == [
            {
                "servo_node.moveit_servo.scale.linear": 1.5,
                "servo_node.moveit_servo.scale.rotational": 3.0,
                "servo_node.moveit_servo.scale.joint": 0.6,
            }
        ]
    finally:
        controller.disconnect()


def test_connect_skips_servo_scale_override_when_unset():
    pytest.importorskip("airbot_py.arm")
    robot = RecordingRobot()
    controller = make_controller(robot)
    try:
        controller.connect()
        assert robot.profiles
        assert robot.params == []
    finally:
        controller.disconnect()
