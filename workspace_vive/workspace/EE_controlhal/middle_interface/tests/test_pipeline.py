"""End-to-end pipeline with DummyRobotAdapter (no hardware)."""

from __future__ import annotations

import numpy as np
import yaml

from processing.pipeline import TeleopPipeline
from protocol.pico_packet import (
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    ControllerState,
    HeadState,
    PicoTeleopPacket,
)
from robot.dummy_robot_adapter import DummyRobotAdapter


def _ctrl(
    pos,
    grip=0.0,
    valid=True,
    trigger=0.0,
    primary=False,
    secondary=False,
) -> ControllerState:
    return ControllerState(
        valid=valid,
        position=np.asarray(pos, dtype=np.float64),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
        linear_velocity=np.zeros(3),
        angular_velocity=np.zeros(3),
        trigger=trigger,
        grip=grip,
        primary_button=primary,
        secondary_button=secondary,
        joystick=np.zeros(2),
    )


def _packet(
    seq: int,
    right_pos,
    trigger: float = 0.0,
    grip: float = 0.0,
    valid: bool = True,
    right_primary: bool = False,
) -> PicoTeleopPacket:
    ident = np.array([0.0, 0.0, 0.0, 1.0])
    pkt = PicoTeleopPacket(
        protocol=PROTOCOL_NAME,
        version=PROTOCOL_VERSION,
        seq=seq,
        device_time_ns=1,
        wall_time_unix_ms=1,
        hmd=HeadState(True, np.array([0.0, 1.6, 0.0]), ident),
        left_controller=_ctrl([-0.2, 1.1, 0.25], 0.0),
        right_controller=_ctrl(
            right_pos,
            grip=grip,
            valid=valid,
            trigger=trigger,
            primary=right_primary,
        ),
    )
    pkt.stamp_receive()
    return pkt


def _config() -> dict:
    with open("config/default.yaml", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    config["limits"]["max_linear_velocity"] = 10.0
    config["limits"]["max_angular_velocity"] = 10.0
    config["filter"]["enabled"] = False
    return config


def test_dummy_pipeline_deadman_and_stop():
    config = _config()
    adapter = DummyRobotAdapter.from_config(config["robot"])
    adapter.connect()
    pipe = TeleopPipeline(config, adapter.get_ee_pose)

    start = adapter.get_ee_pose().position.copy()
    idle = pipe.process(_packet(1, [0.0, 1.2, 0.3], trigger=0.0))
    assert idle.command is None

    press = pipe.process(_packet(2, [0.0, 1.2, 0.3], trigger=0.8, grip=0.2))
    assert press.clutch_active
    assert press.command is not None
    np.testing.assert_allclose(press.command.position, start)
    assert press.command.gripper == 0.2
    adapter.send_ee_target(press.command)

    capture = pipe.process(_packet(3, [0.0, 1.2, 0.3], trigger=0.8, grip=0.2))
    assert capture.command is not None
    adapter.send_ee_target(capture.command)

    # OpenXR +X (right) → teleop -Y. Dummy command frame is identity.
    moved = pipe.process(_packet(4, [0.05, 1.2, 0.3], trigger=0.8, grip=0.9))
    assert moved.command is not None
    assert moved.command.gripper == 1.0
    adapter.send_ee_target(moved.command)
    delta = adapter.get_ee_pose().position - start
    assert delta[1] < 0.0

    released = pipe.process(_packet(5, [0.05, 1.2, 0.3], trigger=0.1))
    assert released.should_stop
    adapter.stop()

    lost = pipe.process(_packet(6, [0.05, 1.2, 0.3], trigger=0.8, valid=False))
    assert lost.command is None


def test_tracking_lost_keeps_baseline():
    config = _config()
    adapter = DummyRobotAdapter.from_config(config["robot"])
    adapter.connect()
    pipe = TeleopPipeline(config, adapter.get_ee_pose)

    pipe.process(_packet(1, [0.0, 1.2, 0.3], trigger=0.8))
    pipe.process(_packet(2, [0.0, 1.2, 0.3], trigger=0.8))
    moved = pipe.process(_packet(3, [0.05, 1.2, 0.3], trigger=0.8))
    assert moved.command is not None
    adapter.send_ee_target(moved.command)
    held = adapter.get_ee_pose().position.copy()

    lost = pipe.process(_packet(4, [0.05, 1.2, 0.3], trigger=0.8, valid=False))
    assert lost.command is None
    assert lost.clutch_active
    assert pipe.safety.state.value == "TELEOP_ACTIVE"

    back = pipe.process(_packet(5, [0.05, 1.2, 0.3], trigger=0.8))
    assert back.command is not None
    np.testing.assert_allclose(back.command.position, held, atol=1e-6)


def test_timeout_stops_commands():
    config = _config()
    adapter = DummyRobotAdapter.from_config(config["robot"])
    pipe = TeleopPipeline(config, adapter.get_ee_pose)
    pipe.process(_packet(1, [0.0, 1.2, 0.3], trigger=0.8))
    result = pipe.on_timeout()
    assert result.should_stop
    assert result.command is None


def test_right_a_requests_go_home():
    config = _config()
    adapter = DummyRobotAdapter.from_config(config["robot"])
    adapter.connect()
    pipe = TeleopPipeline(config, adapter.get_ee_pose)

    pipe.process(_packet(1, [0.0, 1.2, 0.3], trigger=0.8))
    pipe.process(_packet(2, [0.0, 1.2, 0.3], trigger=0.8))
    moved = pipe.process(_packet(3, [0.05, 1.2, 0.3], trigger=0.8))
    assert moved.command is not None
    adapter.send_ee_target(moved.command)
    assert not np.allclose(adapter.get_ee_pose().position, [0.30, 0.0, 0.20])

    edge = pipe.process(_packet(4, [0.05, 1.2, 0.3], trigger=0.8, right_primary=True))
    assert edge.should_go_home
    assert edge.command is None
    assert not edge.clutch_active
    adapter.go_home()
    np.testing.assert_allclose(adapter.get_ee_pose().position, [0.30, 0.0, 0.20])

    held = pipe.process(_packet(5, [0.05, 1.2, 0.3], trigger=0.0, right_primary=True))
    assert not held.should_go_home


def test_grip_does_not_enable_motion():
    config = _config()
    adapter = DummyRobotAdapter.from_config(config["robot"])
    pipe = TeleopPipeline(config, adapter.get_ee_pose)
    result = pipe.process(_packet(1, [0.0, 1.2, 0.3], trigger=0.0, grip=1.0))
    assert result.command is None
    assert not result.clutch_active
