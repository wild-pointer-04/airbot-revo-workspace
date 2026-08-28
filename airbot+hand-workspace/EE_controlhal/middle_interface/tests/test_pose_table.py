"""Robot-command table shows adapter target, not raw headset pose."""

from __future__ import annotations

import numpy as np

from protocol.pico_packet import Pose
from protocol.robot_command import RobotCommand
from tools.pose_table import render


def test_render_command_uses_robot_target():
    cmd = RobotCommand(
        timestamp_ns=1,
        position=np.array([0.31, -0.05, 0.20]),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
        gripper=0.4,
        enabled=True,
        source="pico_right",
    )
    home = Pose(np.array([0.30, 0.0, 0.20]), np.array([0.0, 0.0, 0.0, 1.0]))
    base = Pose(np.array([0.30, 0.0, 0.20]), np.array([0.0, 0.0, 0.0, 1.0]))
    text = render(
        hz=60.0,
        seq=10,
        loss=0,
        state="TELEOP_ACTIVE",
        reason="none",
        sent=True,
        clutch=True,
        tracking=True,
        deadman=0.9,
        grip_raw=0.4,
        command=cmd,
        ee_now=Pose(cmd.position.copy(), cmd.rotation.copy()),
        home=home,
        baseline=base,
        delta_pos=np.array([0.01, -0.05, 0.0]),
    )
    assert "EE_cmd" in text
    assert "EE_base" in text
    assert "d_vr" in text
    assert "0.310" in text
    assert "-0.050" in text
    assert "SEND" in text
    assert "HMD" not in text


def test_render_hold_shows_last_not_live_cmd():
    cmd = RobotCommand(
        timestamp_ns=1,
        position=np.array([0.31, 0.0, 0.20]),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
        gripper=0.0,
        enabled=True,
    )
    text = render(
        hz=60.0,
        seq=11,
        loss=0,
        state="PAUSED",
        reason="none",
        sent=False,
        clutch=False,
        tracking=True,
        deadman=0.0,
        grip_raw=0.0,
        command=cmd,
        ee_now=None,
        home=None,
    )
    assert "HOLD" in text
    assert "EE_hold" in text
    assert "EE_cmd" not in text
    assert "not updating" in text


def test_render_hold_without_command():
    text = render(
        hz=0.0,
        seq=None,
        loss=0,
        state="READY",
        reason="none",
        sent=False,
        clutch=False,
        tracking=True,
        deadman=0.0,
        grip_raw=0.0,
        command=None,
        ee_now=None,
        home=None,
    )
    assert "HOLD" in text
    assert "not sending" in text
