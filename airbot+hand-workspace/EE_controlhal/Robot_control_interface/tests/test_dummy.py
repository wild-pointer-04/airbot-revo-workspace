"""Dummy backend stores the latest absolute command."""

from __future__ import annotations

import numpy as np
import yaml

from factory import create_controller
from arm_types import ArmCommand, MotionMode


def test_dummy_send_and_stop():
    with open("config/default.yaml", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    arm = create_controller(config)
    arm.connect()
    start = arm.get_state().position.copy()
    cmd = ArmCommand(
        position=start + np.array([0.05, 0.0, 0.0]),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
        mode=MotionMode.END_CONTROL,
    )
    arm.send_command(cmd)
    np.testing.assert_allclose(arm.get_state().position, cmd.position)
    arm.stop()
    assert arm.get_state().mode == MotionMode.PROTECT
    arm.disconnect()
    assert not arm.is_connected()
