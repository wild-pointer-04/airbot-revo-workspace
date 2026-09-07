"""In-process arm stand-in. Never opens ROS or CAN."""

from __future__ import annotations

import logging
import time
from typing import Any, Mapping

import numpy as np

from convert import pose_to_xyzrpy
from interface import ArmController
from arm_types import ArmCommand, ArmState, MotionMode

logger = logging.getLogger(__name__)


class DummyArmController(ArmController):
    def __init__(self, initial: ArmState) -> None:
        self._state = initial
        self._connected = False
        self._mode = MotionMode.PROTECT
        self.last_command: ArmCommand | None = None
        self.command_count = 0

    @classmethod
    def from_config(cls, config: Mapping[str, Any] | None) -> "DummyArmController":
        cfg = dict(config or {})
        dummy = dict(cfg.get("dummy") or {})
        pos = np.asarray(dummy.get("position", [0.30, 0.0, 0.20]), dtype=np.float64)
        rot = np.asarray(dummy.get("rotation", [0.0, 0.0, 0.0, 1.0]), dtype=np.float64)
        state = ArmState(
            position=pos,
            rotation=rot,
            xyzrpy=pose_to_xyzrpy(pos, rot),
            joint_pos=np.zeros(6, dtype=np.float64),
            connected=False,
            mode=MotionMode.PROTECT,
        )
        return cls(state)

    def connect(self) -> None:
        self._connected = True
        self._state.connected = True
        logger.info("[ARM] Dummy connected")

    def disconnect(self) -> None:
        self._connected = False
        self._state.connected = False
        logger.info("[ARM] Dummy disconnected")

    def get_state(self) -> ArmState:
        return ArmState(
            position=self._state.position.copy(),
            rotation=self._state.rotation.copy(),
            xyzrpy=self._state.xyzrpy.copy(),
            joint_pos=self._state.joint_pos.copy(),
            connected=self._connected,
            mode=self._mode,
            stamp_ns=time.monotonic_ns(),
        )

    def send_command(self, command: ArmCommand) -> None:
        if not self._connected:
            logger.warning("[ARM] Dummy send ignored: not connected")
            return
        if not command.enabled:
            self.stop()
            return
        self.last_command = command
        self.command_count += 1
        self._mode = command.mode
        self._state.position = command.position.copy()
        self._state.rotation = command.rotation.copy()
        self._state.xyzrpy = pose_to_xyzrpy(command.position, command.rotation)
        self._state.mode = command.mode

    def set_mode(self, mode: MotionMode) -> None:
        self._mode = MotionMode(int(mode))
        self._state.mode = self._mode
        if self._mode == MotionMode.GO_HOME:
            self._state.joint_pos = np.zeros(6, dtype=np.float64)
            logger.info("[ARM] Dummy GO_HOME")

    def stop(self) -> None:
        self._mode = MotionMode.PROTECT
        self._state.mode = MotionMode.PROTECT
        logger.info("[ARM] Dummy PROTECT at %s", np.round(self._state.position, 4).tolist())

    def is_connected(self) -> bool:
        return self._connected
