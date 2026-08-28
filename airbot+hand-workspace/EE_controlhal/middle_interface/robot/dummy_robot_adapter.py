"""In-process robot stand-in. Logs targets; never opens a hardware SDK."""

from __future__ import annotations

import logging
import time
from typing import Any, Mapping

import numpy as np

from protocol.pico_packet import Pose
from protocol.robot_command import RobotCommand
from robot.base_robot_adapter import BaseRobotAdapter

logger = logging.getLogger(__name__)


class DummyRobotAdapter(BaseRobotAdapter):
    def __init__(self, initial: Pose, log_period_sec: float = 1.0) -> None:
        self._home = initial.copy()
        self._pose = initial.copy()
        self._connected = False
        self._enabled = False
        self._log_period = log_period_sec
        self._last_log = 0.0
        self._command_count = 0
        self.last_command: RobotCommand | None = None

    @classmethod
    def from_config(cls, config: Mapping[str, Any] | None) -> "DummyRobotAdapter":
        cfg = dict(config or {})
        pos = np.asarray(
            cfg.get("dummy_initial_position", [0.30, 0.0, 0.20]), dtype=np.float64
        )
        rot = np.asarray(
            cfg.get("dummy_initial_rotation", [0.0, 0.0, 0.0, 1.0]), dtype=np.float64
        )
        return cls(Pose(position=pos, rotation=rot))

    def connect(self) -> None:
        self._connected = True
        logger.info("[ROBOT] Dummy connected")

    def disconnect(self) -> None:
        self._connected = False
        self._enabled = False
        logger.info("[ROBOT] Dummy disconnected")

    def enable(self) -> None:
        self._enabled = True

    def disable(self) -> None:
        self._enabled = False

    def get_ee_pose(self) -> Pose:
        return self._pose.copy()

    def send_ee_target(self, command: RobotCommand) -> None:
        if not self._connected:
            logger.warning("[ROBOT] Dummy send ignored: not connected")
            return
        self._enabled = True
        self._pose = Pose(command.position.copy(), command.rotation.copy())
        self.last_command = command
        self._command_count += 1
        now = time.monotonic()
        if now - self._last_log >= self._log_period:
            self._last_log = now
            logger.info(
                "[ROBOT] Dummy target: position=%s rotation=%s gripper=%.3f n=%s",
                np.round(command.position, 4).tolist(),
                np.round(command.rotation, 4).tolist(),
                command.gripper,
                self._command_count,
            )

    def stop(self) -> None:
        if self._enabled:
            logger.info(
                "[ROBOT] Dummy stop at position=%s",
                np.round(self._pose.position, 4).tolist(),
            )
        self._enabled = False

    def go_home(self) -> None:
        self._pose = self._home.copy()
        self._enabled = False
        logger.info(
            "[ROBOT] Dummy go_home position=%s",
            np.round(self._pose.position, 4).tolist(),
        )

    def is_connected(self) -> bool:
        return self._connected
