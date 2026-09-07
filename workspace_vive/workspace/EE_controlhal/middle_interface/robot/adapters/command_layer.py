"""Bridge middle_interface RobotCommand → Robot_control_interface ArmController."""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import Any, Mapping

from protocol.pico_packet import Pose
from protocol.robot_command import RobotCommand
from robot.base_robot_adapter import BaseRobotAdapter

logger = logging.getLogger(__name__)

_COMMAND_LAYER_ROOT = Path(__file__).resolve().parents[3] / "Robot_control_interface"


def _ensure_command_layer_path() -> Path:
    root = _COMMAND_LAYER_ROOT.resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"Robot_control_interface not found: {root}")
    path = str(root)
    if path not in sys.path:
        sys.path.insert(0, path)
    return root


class CommandLayerAdapter(BaseRobotAdapter):
    def __init__(self, controller) -> None:
        self._arm = controller

    @classmethod
    def from_config(cls, robot_cfg: Mapping[str, Any] | None) -> "CommandLayerAdapter":
        _ensure_command_layer_path()
        from factory import create_controller

        cfg = dict(robot_cfg or {})
        layer = cfg.get("command_layer")
        if isinstance(layer, str):
            import yaml

            path = Path(layer)
            if not path.is_absolute():
                path = Path(__file__).resolve().parents[2] / path
            with open(path, encoding="utf-8") as handle:
                layer_cfg = yaml.safe_load(handle)
            if not isinstance(layer_cfg, dict):
                raise ValueError(f"command_layer config {path} must be a mapping")
        elif isinstance(layer, Mapping):
            layer_cfg = dict(layer)
        else:
            layer_cfg = {"backend": "dummy"}
        return cls(create_controller(layer_cfg))

    def connect(self) -> None:
        self._arm.connect()

    def disconnect(self) -> None:
        self._arm.disconnect()

    def enable(self) -> None:
        return None

    def disable(self) -> None:
        self._arm.stop()

    def get_ee_pose(self) -> Pose:
        state = self._arm.get_state()
        return Pose(position=state.position.copy(), rotation=state.rotation.copy())

    def send_ee_target(self, command: RobotCommand) -> None:
        from arm_types import ArmCommand, MotionMode

        self._arm.send_command(
            ArmCommand(
                position=command.position,
                rotation=command.rotation,
                gripper=command.gripper,
                mode=MotionMode.END_CONTROL,
                enabled=command.enabled,
                source=command.source,
            )
        )

    def stop(self) -> None:
        self._arm.stop()

    def go_home(self) -> None:
        from arm_types import MotionMode

        self._arm.set_mode(MotionMode.GO_HOME)
        logger.info("[ROBOT] go_home")

    def is_connected(self) -> bool:
        return bool(self._arm.is_connected())

    def get_arm_state(self) -> dict:
        import time

        import numpy as np

        state = self._arm.get_state()
        return {
            "observation.state.arm_joint": np.asarray(state.joint_pos, dtype=np.float32).reshape(-1),
            "observation.state.arm_ee_position": np.asarray(state.position, dtype=np.float32).reshape(3),
            "observation.state.arm_ee_orientation": np.asarray(state.rotation, dtype=np.float32).reshape(4),
            "observation.state.arm_valid": np.array(
                [1.0 if state.connected else 0.0], dtype=np.float32
            ),
            "host_monotonic_ns": int(state.stamp_ns or time.monotonic_ns()),
        }
