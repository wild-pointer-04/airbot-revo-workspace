"""JSONL recorder for synchronized PICO and robot poses."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Any, TextIO

from protocol.pico_packet import ControllerState, PicoTeleopPacket, Pose

logger = logging.getLogger(__name__)


def _array(values: Any) -> list[float]:
    return [float(value) for value in values]


def _pose(pose: Pose | None) -> dict[str, list[float]] | None:
    if pose is None:
        return None
    return {"position": _array(pose.position), "rotation": _array(pose.rotation)}


def _controller(controller: ControllerState) -> dict[str, Any]:
    return {
        "valid": bool(controller.valid),
        "position": _array(controller.position),
        "rotation": _array(controller.rotation),
        "linear_velocity": _array(controller.linear_velocity),
        "angular_velocity": _array(controller.angular_velocity),
        "trigger": float(controller.trigger),
        "grip": float(controller.grip),
        "primary_button": bool(controller.primary_button),
        "secondary_button": bool(controller.secondary_button),
        "joystick": _array(controller.joystick),
    }


class PoseRecorder:
    def __init__(self, path: Path, selected_controller: str) -> None:
        self.path = path
        self.selected_controller = selected_controller
        self._file: TextIO = path.open("w", encoding="utf-8", buffering=1)
        self._count = 0

    @classmethod
    def from_config(
        cls, config: dict[str, Any] | None, selected_controller: str
    ) -> "PoseRecorder | None":
        cfg = dict(config or {})
        if not bool(cfg.get("enabled", False)):
            return None
        directory = Path(str(cfg.get("directory", "logs")))
        if not directory.is_absolute():
            directory = Path(__file__).resolve().parent / directory
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        timestamp += f"_{time.time_ns() % 1_000_000_000:09d}"
        filename = str(cfg.get("filename", f"vr_arm_{timestamp}.jsonl"))
        path = directory / filename
        recorder = cls(path, selected_controller)
        logger.info("[RECORD] writing synchronized VR/robot poses to %s", path)
        return recorder

    def record(
        self,
        packet: PicoTeleopPacket,
        actual_pose: Pose | None,
        result: Any,
        action: str,
    ) -> None:
        command = result.command
        row = {
            "record_time_unix_ns": time.time_ns(),
            "record_monotonic_ns": time.monotonic_ns(),
            "packet": {
                "seq": int(packet.seq),
                "device_time_ns": int(packet.device_time_ns),
                "wall_time_unix_ms": int(packet.wall_time_unix_ms),
                "receive_monotonic_ns": int(packet.receive_monotonic_ns),
                "receive_wall_time": float(packet.receive_wall_time),
                "source_addr": packet.source_addr,
            },
            "vr": {
                "selected_controller": self.selected_controller,
                "hmd": {
                    "valid": bool(packet.hmd.valid),
                    "position": _array(packet.hmd.position),
                    "rotation": _array(packet.hmd.rotation),
                },
                "left_controller": _controller(packet.left_controller),
                "right_controller": _controller(packet.right_controller),
            },
            "robot_actual_tcp_before_command": _pose(actual_pose),
            "command_target_tcp": (
                {
                    "position": _array(command.position),
                    "rotation": _array(command.rotation),
                    "gripper": float(command.gripper),
                    "enabled": bool(command.enabled),
                    "source": command.source,
                }
                if command is not None
                else None
            ),
            "teleop": {
                "action": action,
                "state": result.state.value,
                "reason": result.reason.value,
                "tracking_ok": bool(result.tracking_ok),
                "clutch_active": bool(result.clutch_active),
                "should_stop": bool(result.should_stop),
                "should_go_home": bool(result.should_go_home),
                "delta_pos": (
                    _array(result.delta_pos) if result.delta_pos is not None else None
                ),
            },
        }
        self._file.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
        self._count += 1

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()
            logger.info("[RECORD] saved %d pose samples to %s", self._count, self.path)
