"""pico_teleop_v1 packet parse/serialize.

Quaternion order is [x, y, z, w]. Invalid packets raise ValueError from
from_dict; from_json_bytes logs a warning and returns None.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

logger = logging.getLogger(__name__)

PROTOCOL_NAME = "pico_teleop"
PROTOCOL_VERSION = 1


def _require_finite(name: str, values: np.ndarray) -> np.ndarray:
    if values.size == 0 or not np.all(np.isfinite(values)):
        raise ValueError(f"{name} contains NaN/Inf or is empty")
    return values


def _as_vec(name: str, raw: Any, length: int) -> np.ndarray:
    if not isinstance(raw, (list, tuple)):
        raise ValueError(f"{name} must be a list of length {length}")
    if len(raw) != length:
        raise ValueError(f"{name} must have length {length}, got {len(raw)}")
    try:
        arr = np.asarray(raw, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} is not numeric") from exc
    return _require_finite(name, arr)


def _as_bool(name: str, raw: Any) -> bool:
    if not isinstance(raw, bool):
        raise ValueError(f"{name} must be bool")
    return raw


def _as_float01(name: str, raw: Any) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} is not a float") from exc
    if not np.isfinite(value):
        raise ValueError(f"{name} is NaN/Inf")
    return float(np.clip(value, 0.0, 1.0))


def _as_int(name: str, raw: Any) -> int:
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ValueError(f"{name} must be an integer")
    if not np.isfinite(raw):
        raise ValueError(f"{name} is NaN/Inf")
    return int(raw)


@dataclass
class Pose:
    """Position in meters, rotation quaternion [x, y, z, w]."""

    position: np.ndarray
    rotation: np.ndarray

    def copy(self) -> "Pose":
        return Pose(self.position.copy(), self.rotation.copy())


@dataclass
class HeadState:
    valid: bool
    position: np.ndarray
    rotation: np.ndarray

    @property
    def pose(self) -> Pose:
        return Pose(self.position, self.rotation)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "HeadState":
        if not isinstance(data, Mapping):
            raise ValueError("hmd must be an object")
        return cls(
            valid=_as_bool("hmd.valid", data.get("valid")),
            position=_as_vec("hmd.position", data.get("position"), 3),
            rotation=_as_vec("hmd.rotation", data.get("rotation"), 4),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "position": self.position.tolist(),
            "rotation": self.rotation.tolist(),
        }


@dataclass
class ControllerState:
    valid: bool
    position: np.ndarray
    rotation: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    trigger: float
    grip: float
    primary_button: bool
    secondary_button: bool
    joystick: np.ndarray

    @property
    def pose(self) -> Pose:
        return Pose(self.position, self.rotation)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any], prefix: str) -> "ControllerState":
        if not isinstance(data, Mapping):
            raise ValueError(f"{prefix} must be an object")
        lin = data.get("linear_velocity", [0.0, 0.0, 0.0])
        ang = data.get("angular_velocity", [0.0, 0.0, 0.0])
        return cls(
            valid=_as_bool(f"{prefix}.valid", data.get("valid")),
            position=_as_vec(f"{prefix}.position", data.get("position"), 3),
            rotation=_as_vec(f"{prefix}.rotation", data.get("rotation"), 4),
            linear_velocity=_as_vec(f"{prefix}.linear_velocity", lin, 3),
            angular_velocity=_as_vec(f"{prefix}.angular_velocity", ang, 3),
            trigger=_as_float01(f"{prefix}.trigger", data.get("trigger", 0.0)),
            grip=_as_float01(f"{prefix}.grip", data.get("grip", 0.0)),
            primary_button=_as_bool(
                f"{prefix}.primary_button", data.get("primary_button", False)
            ),
            secondary_button=_as_bool(
                f"{prefix}.secondary_button", data.get("secondary_button", False)
            ),
            joystick=_as_vec(f"{prefix}.joystick", data.get("joystick", [0.0, 0.0]), 2),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "position": self.position.tolist(),
            "rotation": self.rotation.tolist(),
            "linear_velocity": self.linear_velocity.tolist(),
            "angular_velocity": self.angular_velocity.tolist(),
            "trigger": self.trigger,
            "grip": self.grip,
            "primary_button": self.primary_button,
            "secondary_button": self.secondary_button,
            "joystick": self.joystick.tolist(),
        }


@dataclass
class PicoTeleopPacket:
    protocol: str
    version: int
    seq: int
    device_time_ns: int
    wall_time_unix_ms: int
    hmd: HeadState
    left_controller: ControllerState
    right_controller: ControllerState
    receive_monotonic_ns: int = 0
    receive_wall_time: float = 0.0
    source_addr: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    def controller(self, which: str) -> ControllerState:
        if which == "left":
            return self.left_controller
        if which == "right":
            return self.right_controller
        raise ValueError(f"unknown controller: {which}")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "PicoTeleopPacket":
        if not isinstance(data, Mapping):
            raise ValueError("packet must be a JSON object")
        protocol = data.get("protocol")
        if protocol != PROTOCOL_NAME:
            raise ValueError(f"unexpected protocol: {protocol!r}")
        version = _as_int("version", data.get("version"))
        if version != PROTOCOL_VERSION:
            raise ValueError(f"unsupported protocol version: {version}")
        for key in (
            "seq",
            "device_time_ns",
            "wall_time_unix_ms",
            "hmd",
            "left_controller",
            "right_controller",
        ):
            if key not in data:
                raise ValueError(f"missing field: {key}")
        return cls(
            protocol=protocol,
            version=version,
            seq=_as_int("seq", data["seq"]),
            device_time_ns=_as_int("device_time_ns", data["device_time_ns"]),
            wall_time_unix_ms=_as_int("wall_time_unix_ms", data["wall_time_unix_ms"]),
            hmd=HeadState.from_dict(data["hmd"]),
            left_controller=ControllerState.from_dict(
                data["left_controller"], "left_controller"
            ),
            right_controller=ControllerState.from_dict(
                data["right_controller"], "right_controller"
            ),
        )

    @classmethod
    def from_json_bytes(cls, raw: bytes) -> PicoTeleopPacket | None:
        try:
            text = raw.decode("utf-8")
            data = json.loads(text)
            return cls.from_dict(data)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError, TypeError) as exc:
            logger.warning("drop packet: %s", exc)
            return None

    def to_dict(self) -> dict[str, Any]:
        return {
            "protocol": self.protocol,
            "version": self.version,
            "seq": self.seq,
            "device_time_ns": self.device_time_ns,
            "wall_time_unix_ms": self.wall_time_unix_ms,
            "hmd": self.hmd.to_dict(),
            "left_controller": self.left_controller.to_dict(),
            "right_controller": self.right_controller.to_dict(),
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))

    def stamp_receive(self, source_addr: str = "") -> None:
        self.receive_monotonic_ns = time.monotonic_ns()
        self.receive_wall_time = time.time()
        self.source_addr = source_addr
