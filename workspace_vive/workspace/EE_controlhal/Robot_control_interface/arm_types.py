"""Arm command / state. Absolute Cartesian only; no VR semantics."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np


class MotionMode(IntEnum):
    SOFT = 0
    GO_HOME = 1
    PROTECT = 2
    GRAVITY = 3
    END_CONTROL = 4
    JOINT = 5


@dataclass
class ArmCommand:
    """Absolute TCP target in the robot base frame.

    rotation is quaternion [x, y, z, w].
    """

    position: np.ndarray
    rotation: np.ndarray
    gripper: float = 0.0
    mode: MotionMode = MotionMode.END_CONTROL
    enabled: bool = True
    source: str = "command_layer"

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=np.float64).reshape(3)
        self.rotation = np.asarray(self.rotation, dtype=np.float64).reshape(4)
        self.gripper = float(np.clip(self.gripper, 0.0, 1.0))
        self.mode = MotionMode(int(self.mode))


@dataclass
class ArmState:
    position: np.ndarray
    rotation: np.ndarray
    xyzrpy: np.ndarray
    joint_pos: np.ndarray
    connected: bool
    mode: MotionMode | None = None
    stamp_ns: int = 0
    extra: dict = field(default_factory=dict)
