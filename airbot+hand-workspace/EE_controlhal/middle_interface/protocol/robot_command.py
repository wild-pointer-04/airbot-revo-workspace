"""Unified robot command produced by middle_interface."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class RobotCommand:
    """End-effector target in the robot frame.

    rotation is quaternion [x, y, z, w].
    gripper is 0.0 (open) .. 1.0 (closed); adapters interpret the mapping.
    """

    timestamp_ns: int
    position: np.ndarray
    rotation: np.ndarray
    gripper: float
    enabled: bool
    source: str = "pico_right"

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=np.float64).reshape(3)
        self.rotation = np.asarray(self.rotation, dtype=np.float64).reshape(4)
        self.gripper = float(np.clip(self.gripper, 0.0, 1.0))
