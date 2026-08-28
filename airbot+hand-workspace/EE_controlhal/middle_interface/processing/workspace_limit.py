"""Axis-aligned workspace box. Values are examples; retune before real hardware."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from protocol.pico_packet import Pose


@dataclass
class AxisLimit:
    min: float
    max: float

    def clamp(self, value: float) -> float:
        return float(np.clip(value, self.min, self.max))

    def contains(self, value: float) -> bool:
        return self.min <= value <= self.max


class WorkspaceLimit:
    def __init__(self, enabled: bool, x: AxisLimit, y: AxisLimit, z: AxisLimit) -> None:
        self.enabled = enabled
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_config(cls, config: Mapping[str, Any] | None) -> "WorkspaceLimit":
        cfg = dict(config or {})
        enabled = bool(cfg.get("enabled", False))

        def _axis(name: str, default_min: float, default_max: float) -> AxisLimit:
            block = cfg.get(name) or {}
            return AxisLimit(
                min=float(block.get("min", default_min)),
                max=float(block.get("max", default_max)),
            )

        return cls(
            enabled=enabled,
            x=_axis("x", -0.5, 0.5),
            y=_axis("y", -0.5, 0.5),
            z=_axis("z", 0.05, 0.8),
        )

    def contains(self, position: np.ndarray) -> bool:
        if not self.enabled:
            return True
        p = np.asarray(position, dtype=np.float64).reshape(3)
        return self.x.contains(p[0]) and self.y.contains(p[1]) and self.z.contains(p[2])

    def clamp_pose(self, pose: Pose) -> tuple[Pose, bool]:
        """Returns (possibly clamped pose, exceeded)."""
        if not self.enabled:
            return pose.copy(), False
        p = np.asarray(pose.position, dtype=np.float64).reshape(3)
        exceeded = not self.contains(p)
        clamped = np.array(
            [self.x.clamp(p[0]), self.y.clamp(p[1]), self.z.clamp(p[2])],
            dtype=np.float64,
        )
        return Pose(position=clamped, rotation=pose.rotation.copy()), exceeded
