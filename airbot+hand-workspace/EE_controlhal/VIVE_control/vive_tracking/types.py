"""Core pose types.

Quaternion order is fixed project-wide as [w, x, y, z].
Never use [x, y, z, w] in this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True, slots=True)
class Vec3:
    x: float
    y: float
    z: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z]


@dataclass(frozen=True, slots=True)
class Quaternion:
    """Unit quaternion stored as [w, x, y, z]."""

    w: float
    x: float
    y: float
    z: float

    def as_wxyz(self) -> tuple[float, float, float, float]:
        return (self.w, self.x, self.y, self.z)

    def as_list(self) -> list[float]:
        return [self.w, self.x, self.y, self.z]


@dataclass(frozen=True, slots=True)
class Pose6DoF:
    position_m: Vec3
    orientation: Quaternion


@dataclass(frozen=True, slots=True)
class TrackerDevice:
    serial: str
    logical_name: str
    tracking_valid: bool
    source: str


@dataclass(frozen=True, slots=True)
class TrackerPose:
    """One pose sample from one tracker.

    sensor_timestamp
        Device / libsurvive capture time when available. Never fabricated.
        None means the backend did not provide a sensor clock.

    host_monotonic_ns
        Host receive time from time.monotonic_ns(), taken as soon as this
        process actually obtained the pose. Not wall-clock time.time().
    """

    serial: str
    logical_name: str
    position_m: Vec3
    orientation: Quaternion
    sensor_timestamp: Optional[float]
    host_monotonic_ns: int
    tracking_valid: bool
    sequence_id: int
    source: str

    def as_raw(self) -> TrackerPose:
        """Identity helper: this object is already the raw observation."""
        return self


IDENTITY_QUATERNION = Quaternion(1.0, 0.0, 0.0, 0.0)
ORIGIN = Vec3(0.0, 0.0, 0.0)
