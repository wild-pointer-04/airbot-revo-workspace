"""Pose and tracking types for the OpenVR VIVE backend.

Quaternion convention in this package: [x, y, z, w].
Transforms are 4x4 SE(3) matrices T_A_B: maps B-frame points into A.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np


class TrackingStatus(str, Enum):
    OK = "OK"
    INVALID = "INVALID"
    DISCONNECTED = "DISCONNECTED"
    STALE = "STALE"


@dataclass
class TrackerPose:
    """One OpenVR GenericTracker sample.

    transform
        T_world_tracker, shape (4, 4). Rotation in [:3,:3], translation (m)
        in [:3,3], bottom row [0,0,0,1].

    position
        Same as transform[:3,3], shape (3,).

    quaternion
        Unit quaternion [x, y, z, w] of transform[:3,:3].

    role
        Logical role from tracker.yaml (e.g. body, right_wrist), or "" if
        the device is discovered but not mapped.
    """

    serial: str
    role: str
    timestamp: float
    transform: np.ndarray
    position: np.ndarray
    quaternion: np.ndarray
    pose_valid: bool
    device_connected: bool
    tracking_result: Optional[int] = None
    device_index: Optional[int] = None
    status: TrackingStatus = TrackingStatus.INVALID

    def copy(self) -> "TrackerPose":
        return TrackerPose(
            serial=self.serial,
            role=self.role,
            timestamp=self.timestamp,
            transform=np.array(self.transform, dtype=np.float64, copy=True),
            position=np.array(self.position, dtype=np.float64, copy=True),
            quaternion=np.array(self.quaternion, dtype=np.float64, copy=True),
            pose_valid=self.pose_valid,
            device_connected=self.device_connected,
            tracking_result=self.tracking_result,
            device_index=self.device_index,
            status=self.status,
        )


@dataclass(frozen=True)
class DiscoveredTracker:
    serial: str
    device_index: int
    connected: bool
    pose_valid: bool
    role: str = ""
