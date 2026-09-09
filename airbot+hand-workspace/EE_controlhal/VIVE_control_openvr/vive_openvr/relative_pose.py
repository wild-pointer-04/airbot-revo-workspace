"""Body-relative SE(3): hand expressed in the body tracker frame.

Given SteamVR world poses:

    T_world_body  = {}^W T_B
    T_world_hand  = {}^W T_H

the body-relative hand pose is:

    T_body_hand = inv(T_world_body) @ T_world_hand
                = {}^B T_H

T_A_B maps points expressed in frame B into frame A (metres, [x,y,z,w] quat).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from vive_openvr.pose_utils import (
    invert_se3,
    rotation_matrix_to_quaternion,
    transform_rotation,
    transform_translation,
)
from vive_openvr.types import TrackerPose


@dataclass
class RelativePose:
    """Hand (or other) pose expressed in the body frame."""

    transform: np.ndarray  # T_body_hand, 4x4
    position: np.ndarray  # metres
    quaternion: np.ndarray  # [x, y, z, w]
    body_valid: bool
    hand_valid: bool
    timestamp: float


def body_relative_transform(
    T_world_body: np.ndarray,
    T_world_hand: np.ndarray,
) -> np.ndarray:
    """Compute T_body_hand = inv(T_world_body) @ T_world_hand."""
    T_wb = np.asarray(T_world_body, dtype=np.float64).reshape(4, 4)
    T_wh = np.asarray(T_world_hand, dtype=np.float64).reshape(4, 4)
    return invert_se3(T_wb) @ T_wh


class BodyRelativePose:
    """Compute hand pose relative to the body tracker."""

    def compute(
        self,
        body: Optional[TrackerPose],
        hand: Optional[TrackerPose],
    ) -> Optional[RelativePose]:
        if body is None or hand is None:
            return None
        body_ok = bool(body.pose_valid and body.device_connected)
        hand_ok = bool(hand.pose_valid and hand.device_connected)
        if not (body_ok and hand_ok):
            # Still return geometry if both transforms exist; mark validity.
            if body.transform is None or hand.transform is None:
                return None
        T = body_relative_transform(body.transform, hand.transform)
        return RelativePose(
            transform=T,
            position=transform_translation(T),
            quaternion=rotation_matrix_to_quaternion(transform_rotation(T)),
            body_valid=body_ok,
            hand_valid=hand_ok,
            timestamp=float(max(body.timestamp, hand.timestamp)),
        )
