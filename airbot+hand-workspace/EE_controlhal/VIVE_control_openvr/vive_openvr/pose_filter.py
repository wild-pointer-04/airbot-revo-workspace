"""Optional EMA / SLERP pose filter for OpenVR samples.

Default is off — do not hide raw SteamVR jitter during first bring-up.
Quaternion convention: [x, y, z, w]. Filter does not change SE(3) convention.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from vive_openvr.pose_utils import (
    enforce_quaternion_continuity,
    normalize_quaternion,
    quaternion_to_rotation_matrix,
    se3_from_rt,
)
from vive_openvr.types import TrackerPose

_SLERP_LINEAR_DOT = 0.9995


def _slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    a = normalize_quaternion(q0)
    b = normalize_quaternion(q1)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > _SLERP_LINEAR_DOT:
        return normalize_quaternion(a + alpha * (b - a))
    theta = float(np.arccos(dot))
    sin_theta = float(np.sin(theta))
    w0 = float(np.sin((1.0 - alpha) * theta) / sin_theta)
    w1 = float(np.sin(alpha * theta) / sin_theta)
    return normalize_quaternion(w0 * a + w1 * b)


class PoseFilter:
    """Per-role EMA on translation + optional quaternion SLERP."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        position_alpha: float = 0.25,
        orientation_alpha: float = 0.25,
        filter_orientation: bool = True,
    ) -> None:
        self.enabled = bool(enabled)
        self.position_alpha = float(np.clip(position_alpha, 0.0, 1.0))
        self.orientation_alpha = float(np.clip(orientation_alpha, 0.0, 1.0))
        self.filter_orientation = bool(filter_orientation)
        self._pos: dict[str, np.ndarray] = {}
        self._quat: dict[str, np.ndarray] = {}

    def reset(self, role: Optional[str] = None) -> None:
        if role is None:
            self._pos.clear()
            self._quat.clear()
            return
        self._pos.pop(role, None)
        self._quat.pop(role, None)

    def update(self, role: str, pose: TrackerPose) -> TrackerPose:
        if not self.enabled or not pose.pose_valid:
            return pose
        key = str(role)
        p = np.asarray(pose.position, dtype=np.float64).reshape(3)
        q = normalize_quaternion(pose.quaternion)
        if key not in self._pos:
            self._pos[key] = p.copy()
            self._quat[key] = q.copy()
            return pose

        a_p = self.position_alpha
        filt_p = (1.0 - a_p) * self._pos[key] + a_p * p
        self._pos[key] = filt_p

        if self.filter_orientation:
            q = enforce_quaternion_continuity(q, self._quat[key])
            a_q = self.orientation_alpha
            filt_q = _slerp(self._quat[key], q, a_q)
            self._quat[key] = filt_q
        else:
            filt_q = q
            self._quat[key] = q

        out = pose.copy()
        out.position = filt_p
        out.quaternion = filt_q
        out.transform = se3_from_rt(quaternion_to_rotation_matrix(filt_q), filt_p)
        return out
