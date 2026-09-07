"""Yaw-only operator frame so walking does not move a held-still hand.

teleop_standard: +X forward, +Y left, +Z up.

yaw_source:
  tracker — HMD/puck local +X (after OpenXR normalize) projected onto the
            floor. Right for a neck-worn HMD. Wrong for a belt-mounted
            tracker: the puck's face is rarely "operator forward", so
            hand-forward becomes robot left/right.
  world   — origin follows the body tracker; yaw is the teleop world frame
            (optional yaw_offset_deg). Use this when the body tracker is
            clipped to the waist.
"""

from __future__ import annotations

import math

import numpy as np

from processing.math_pose import as_vec3, matrix_from_quat, quat_from_matrix
from protocol.pico_packet import Pose

_EPS = 1e-6


class HeadBase:
    def __init__(self, yaw_source: str = "tracker", yaw_offset_deg: float = 0.0) -> None:
        source = str(yaw_source or "tracker").lower()
        if source not in {"tracker", "world"}:
            raise ValueError(f"unknown head_base yaw_source: {yaw_source!r}")
        self.yaw_source = source
        self.yaw_offset_deg = float(yaw_offset_deg)
        self._forward = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        self._origin = np.zeros(3, dtype=np.float64)
        self._rotation = np.eye(3, dtype=np.float64)
        self._has_ref = False

    def reset(self) -> None:
        self._forward = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        self._origin = np.zeros(3, dtype=np.float64)
        self._rotation = np.eye(3, dtype=np.float64)
        self._has_ref = False

    def relative(self, hmd: Pose | None, hmd_valid: bool, hand: Pose) -> Pose:
        rotation, origin = self._frame(hmd, hmd_valid)
        p_rel = rotation.T @ (as_vec3(hand.position) - origin)
        r_rel = rotation.T @ matrix_from_quat(hand.rotation)
        return Pose(position=p_rel, rotation=quat_from_matrix(r_rel))

    def _offset_rotation(self) -> np.ndarray:
        rad = math.radians(self.yaw_offset_deg)
        c, s = math.cos(rad), math.sin(rad)
        # teleop: +X forward, +Y left. Positive offset = operator turned left.
        return np.array(
            [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def _frame(self, hmd: Pose | None, hmd_valid: bool) -> tuple[np.ndarray, np.ndarray]:
        origin = as_vec3(hmd.position) if (hmd is not None and hmd_valid) else None
        if self.yaw_source == "world":
            rotation = self._offset_rotation()
            if origin is None:
                if self._has_ref:
                    return self._rotation, self._origin
                return rotation, np.zeros(3, dtype=np.float64)
            self._rotation = rotation
            self._origin = origin
            self._has_ref = True
            return rotation, origin
        if hmd is not None and hmd_valid:
            rotation = self._yaw_rotation(hmd) @ self._offset_rotation()
            assert origin is not None
            self._rotation = rotation
            self._origin = origin
            self._has_ref = True
            return rotation, origin
        if self._has_ref:
            return self._rotation, self._origin
        return self._offset_rotation(), np.zeros(3, dtype=np.float64)

    def _yaw_rotation(self, hmd: Pose) -> np.ndarray:
        forward = matrix_from_quat(hmd.rotation) @ np.array([1.0, 0.0, 0.0], dtype=np.float64)
        forward[2] = 0.0
        norm = float(np.linalg.norm(forward))
        if norm < _EPS:
            forward = self._forward.copy()
        else:
            forward = forward / norm
        self._forward = forward
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        left = np.cross(up, forward)
        left_norm = float(np.linalg.norm(left))
        if left_norm < _EPS:
            left = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        else:
            left = left / left_norm
        return np.column_stack((forward, left, up))
