"""Yaw-only head frame so walking/turning does not move a held-still hand.

teleop_standard: +X forward, +Y left, +Z up.
Head local +X is look-forward after OpenXR normalize. Pitch/roll are dropped.
"""

from __future__ import annotations

import numpy as np

from processing.math_pose import as_vec3, matrix_from_quat, quat_from_matrix
from protocol.pico_packet import Pose

_EPS = 1e-6


class HeadBase:
    def __init__(self) -> None:
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

    def _frame(self, hmd: Pose | None, hmd_valid: bool) -> tuple[np.ndarray, np.ndarray]:
        if hmd is not None and hmd_valid:
            rotation = self._yaw_rotation(hmd)
            origin = as_vec3(hmd.position)
            self._rotation = rotation
            self._origin = origin
            self._has_ref = True
            return rotation, origin
        if self._has_ref:
            return self._rotation, self._origin
        return np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64)

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
