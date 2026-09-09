"""Yaw-only body frame for two-tracker teleoperation.

teleop_standard: +X forward, +Y left, +Z up.  ``tracker_to_body`` maps
coordinates from the tracker housing into that body frame.  It is an explicit
mounting calibration: the body tracker need not be installed with its own
local +X facing forward.
"""

from __future__ import annotations

import numpy as np

from processing.coordinate_transform import rotation_matrix_from_axis_map, rotation_matrix_from_values
from processing.math_pose import as_vec3, matrix_from_quat, quat_from_matrix
from protocol.pico_packet import Pose

_EPS = 1e-6


class HeadBase:
    def __init__(self, tracker_to_body: np.ndarray | None = None) -> None:
        self._tracker_to_body = (
            np.eye(3, dtype=np.float64)
            if tracker_to_body is None
            else rotation_matrix_from_values(tracker_to_body, name="head_tracker_to_body")
        )
        self._forward = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        self._origin = np.zeros(3, dtype=np.float64)
        self._rotation = np.eye(3, dtype=np.float64)
        self._has_ref = False

    @classmethod
    def from_config(cls, teleop: dict | None) -> "HeadBase":
        """Create the body frame from ``teleop.head_tracker_to_body``.

        The legacy YAML mapping uses the same convention as ``axis_map``:
        ``x: ["z", -1]`` means body +X is tracker -Z.  For a measured
        mounting offset it may instead contain ``rotation_matrix``: a nested
        3x3 proper rotation E where ``body_vector = E @ tracker_vector``.
        Omit the entire setting only when the tracker housing is physically
        mounted with +X forward, +Y left, and +Z up.
        """
        config = dict(teleop or {})
        mapping = config.get("head_tracker_to_body")
        if mapping is None:
            return cls()
        if not isinstance(mapping, dict):
            raise ValueError("head_tracker_to_body must be a mapping")
        rotation_matrix = mapping.get("rotation_matrix")
        if rotation_matrix is not None:
            return cls(rotation_matrix)
        return cls(rotation_matrix_from_axis_map(mapping, name="head_tracker_to_body"))

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
        tracker_world = matrix_from_quat(hmd.rotation)
        # b = E t, so a body-local vector b has tracker coordinates E.T @ b.
        body_world = tracker_world @ self._tracker_to_body.T
        forward = body_world @ np.array([1.0, 0.0, 0.0], dtype=np.float64)
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
