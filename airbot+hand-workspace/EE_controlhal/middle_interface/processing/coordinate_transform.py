"""teleop_standard → robot command frame (HAL spec). Device remap is vr_normalize."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from processing.math_pose import as_quat, as_vec3, matrix_from_quat, quat_from_matrix, quat_normalize
from protocol.pico_packet import Pose

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


class CoordinateTransform:
    """Maps vectors with a 3x3 matrix built from YAML axis_map.

    robot_vec = R @ pico_vec
    robot_R   = R @ pico_R @ R.T
    """

    def __init__(self, rotation_matrix: np.ndarray) -> None:
        self.R = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)

    @classmethod
    def from_config(cls, config: Mapping[str, Any] | None) -> "CoordinateTransform":
        cfg = dict(config or {})
        matrix = cfg.get("rotation_matrix")
        if matrix is not None:
            r = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
            return cls(r)
        axis_map = cfg.get("axis_map") or {
            "x": ["x", 1.0],
            "y": ["y", 1.0],
            "z": ["z", 1.0],
        }
        return cls(_matrix_from_axis_map(axis_map))

    def transform_position(self, position: np.ndarray) -> np.ndarray:
        return self.R @ as_vec3(position)

    def transform_rotation(self, rotation_xyzw: np.ndarray) -> np.ndarray:
        pico_r = matrix_from_quat(as_quat(rotation_xyzw))
        robot_r = self.R @ pico_r @ self.R.T
        return quat_normalize(quat_from_matrix(robot_r))

    def transform_pose(self, pose: Pose) -> Pose:
        return Pose(
            position=self.transform_position(pose.position),
            rotation=self.transform_rotation(pose.rotation),
        )


def _matrix_from_axis_map(axis_map: Mapping[str, Any]) -> np.ndarray:
    """axis_map.x: [pico_axis, sign] means robot_x = sign * pico_axis."""
    r = np.zeros((3, 3), dtype=np.float64)
    for robot_axis, spec in axis_map.items():
        if robot_axis not in _AXIS_INDEX:
            raise ValueError(f"invalid robot axis {robot_axis!r}")
        if not isinstance(spec, (list, tuple)) or len(spec) != 2:
            raise ValueError(f"axis_map.{robot_axis} must be [pico_axis, sign]")
        pico_axis, sign = spec[0], float(spec[1])
        if pico_axis not in _AXIS_INDEX:
            raise ValueError(f"invalid pico axis {pico_axis!r}")
        r[_AXIS_INDEX[robot_axis], _AXIS_INDEX[pico_axis]] = sign
    if abs(float(np.linalg.det(r))) < 1e-8:
        raise ValueError("transform axis_map is singular")
    return r
