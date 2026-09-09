"""teleop_standard → robot command frame (HAL spec). Device remap is vr_normalize."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from processing.math_pose import as_quat, as_vec3, matrix_from_quat, quat_from_matrix, quat_normalize
from protocol.pico_packet import Pose

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
_ROTATION_TOL = 1e-6


class CoordinateTransform:
    """Maps vectors with a 3x3 matrix built from YAML axis_map.

    robot_vec = R @ pico_vec
    robot_R   = R @ pico_R @ R.T
    """

    def __init__(self, rotation_matrix: np.ndarray) -> None:
        self.R = rotation_matrix_from_values(rotation_matrix, name="transform rotation")

    @classmethod
    def from_config(cls, config: Mapping[str, Any] | None) -> "CoordinateTransform":
        cfg = dict(config or {})
        matrix = cfg.get("rotation_matrix")
        if matrix is not None:
            return cls(matrix)
        axis_map = cfg.get("axis_map") or {
            "x": ["x", 1.0],
            "y": ["y", 1.0],
            "z": ["z", 1.0],
        }
        return cls(rotation_matrix_from_axis_map(axis_map))

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


def rotation_matrix_from_values(values: Any, *, name: str) -> np.ndarray:
    """Validate a proper 3D rotation read from configuration."""
    try:
        rotation = np.asarray(values, dtype=np.float64).reshape(3, 3)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a 3x3 matrix") from exc
    if not np.isfinite(rotation).all():
        raise ValueError(f"{name} must contain only finite values")
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=_ROTATION_TOL):
        raise ValueError(f"{name} must be orthonormal")
    if not np.isclose(float(np.linalg.det(rotation)), 1.0, atol=_ROTATION_TOL):
        raise ValueError(f"{name} must have determinant +1")
    return rotation


def rotation_matrix_from_axis_map(axis_map: Mapping[str, Any], *, name: str = "axis_map") -> np.ndarray:
    """Build a proper rotation from ``out_x: [input_axis, sign]`` entries."""
    if not isinstance(axis_map, Mapping):
        raise ValueError(f"{name} must be a mapping")
    r = np.zeros((3, 3), dtype=np.float64)
    for output_axis in _AXIS_INDEX:
        if output_axis not in axis_map:
            raise ValueError(f"{name}.{output_axis} is required")
        spec = axis_map[output_axis]
        if not isinstance(spec, (list, tuple)) or len(spec) != 2:
            raise ValueError(f"{name}.{output_axis} must be [input_axis, sign]")
        input_axis, sign = spec[0], float(spec[1])
        if input_axis not in _AXIS_INDEX:
            raise ValueError(f"invalid input axis {input_axis!r} in {name}")
        if sign not in {-1.0, 1.0}:
            raise ValueError(f"{name}.{output_axis} sign must be -1 or 1")
        r[_AXIS_INDEX[output_axis], _AXIS_INDEX[input_axis]] = sign
    return rotation_matrix_from_values(r, name=name)
