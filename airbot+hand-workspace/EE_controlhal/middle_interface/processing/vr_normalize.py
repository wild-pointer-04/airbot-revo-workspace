"""Device tracking space → teleop_standard (+X forward, +Y left, +Z up).

PICO Unity uses Unity XR InputDevices: +X right, +Y up, +Z forward.
Raw OpenXR Stage convention is +X right, +Y up, -Z forward.

libsurvive world frame is NOT OpenXR Stage despite both being SteamVR tracking:
the global solve references one base station and Y is NOT guaranteed up. Empirical
measurement (2026-09-04) with gen-1 lighthouses at 1.735m baseline found the
coordinate frame varies per setup, requiring per-installation calibration via
probe_axes.py. This build uses the measured transform; re-calibrate if base
stations are moved or re-mounted."""

from __future__ import annotations

import numpy as np

from processing.math_pose import as_quat, as_vec3, matrix_from_quat, quat_from_matrix
from protocol.pico_packet import Pose

# OpenXR Stage → teleop_standard
OPENXR_TO_TELEOP_R = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

# Unity XR tracking → teleop_standard
UNITY_TO_TELEOP_R = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

# libsurvive world → teleop_standard (empirically measured 2026-09-04)
# Baseline 1.735m, gen-1 lighthouses. Re-calibrate if base stations move.
# Final verification 2026-09-04: operator behind robot, direct motion test with real arm:
#   tracker 上下 → 臂前后 → libsurvive z 变化 → teleop_x = -libsurvive_z
#   tracker 左右 → 臂左右 → libsurvive y 变化 → teleop_y = -libsurvive_y
#   tracker 前后 → 臂上下 → libsurvive x 变化 → teleop_z = -libsurvive_x
LIBSURVIVE_TO_TELEOP_R = np.array(
    [
        [0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)


def openxr_to_teleop_position(position: np.ndarray) -> np.ndarray:
    x, y, z = as_vec3(position)
    return np.array([-z, -x, y], dtype=np.float64)


def openxr_to_teleop_rotation(rotation_xyzw: np.ndarray) -> np.ndarray:
    rotation = OPENXR_TO_TELEOP_R @ matrix_from_quat(as_quat(rotation_xyzw)) @ OPENXR_TO_TELEOP_R.T
    return quat_from_matrix(rotation)


def openxr_to_teleop_pose(pose: Pose) -> Pose:
    return Pose(
        position=openxr_to_teleop_position(pose.position),
        rotation=openxr_to_teleop_rotation(pose.rotation),
    )


def unity_to_teleop_position(position: np.ndarray) -> np.ndarray:
    x, y, z = as_vec3(position)
    return np.array([z, -x, y], dtype=np.float64)


def unity_to_teleop_rotation(rotation_xyzw: np.ndarray) -> np.ndarray:
    rotation = UNITY_TO_TELEOP_R @ matrix_from_quat(as_quat(rotation_xyzw)) @ UNITY_TO_TELEOP_R.T
    return quat_from_matrix(rotation)


def unity_to_teleop_pose(pose: Pose) -> Pose:
    return Pose(
        position=unity_to_teleop_position(pose.position),
        rotation=unity_to_teleop_rotation(pose.rotation),
    )


def libsurvive_to_teleop_position(position: np.ndarray) -> np.ndarray:
    x, y, z = as_vec3(position)
    return np.array([-z, -y, -x], dtype=np.float64)


def libsurvive_to_teleop_rotation(rotation_xyzw: np.ndarray) -> np.ndarray:
    rotation = LIBSURVIVE_TO_TELEOP_R @ matrix_from_quat(as_quat(rotation_xyzw)) @ LIBSURVIVE_TO_TELEOP_R.T
    return quat_from_matrix(rotation)


def libsurvive_to_teleop_pose(pose: Pose) -> Pose:
    return Pose(
        position=libsurvive_to_teleop_position(pose.position),
        rotation=libsurvive_to_teleop_rotation(pose.rotation),
    )


def to_teleop_pose(pose: Pose, vr_space: str) -> Pose:
    space = str(vr_space or "unity").lower()
    if space in {"unity", "pico", "pico_unity"}:
        return unity_to_teleop_pose(pose)
    if space in {"libsurvive", "vive"}:
        return libsurvive_to_teleop_pose(pose)
    if space in {"openxr", "pico_openxr", "steamvr"}:
        return openxr_to_teleop_pose(pose)
    if space in {"teleop", "teleop_standard", "identity"}:
        return pose.copy()
    raise ValueError(f"unknown vr_space: {vr_space!r}")
