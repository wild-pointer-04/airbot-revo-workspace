"""Tracker → semantic-frame extrinsics (identity by default).

OpenVR tracker puck frame ≠ body / wrist semantic frame. Apply a fixed
SE(3) on the right:

    T_world_semantic = T_world_tracker @ T_tracker_semantic

Do not encode mount corrections as ad-hoc axis flips. Keep first-run
extrinsics as identity until a mount calibration is measured.
"""

from __future__ import annotations

from typing import Any, Mapping, Optional

import numpy as np

from vive_openvr.pose_utils import (
    identity_transform,
    quaternion_to_rotation_matrix,
    se3_from_rt,
    transform_rotation,
    transform_translation,
)
from vive_openvr.types import TrackerPose


def se3_from_translation_quat(
    translation: Any,
    quaternion_xyzw: Any,
) -> np.ndarray:
    """Build T_tracker_semantic from translation (m) and quat [x,y,z,w]."""
    t = np.asarray(translation, dtype=np.float64).reshape(3)
    q = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    return se3_from_rt(quaternion_to_rotation_matrix(q), t)


def load_role_extrinsics(config: Mapping[str, Any]) -> dict[str, np.ndarray]:
    """Parse config['extrinsics'][role] → {role: T_tracker_semantic}.

    Missing roles default to identity. Disabled / unknown roles are skipped.
    """
    raw = config.get("extrinsics") or {}
    if not isinstance(raw, Mapping):
        raise ValueError("extrinsics must be a mapping of role -> {translation, quaternion_xyzw}")
    out: dict[str, np.ndarray] = {}
    for role, row in raw.items():
        if row is None:
            out[str(role)] = identity_transform()
            continue
        if not isinstance(row, Mapping):
            raise ValueError(f"extrinsics.{role} must be a mapping or null")
        translation = row.get("translation", [0.0, 0.0, 0.0])
        quat = row.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        out[str(role)] = se3_from_translation_quat(translation, quat)
    return out


def apply_extrinsics_transform(
    T_world_tracker: np.ndarray,
    T_tracker_semantic: Optional[np.ndarray],
) -> np.ndarray:
    """Return T_world_semantic = T_world_tracker @ T_tracker_semantic.

    If T_tracker_semantic is None, returns a copy of T_world_tracker.
    """
    T_w = np.asarray(T_world_tracker, dtype=np.float64).reshape(4, 4)
    if T_tracker_semantic is None:
        return T_w.copy()
    T_ts = np.asarray(T_tracker_semantic, dtype=np.float64).reshape(4, 4)
    return T_w @ T_ts


def apply_pose_extrinsics(
    pose: TrackerPose,
    T_tracker_semantic: Optional[np.ndarray],
) -> TrackerPose:
    """Right-multiply extrinsics into a TrackerPose (position/quat refreshed)."""
    if T_tracker_semantic is None:
        return pose
    T = apply_extrinsics_transform(pose.transform, T_tracker_semantic)
    from vive_openvr.pose_utils import rotation_matrix_to_quaternion

    out = pose.copy()
    out.transform = T
    out.position = transform_translation(T)
    out.quaternion = rotation_matrix_to_quaternion(transform_rotation(T))
    return out


def extrinsics_for_role(
    role_extrinsics: Mapping[str, np.ndarray],
    role: str,
) -> np.ndarray:
    """Lookup with identity fallback."""
    T = role_extrinsics.get(role)
    if T is None:
        return identity_transform()
    return np.asarray(T, dtype=np.float64).reshape(4, 4)
