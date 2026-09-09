"""Unit tests for body-relative SE(3) (no SteamVR)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_openvr.pose_utils import identity_transform, invert_se3, se3_from_rt
from vive_openvr.relative_pose import BodyRelativePose, body_relative_transform
from vive_openvr.types import TrackerPose, TrackingStatus


def _pose(serial: str, role: str, T: np.ndarray) -> TrackerPose:
    from vive_openvr.pose_utils import rotation_matrix_to_quaternion, transform_translation

    return TrackerPose(
        serial=serial,
        role=role,
        timestamp=1.0,
        transform=T,
        position=transform_translation(T),
        quaternion=rotation_matrix_to_quaternion(T[:3, :3]),
        pose_valid=True,
        device_connected=True,
        status=TrackingStatus.OK,
    )


def test_composition_world_body_hand() -> None:
    T_wb = se3_from_rt(np.eye(3), np.array([1.0, 0.0, 0.0]))
    T_bh = se3_from_rt(np.eye(3), np.array([0.0, 0.5, 0.0]))
    T_wh = T_wb @ T_bh
    rel = body_relative_transform(T_wb, T_wh)
    np.testing.assert_allclose(rel, T_bh, atol=1e-9)


def test_body_relative_invariant_to_common_translation() -> None:
    """Body and hand both +1 m in X → relative hand unchanged."""
    T_wb0 = identity_transform()
    T_wh0 = se3_from_rt(np.eye(3), np.array([0.2, 0.1, 0.0]))
    rel0 = body_relative_transform(T_wb0, T_wh0)

    shift = se3_from_rt(np.eye(3), np.array([1.0, 0.0, 0.0]))
    rel1 = body_relative_transform(shift @ T_wb0, shift @ T_wh0)
    np.testing.assert_allclose(rel0, rel1, atol=1e-9)


def test_body_relative_pose_class() -> None:
    T_wb = se3_from_rt(np.eye(3), np.array([0.0, 0.0, 1.0]))
    T_wh = se3_from_rt(np.eye(3), np.array([0.3, 0.0, 1.0]))
    br = BodyRelativePose()
    rel = br.compute(_pose("B", "body", T_wb), _pose("H", "right_wrist", T_wh))
    assert rel is not None
    assert rel.body_valid and rel.hand_valid
    np.testing.assert_allclose(rel.position, [0.3, 0.0, 0.0], atol=1e-9)


def test_inverse_roundtrip() -> None:
    T = se3_from_rt(np.eye(3), np.array([0.1, -0.2, 0.3]))
    np.testing.assert_allclose(T @ invert_se3(T), np.eye(4), atol=1e-9)
