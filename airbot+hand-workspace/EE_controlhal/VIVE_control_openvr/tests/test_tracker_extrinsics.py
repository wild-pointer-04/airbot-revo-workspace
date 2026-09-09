"""Unit tests for tracker extrinsics (no SteamVR)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_openvr.pose_utils import identity_transform, se3_from_rt
from vive_openvr.tracker_extrinsics import (
    apply_extrinsics_transform,
    apply_pose_extrinsics,
    load_role_extrinsics,
    se3_from_translation_quat,
)
from vive_openvr.types import TrackerPose, TrackingStatus


def test_identity_extrinsics_noop() -> None:
    T_w = se3_from_rt(np.eye(3), np.array([1.0, 2.0, 3.0]))
    out = apply_extrinsics_transform(T_w, identity_transform())
    np.testing.assert_allclose(out, T_w, atol=1e-9)


def test_translation_extrinsic() -> None:
    T_w = se3_from_rt(np.eye(3), np.array([1.0, 0.0, 0.0]))
    T_ts = se3_from_rt(np.eye(3), np.array([0.0, 0.1, 0.0]))
    out = apply_extrinsics_transform(T_w, T_ts)
    np.testing.assert_allclose(out[:3, 3], [1.0, 0.1, 0.0], atol=1e-9)


def test_load_config_identity() -> None:
    cfg = {
        "extrinsics": {
            "body": {
                "translation": [0.0, 0.0, 0.0],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
        }
    }
    loaded = load_role_extrinsics(cfg)
    np.testing.assert_allclose(loaded["body"], identity_transform(), atol=1e-9)


def test_apply_pose_extrinsics_updates_fields() -> None:
    T_w = se3_from_rt(np.eye(3), np.array([0.0, 0.0, 0.0]))
    T_ts = se3_from_translation_quat([0.05, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    pose = TrackerPose(
        serial="LHR-TEST",
        role="right_wrist",
        timestamp=0.0,
        transform=T_w,
        position=np.zeros(3),
        quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
        pose_valid=True,
        device_connected=True,
        status=TrackingStatus.OK,
    )
    out = apply_pose_extrinsics(pose, T_ts)
    np.testing.assert_allclose(out.position, [0.05, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(out.transform[:3, 3], [0.05, 0.0, 0.0], atol=1e-9)
