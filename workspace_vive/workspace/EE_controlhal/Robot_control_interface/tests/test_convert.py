"""xyzrpy ↔ xyzw round-trip matches the ARX SingleArm convention."""

from __future__ import annotations

import numpy as np

from convert import pose_to_xyzrpy, quat_xyzw_to_rpy, rpy_to_quat_xyzw, xyzrpy_to_pose


def test_identity_roundtrip():
    q = np.array([0.0, 0.0, 0.0, 1.0])
    rpy = quat_xyzw_to_rpy(q)
    np.testing.assert_allclose(rpy, [0.0, 0.0, 0.0], atol=1e-8)
    back = rpy_to_quat_xyzw(*rpy)
    np.testing.assert_allclose(np.abs(np.dot(back, q)), 1.0, atol=1e-8)


def test_yaw_90_roundtrip():
    q = rpy_to_quat_xyzw(0.0, 0.0, np.pi / 2.0)
    rpy = quat_xyzw_to_rpy(q)
    np.testing.assert_allclose(rpy[2], np.pi / 2.0, atol=1e-6)
    again = rpy_to_quat_xyzw(*rpy)
    np.testing.assert_allclose(np.abs(np.dot(again, q)), 1.0, atol=1e-8)


def test_slew_caps_step():
    from convert import slew_pose

    pos, rot = slew_pose(
        np.zeros(3),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        dt=0.01,
        max_linear=0.50,
        max_angular=1.50,
    )
    np.testing.assert_allclose(pos, [0.005, 0.0, 0.0], atol=1e-8)


def test_pose_xyzrpy():
    pos = np.array([0.25, -0.05, 0.15])
    rot = rpy_to_quat_xyzw(0.1, -0.2, 0.3)
    xyzrpy = pose_to_xyzrpy(pos, rot)
    pos2, rot2 = xyzrpy_to_pose(xyzrpy)
    np.testing.assert_allclose(pos2, pos, atol=1e-8)
    np.testing.assert_allclose(np.abs(np.dot(rot2, rot)), 1.0, atol=1e-8)
