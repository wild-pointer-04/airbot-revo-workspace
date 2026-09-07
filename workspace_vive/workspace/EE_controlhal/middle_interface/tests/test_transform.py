"""Coordinate transform and quaternion relative rotation tests."""

from __future__ import annotations

import numpy as np

from processing.coordinate_transform import CoordinateTransform
from processing.math_pose import (
    matrix_from_quat,
    quat_angle,
    quat_apply,
    quat_from_matrix,
    quat_multiply,
    quat_normalize,
    quat_relative,
    quat_slerp,
)
from protocol.pico_packet import Pose


def test_identity_transform():
    tf = CoordinateTransform.from_config(
        {"axis_map": {"x": ["x", 1.0], "y": ["y", 1.0], "z": ["z", 1.0]}}
    )
    pos = np.array([0.1, 0.2, 0.3])
    rot = np.array([0.0, 0.0, 0.0, 1.0])
    pose = tf.transform_pose(Pose(pos, rot))
    np.testing.assert_allclose(pose.position, pos)
    np.testing.assert_allclose(pose.rotation, rot, atol=1e-8)


def test_axis_remap_unity_y_up_to_z_up():
    tf = CoordinateTransform.from_config(
        {"axis_map": {"x": ["x", 1.0], "y": ["z", -1.0], "z": ["y", 1.0]}}
    )
    pico = np.array([0.1, 0.2, 0.3])
    robot = tf.transform_position(pico)
    np.testing.assert_allclose(robot, [0.1, -0.3, 0.2])


def test_quaternion_normalization():
    q = quat_normalize(np.array([0.0, 0.0, 0.0, 2.0]))
    np.testing.assert_allclose(q, [0.0, 0.0, 0.0, 1.0])
    q0 = quat_normalize(np.zeros(4))
    np.testing.assert_allclose(q0, [0.0, 0.0, 0.0, 1.0])


def test_relative_rotation_90deg_z():
    q_ref = np.array([0.0, 0.0, 0.0, 1.0])
    q_cur = quat_from_matrix(
        np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    )
    q_delta = quat_relative(q_cur, q_ref)
    np.testing.assert_allclose(quat_angle(q_delta), np.pi / 2.0, atol=1e-6)
    q_target = quat_apply(q_delta, q_ref)
    np.testing.assert_allclose(np.abs(np.dot(q_target, q_cur)), 1.0, atol=1e-6)


def test_slerp_stays_normalized():
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = quat_from_matrix(matrix_from_quat(np.array([0.0, 0.70710678, 0.0, 0.70710678])))
    q = quat_slerp(q0, q1, 0.5)
    np.testing.assert_allclose(np.linalg.norm(q), 1.0, atol=1e-8)


def test_quat_multiply_identity():
    q = quat_normalize(np.array([0.1, 0.2, 0.3, 0.9]))
    ident = np.array([0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(quat_multiply(q, ident), q, atol=1e-8)


def test_openxr_to_teleop_standard():
    from processing.math_pose import matrix_from_quat
    from processing.vr_normalize import (
        OPENXR_TO_TELEOP_R,
        openxr_to_teleop_position,
        openxr_to_teleop_rotation,
    )

    np.testing.assert_allclose(
        openxr_to_teleop_position([0.1, 0.2, 0.3]), [-0.3, -0.1, 0.2]
    )
    # Position and rotation must describe the same change of basis:
    # T(R @ v) == T_rot(R) @ T(v) for every rotation R and vector v.
    _assert_pose_consistency(openxr_to_teleop_position, openxr_to_teleop_rotation)


def test_unity_to_teleop_standard():
    from processing.vr_normalize import unity_to_teleop_position, unity_to_teleop_rotation

    np.testing.assert_allclose(
        unity_to_teleop_position([0.1, 0.2, 0.3]), [0.3, -0.1, 0.2]
    )
    _assert_pose_consistency(unity_to_teleop_position, unity_to_teleop_rotation)


def _assert_pose_consistency(to_position, to_rotation):
    """A change of basis must map positions and rotations consistently.

    Unity is left-handed and teleop_standard is right-handed, so the position
    matrix has det -1. Conjugating by it still yields a proper rotation, but
    only without an extra transpose -- that transpose silently reverses the
    rotation and makes the arm wrench J4/J5 to reach a distorted target.
    """
    from processing.math_pose import matrix_from_quat

    rng = np.random.default_rng(0)
    for _ in range(64):
        raw = rng.normal(size=4)
        quat = raw / np.linalg.norm(raw)
        vector = rng.normal(size=3)
        rotated = matrix_from_quat(to_rotation(quat))
        # Output stays a proper rotation.
        np.testing.assert_allclose(np.linalg.det(rotated), 1.0, atol=1e-9)
        np.testing.assert_allclose(rotated @ rotated.T, np.eye(3), atol=1e-9)
        # Rotating then converting equals converting then rotating.
        np.testing.assert_allclose(
            to_position(matrix_from_quat(quat) @ vector),
            rotated @ to_position(vector),
            atol=1e-9,
        )


def test_head_base_yaw_invariance():
    from processing.head_base import HeadBase
    from processing.math_pose import matrix_from_quat, quat_from_matrix

    hb = HeadBase()
    ident = np.array([0.0, 0.0, 0.0, 1.0])
    hmd0 = Pose(np.array([0.0, 0.0, 1.6]), ident)
    hand0 = Pose(np.array([0.30, 0.0, 1.2]), ident)
    rel0 = hb.relative(hmd0, True, hand0)

    yaw = quat_from_matrix(
        np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    )
    origin = hmd0.position
    hand1_pos = origin + matrix_from_quat(yaw) @ (hand0.position - origin)
    hmd1 = Pose(hmd0.position.copy(), yaw)
    hand1 = Pose(hand1_pos, yaw)
    rel1 = hb.relative(hmd1, True, hand1)
    np.testing.assert_allclose(rel0.position, rel1.position, atol=1e-8)
    np.testing.assert_allclose(
        np.abs(np.dot(rel0.rotation, rel1.rotation)), 1.0, atol=1e-8
    )
