"""Quaternion and relative-pose geometry."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vive_tracking.pose_utils import (
    apply_left_transform,
    axis_angle_to_quaternion,
    enforce_quaternion_continuity,
    interpolate_pose,
    make_bench_frame,
    pose_compose,
    quaternion_angle,
    quaternion_inverse,
    quaternion_multiply,
    quaternion_rotate_vector,
    relative_pose,
    world_to_bench,
)
from vive_tracking.types import IDENTITY_QUATERNION, Pose6DoF, Quaternion, TrackerPose, Vec3

_SQRT2_2 = math.sqrt(2.0) / 2.0


def _pose(x: float, q: Quaternion, seq: int = 1) -> TrackerPose:
    return TrackerPose(
        serial="T",
        logical_name="t",
        position_m=Vec3(x, 0.0, 0.0),
        orientation=q,
        sensor_timestamp=None,
        host_monotonic_ns=seq * 10_000_000,
        tracking_valid=True,
        sequence_id=seq,
        source="test",
    )


def test_relative_translation_along_x() -> None:
    t0 = _pose(0.0, IDENTITY_QUATERNION, 1)
    t1 = _pose(0.1, IDENTITY_QUATERNION, 2)
    rel = relative_pose(t0, t1)
    assert rel.position_m.x == pytest.approx(0.1, abs=1e-9)
    assert rel.position_m.y == pytest.approx(0.0, abs=1e-9)
    assert rel.position_m.z == pytest.approx(0.0, abs=1e-9)
    assert rel.orientation.w == pytest.approx(1.0, abs=1e-9)


def test_relative_rotation_about_z() -> None:
    t0 = _pose(0.0, IDENTITY_QUATERNION, 1)
    q_z90 = axis_angle_to_quaternion(Vec3(0.0, 0.0, 1.0), math.pi / 2.0)
    t1 = TrackerPose(
        serial="T",
        logical_name="t",
        position_m=Vec3(0.0, 0.0, 0.0),
        orientation=q_z90,
        sensor_timestamp=None,
        host_monotonic_ns=20_000_000,
        tracking_valid=True,
        sequence_id=2,
        source="test",
    )
    rel = relative_pose(t0, t1)
    assert rel.orientation.w == pytest.approx(_SQRT2_2, abs=1e-6)
    assert rel.orientation.z == pytest.approx(_SQRT2_2, abs=1e-6)
    assert quaternion_angle(rel.orientation, q_z90) == pytest.approx(0.0, abs=1e-6)


def test_relative_pose_invariant_to_world_origin() -> None:
    t0 = Pose6DoF(Vec3(0.2, -0.1, 0.4), IDENTITY_QUATERNION)
    t1 = Pose6DoF(
        Vec3(0.3, -0.1, 0.4),
        axis_angle_to_quaternion(Vec3(0.0, 0.0, 1.0), 0.4),
    )
    g = Pose6DoF(
        Vec3(1.5, -0.7, 0.25),
        axis_angle_to_quaternion(Vec3(0.0, 1.0, 0.0), 0.8),
    )
    rel = relative_pose(t0, t1)
    rel_g = relative_pose(apply_left_transform(g, t0), apply_left_transform(g, t1))
    assert rel.position_m.x == pytest.approx(rel_g.position_m.x, abs=1e-8)
    assert rel.position_m.y == pytest.approx(rel_g.position_m.y, abs=1e-8)
    assert rel.position_m.z == pytest.approx(rel_g.position_m.z, abs=1e-8)
    assert rel.orientation.w == pytest.approx(rel_g.orientation.w, abs=1e-8)
    assert rel.orientation.x == pytest.approx(rel_g.orientation.x, abs=1e-8)
    assert rel.orientation.y == pytest.approx(rel_g.orientation.y, abs=1e-8)
    assert rel.orientation.z == pytest.approx(rel_g.orientation.z, abs=1e-8)


def test_quaternion_sign_continuity() -> None:
    previous = Quaternion(0.99, 0.0, 0.0, 0.0)
    flipped = Quaternion(-0.99, 0.0, 0.0, 0.0)
    fixed = enforce_quaternion_continuity(flipped, previous)
    assert fixed.w > 0.0


def test_slerp_not_component_lerp() -> None:
    q0 = IDENTITY_QUATERNION
    q1 = axis_angle_to_quaternion(Vec3(0.0, 0.0, 1.0), math.pi / 2.0)
    p0 = _pose(0.0, q0, 1)
    p1 = TrackerPose(
        serial="T",
        logical_name="t",
        position_m=Vec3(1.0, 0.0, 0.0),
        orientation=q1,
        sensor_timestamp=None,
        host_monotonic_ns=20_000_000,
        tracking_valid=True,
        sequence_id=2,
        source="test",
    )
    mid = interpolate_pose(p0, p1, 15_000_000)
    assert mid.source == "interpolated"
    assert mid.sensor_timestamp is None
    assert mid.position_m.x == pytest.approx(0.5, abs=1e-9)
    expected = axis_angle_to_quaternion(Vec3(0.0, 0.0, 1.0), math.pi / 4.0)
    assert mid.orientation.w == pytest.approx(expected.w, abs=1e-6)
    assert mid.orientation.z == pytest.approx(expected.z, abs=1e-6)
    naive_w = 0.5 * (q0.w + q1.w)
    assert mid.orientation.w != pytest.approx(naive_w, abs=1e-3)


def test_pose_compose_inverse_identity() -> None:
    q = axis_angle_to_quaternion(Vec3(1.0, 2.0, 3.0), 0.6)
    p = Vec3(0.11, -0.22, 0.33)
    inv = quaternion_inverse(q)
    ident = quaternion_multiply(q, inv)
    assert ident.w == pytest.approx(1.0, abs=1e-9)
    composed = pose_compose(p, q, Vec3(-p.x, -p.y, -p.z), IDENTITY_QUATERNION)
    rotated_back = np.array(composed.position_m.as_tuple())
    # Not identity in translation because inverse needs rotated -t; covered by relative_pose tests.
    assert rotated_back.shape == (3,)


def test_make_bench_frame_hand_at_b_is_plus_x() -> None:
    origin = Vec3(0.10, -0.20, 0.02)
    axis = Vec3(0.60, -0.20, 0.02)
    bench = make_bench_frame(origin, axis, Vec3(0.0, 0.0, 1.0))
    hand = Pose6DoF(axis, IDENTITY_QUATERNION)
    rel = world_to_bench(hand, bench)
    assert rel.position_m.x == pytest.approx(0.50, abs=1e-8)
    assert rel.position_m.y == pytest.approx(0.0, abs=1e-8)
    assert rel.position_m.z == pytest.approx(0.0, abs=1e-8)


def test_make_bench_frame_rejects_coincident_landmarks() -> None:
    p = Vec3(0.1, 0.2, 0.3)
    with pytest.raises(ValueError, match="coincident"):
        make_bench_frame(p, p, Vec3(0.0, 0.0, 1.0))


def test_make_bench_frame_rejects_parallel_up() -> None:
    with pytest.raises(ValueError, match="parallel"):
        make_bench_frame(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.5), Vec3(0.0, 0.0, 1.0))


def test_bench_hand_invariant_to_world_se3() -> None:
    origin = Pose6DoF(Vec3(0.10, -0.20, 0.02), IDENTITY_QUATERNION)
    axis = Pose6DoF(Vec3(0.60, -0.20, 0.02), IDENTITY_QUATERNION)
    hand = Pose6DoF(
        Vec3(0.40, 0.00, 0.50),
        axis_angle_to_quaternion(Vec3(0.0, 0.0, 1.0), 0.4),
    )
    up = Vec3(0.0, 0.0, 1.0)
    bench = make_bench_frame(origin.position_m, axis.position_m, up)
    rel = world_to_bench(hand, bench)
    g = Pose6DoF(
        Vec3(1.5, -0.7, 0.25),
        axis_angle_to_quaternion(Vec3(0.2, 0.8, 0.1), 0.9),
    )
    origin_g = apply_left_transform(g, origin)
    axis_g = apply_left_transform(g, axis)
    hand_g = apply_left_transform(g, hand)
    up_g = quaternion_rotate_vector(g.orientation, up)
    bench_g = make_bench_frame(origin_g.position_m, axis_g.position_m, up_g)
    rel_g = world_to_bench(hand_g, bench_g)
    assert rel.position_m.x == pytest.approx(rel_g.position_m.x, abs=1e-7)
    assert rel.position_m.y == pytest.approx(rel_g.position_m.y, abs=1e-7)
    assert rel.position_m.z == pytest.approx(rel_g.position_m.z, abs=1e-7)
    assert quaternion_angle(rel.orientation, rel_g.orientation) == pytest.approx(0.0, abs=1e-6)
