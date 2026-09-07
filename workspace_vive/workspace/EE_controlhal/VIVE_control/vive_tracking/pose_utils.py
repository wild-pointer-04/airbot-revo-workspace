"""SE(3) helpers. Internal orientation is always quaternion [w, x, y, z]."""

from __future__ import annotations

from dataclasses import replace
from typing import Optional, Union

import numpy as np

from vive_tracking.types import (
    IDENTITY_QUATERNION,
    Pose6DoF,
    Quaternion,
    TrackerPose,
    Vec3,
)

_NORM_EPS = 1e-12
_SLERP_LINEAR_DOT = 0.9995
PoseLike = Union[TrackerPose, Pose6DoF]


def quaternion_to_array(q: Quaternion) -> np.ndarray:
    return np.array(q.as_wxyz(), dtype=np.float64)


def array_to_quaternion(values: np.ndarray) -> Quaternion:
    return Quaternion(
        w=float(values[0]),
        x=float(values[1]),
        y=float(values[2]),
        z=float(values[3]),
    )


def normalize_quaternion(q: Quaternion) -> Quaternion:
    values = quaternion_to_array(q)
    norm = float(np.linalg.norm(values))
    if norm < _NORM_EPS:
        return IDENTITY_QUATERNION
    return array_to_quaternion(values / norm)


def _hamilton(a: Quaternion, b: Quaternion) -> Quaternion:
    """Raw Hamilton product. Does not normalize; needed for vector rotation."""
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """Hamilton product a ⊗ b on unit orientations. Applies b first, then a."""
    return _hamilton(normalize_quaternion(a), normalize_quaternion(b))


def quaternion_inverse(q: Quaternion) -> Quaternion:
    q = normalize_quaternion(q)
    return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)


def quaternion_dot(a: Quaternion, b: Quaternion) -> float:
    return float(np.dot(quaternion_to_array(a), quaternion_to_array(b)))


def quaternion_angle(a: Quaternion, b: Quaternion) -> float:
    """Geodesic angle in radians between two orientations."""
    a = normalize_quaternion(a)
    b = normalize_quaternion(b)
    dot = abs(quaternion_dot(a, b))
    dot = min(1.0, max(0.0, dot))
    return 2.0 * float(np.arccos(dot))


def enforce_quaternion_continuity(current: Quaternion, previous: Optional[Quaternion]) -> Quaternion:
    """Flip sign so q and -q do not appear as a jump. Same rotation."""
    current = normalize_quaternion(current)
    if previous is None:
        return current
    if quaternion_dot(current, previous) < 0.0:
        return Quaternion(w=-current.w, x=-current.x, y=-current.y, z=-current.z)
    return current


def axis_angle_to_quaternion(axis: Vec3, angle_rad: float) -> Quaternion:
    axis_np = np.array(axis.as_tuple(), dtype=np.float64)
    norm = float(np.linalg.norm(axis_np))
    if norm < _NORM_EPS:
        return IDENTITY_QUATERNION
    axis_np = axis_np / norm
    half = 0.5 * angle_rad
    sin_half = float(np.sin(half))
    return normalize_quaternion(
        Quaternion(
            w=float(np.cos(half)),
            x=axis_np[0] * sin_half,
            y=axis_np[1] * sin_half,
            z=axis_np[2] * sin_half,
        )
    )


def quaternion_rotate_vector(q: Quaternion, v: Vec3) -> Vec3:
    q = normalize_quaternion(q)
    qv = Quaternion(0.0, v.x, v.y, v.z)
    rotated = _hamilton(_hamilton(q, qv), Quaternion(q.w, -q.x, -q.y, -q.z))
    return Vec3(rotated.x, rotated.y, rotated.z)


def pose_inverse(position: Vec3, orientation: Quaternion) -> Pose6DoF:
    q_inv = quaternion_inverse(orientation)
    t_inv = quaternion_rotate_vector(q_inv, Vec3(-position.x, -position.y, -position.z))
    return Pose6DoF(position_m=t_inv, orientation=q_inv)


def pose_compose(
    a_pos: Vec3,
    a_ori: Quaternion,
    b_pos: Vec3,
    b_ori: Quaternion,
) -> Pose6DoF:
    """T_a * T_b."""
    rotated_b = quaternion_rotate_vector(a_ori, b_pos)
    return Pose6DoF(
        position_m=Vec3(
            a_pos.x + rotated_b.x,
            a_pos.y + rotated_b.y,
            a_pos.z + rotated_b.z,
        ),
        orientation=quaternion_multiply(a_ori, b_ori),
    )


def _split_pose(pose: PoseLike) -> tuple[Vec3, Quaternion]:
    return pose.position_m, pose.orientation


def relative_pose(current: PoseLike, future: PoseLike) -> Pose6DoF:
    """T_rel = T_current^{-1} * T_future (UMI-style hand-relative motion)."""
    current_inv = pose_inverse(*_split_pose(current))
    return pose_compose(
        current_inv.position_m,
        current_inv.orientation,
        future.position_m,
        future.orientation,
    )


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> Quaternion:
    """Convert a 3×3 rotation matrix (columns = axes in the parent frame) to [w,x,y,z]."""
    m = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        scale = float(np.sqrt(trace + 1.0)) * 2.0
        w = 0.25 * scale
        x = (m[2, 1] - m[1, 2]) / scale
        y = (m[0, 2] - m[2, 0]) / scale
        z = (m[1, 0] - m[0, 1]) / scale
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        scale = float(np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
        w = (m[2, 1] - m[1, 2]) / scale
        x = 0.25 * scale
        y = (m[0, 1] + m[1, 0]) / scale
        z = (m[0, 2] + m[2, 0]) / scale
    elif m[1, 1] > m[2, 2]:
        scale = float(np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
        w = (m[0, 2] - m[2, 0]) / scale
        x = (m[0, 1] + m[1, 0]) / scale
        y = 0.25 * scale
        z = (m[1, 2] + m[2, 1]) / scale
    else:
        scale = float(np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
        w = (m[1, 0] - m[0, 1]) / scale
        x = (m[0, 2] + m[2, 0]) / scale
        y = (m[1, 2] + m[2, 1]) / scale
        z = 0.25 * scale
    return normalize_quaternion(Quaternion(w, x, y, z))


def make_bench_frame(p_a: Vec3, p_b: Vec3, up_hint: Vec3) -> Pose6DoF:
    """Workbench frame from two landmark positions.

    Origin at A, X along A→B, Z from ``up_hint`` projected off X, Y = Z × X.
    Uses positions only (landmark tracker orientation is ignored).
    """
    origin = np.array(p_a.as_tuple(), dtype=np.float64)
    x_axis = np.array(p_b.as_tuple(), dtype=np.float64) - origin
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm < _NORM_EPS:
        raise ValueError("bench_origin and bench_x are coincident")
    x_axis = x_axis / x_norm
    up = np.array(up_hint.as_tuple(), dtype=np.float64)
    up_norm = float(np.linalg.norm(up))
    if up_norm < _NORM_EPS:
        raise ValueError("up_hint is a zero vector")
    up = up / up_norm
    z_axis = up - x_axis * float(np.dot(up, x_axis))
    z_norm = float(np.linalg.norm(z_axis))
    if z_norm < _NORM_EPS:
        raise ValueError("up_hint is parallel to bench X")
    z_axis = z_axis / z_norm
    y_axis = np.cross(z_axis, x_axis)
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm < _NORM_EPS:
        raise ValueError("failed to complete bench frame")
    y_axis = y_axis / y_norm
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / float(np.linalg.norm(z_axis))
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    return Pose6DoF(position_m=p_a, orientation=rotation_matrix_to_quaternion(rotation))


def world_to_bench(hand: PoseLike, bench: Pose6DoF) -> Pose6DoF:
    """T_bench_hand = T_world_bench^{-1} * T_world_hand."""
    return relative_pose(bench, hand)


def apply_left_transform(transform: Pose6DoF, pose: PoseLike) -> Pose6DoF:
    """G * T. Used to test invariance to VIVE world origin."""
    return pose_compose(
        transform.position_m,
        transform.orientation,
        pose.position_m,
        pose.orientation,
    )


def quaternion_slerp(q0: Quaternion, q1: Quaternion, alpha: float) -> Quaternion:
    a = quaternion_to_array(normalize_quaternion(q0))
    b = quaternion_to_array(normalize_quaternion(q1))
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > _SLERP_LINEAR_DOT:
        out = a + alpha * (b - a)
        return normalize_quaternion(array_to_quaternion(out))
    theta = float(np.arccos(dot))
    sin_theta = float(np.sin(theta))
    w0 = float(np.sin((1.0 - alpha) * theta) / sin_theta)
    w1 = float(np.sin(alpha * theta) / sin_theta)
    return array_to_quaternion(w0 * a + w1 * b)


def interpolate_pose(
    pose_before: TrackerPose,
    pose_after: TrackerPose,
    target_time_ns: int,
) -> TrackerPose:
    """Linear translation + quaternion SLERP.

    This is a utility for a future Synchronizer. The tracker pipeline does
    not auto-interpolate live samples. Interpolated poses are marked
    source='interpolated' and sensor_timestamp is None (never forged).
    """
    t0 = pose_before.host_monotonic_ns
    t1 = pose_after.host_monotonic_ns
    if t1 == t0:
        alpha = 0.0
    else:
        alpha = (target_time_ns - t0) / (t1 - t0)
    alpha = float(np.clip(alpha, 0.0, 1.0))
    pos0 = np.array(pose_before.position_m.as_tuple(), dtype=np.float64)
    pos1 = np.array(pose_after.position_m.as_tuple(), dtype=np.float64)
    pos = pos0 + alpha * (pos1 - pos0)
    orientation = quaternion_slerp(pose_before.orientation, pose_after.orientation, alpha)
    return TrackerPose(
        serial=pose_after.serial,
        logical_name=pose_after.logical_name,
        position_m=Vec3(float(pos[0]), float(pos[1]), float(pos[2])),
        orientation=orientation,
        sensor_timestamp=None,
        host_monotonic_ns=int(target_time_ns),
        tracking_valid=pose_before.tracking_valid and pose_after.tracking_valid,
        sequence_id=pose_after.sequence_id,
        source="interpolated",
    )


def apply_extrinsics(pose: TrackerPose, t_tracker_hand: Optional[Pose6DoF]) -> TrackerPose:
    """Reserved Tracker→Hand extrinsics hook. Does not guess a calibration."""
    if t_tracker_hand is None:
        return pose
    composed = pose_compose(
        pose.position_m,
        pose.orientation,
        t_tracker_hand.position_m,
        t_tracker_hand.orientation,
    )
    return replace(
        pose,
        position_m=composed.position_m,
        orientation=composed.orientation,
        source=f"{pose.source}+extrinsics",
    )
