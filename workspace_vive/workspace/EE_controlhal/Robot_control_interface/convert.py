"""Quaternion [x, y, z, w] ↔ xyzrpy used by ARX X5 (same formulas as SingleArm)."""

from __future__ import annotations

import numpy as np

_EPS = 1e-12


def quat_xyzw_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < _EPS:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n


def quat_xyzw_to_rpy(rotation_xyzw: np.ndarray) -> np.ndarray:
    qx, qy, qz, qw = quat_xyzw_normalize(rotation_xyzw)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = float(np.arctan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = float(np.copysign(np.pi / 2.0, sinp))
    else:
        pitch = float(np.arcsin(sinp))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = float(np.arctan2(siny_cosp, cosy_cosp))
    return np.array([roll, pitch, yaw], dtype=np.float64)


def rpy_to_quat_xyzw(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return quat_xyzw_normalize(np.array([x, y, z, w], dtype=np.float64))


def pose_to_xyzrpy(position: np.ndarray, rotation_xyzw: np.ndarray) -> np.ndarray:
    pos = np.asarray(position, dtype=np.float64).reshape(3)
    rpy = quat_xyzw_to_rpy(rotation_xyzw)
    return np.concatenate([pos, rpy])


def xyzrpy_to_pose(xyzrpy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    arr = np.asarray(xyzrpy, dtype=np.float64).reshape(6)
    return arr[:3].copy(), rpy_to_quat_xyzw(float(arr[3]), float(arr[4]), float(arr[5]))


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0 = quat_xyzw_normalize(q0)
    q1 = quat_xyzw_normalize(q1)
    t = float(np.clip(t, 0.0, 1.0))
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return quat_xyzw_normalize(q0 + t * (q1 - q0))
    theta = float(np.arccos(np.clip(dot, -1.0, 1.0)))
    sin_theta = float(np.sin(theta))
    a = float(np.sin((1.0 - t) * theta) / sin_theta)
    b = float(np.sin(t * theta) / sin_theta)
    return quat_xyzw_normalize(a * q0 + b * q1)


def quat_angle(q: np.ndarray) -> float:
    w = float(np.clip(abs(quat_xyzw_normalize(q)[3]), 0.0, 1.0))
    return 2.0 * float(np.arccos(w))


def slew_pose(
    pos: np.ndarray,
    rot: np.ndarray,
    target_pos: np.ndarray,
    target_rot: np.ndarray,
    dt: float,
    max_linear: float,
    max_angular: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Move at most max_linear m/s and max_angular rad/s toward the target."""
    dt = max(float(dt), 1e-6)
    pos = np.asarray(pos, dtype=np.float64).reshape(3)
    target_pos = np.asarray(target_pos, dtype=np.float64).reshape(3)
    delta = target_pos - pos
    dist = float(np.linalg.norm(delta))
    max_dist = float(max_linear) * dt
    if dist > max_dist and dist > 1e-12:
        pos = pos + delta * (max_dist / dist)
    else:
        pos = target_pos.copy()

    rot = quat_xyzw_normalize(rot)
    target_rot = quat_xyzw_normalize(target_rot)
    if float(np.dot(rot, target_rot)) < 0.0:
        target_rot = -target_rot
    # q_rel = target * conj(current), both xyzw
    rx, ry, rz, rw = rot
    tx, ty, tz, tw = target_rot
    rel = quat_xyzw_normalize(
        np.array(
            [
                tw * (-rx) + tx * rw + ty * rz - tz * ry,
                tw * (-ry) - tx * rz + ty * rw + tz * rx,
                tw * (-rz) + tx * ry - ty * rx + tz * rw,
                tw * rw - tx * rx - ty * ry - tz * rz,
            ],
            dtype=np.float64,
        )
    )
    angle = quat_angle(rel)
    max_ang = float(max_angular) * dt
    if angle > max_ang and angle > 1e-12:
        rot = quat_slerp(rot, target_rot, max_ang / angle)
    else:
        rot = target_rot
    return pos, rot
