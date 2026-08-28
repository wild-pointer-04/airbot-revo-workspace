"""Pose math: quaternion [x, y, z, w], never Euler subtraction."""

from __future__ import annotations

import numpy as np

_EPS = 1e-12


def as_vec3(value: np.ndarray) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(3)
    return arr


def as_quat(value: np.ndarray) -> np.ndarray:
    return quat_normalize(np.asarray(value, dtype=np.float64).reshape(4))


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < _EPS:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = as_quat(q)
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def quat_inverse(q: np.ndarray) -> np.ndarray:
    return quat_conjugate(as_quat(q))


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, both [x, y, z, w]."""
    ax, ay, az, aw = as_quat(a)
    bx, by, bz, bw = as_quat(b)
    return quat_normalize(
        np.array(
            [
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ],
            dtype=np.float64,
        )
    )


def quat_relative(q_current: np.ndarray, q_reference: np.ndarray) -> np.ndarray:
    """Rotation that takes reference onto current, expressed in the same frame."""
    return quat_multiply(as_quat(q_current), quat_inverse(q_reference))


def quat_apply(q_delta: np.ndarray, q_base: np.ndarray) -> np.ndarray:
    return quat_multiply(as_quat(q_delta), as_quat(q_base))


def quat_angle(q: np.ndarray) -> float:
    w = float(np.clip(abs(as_quat(q)[3]), 0.0, 1.0))
    return 2.0 * float(np.arccos(w))


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0 = as_quat(q0)
    q1 = as_quat(q1)
    t = float(np.clip(t, 0.0, 1.0))
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return quat_normalize(q0 + t * (q1 - q0))
    theta = float(np.arccos(np.clip(dot, -1.0, 1.0)))
    sin_theta = float(np.sin(theta))
    a = float(np.sin((1.0 - t) * theta) / sin_theta)
    b = float(np.sin(t * theta) / sin_theta)
    return quat_normalize(a * q0 + b * q1)


def quat_from_matrix(r: np.ndarray) -> np.ndarray:
    m = np.asarray(r, dtype=np.float64).reshape(3, 3)
    t = float(np.trace(m))
    if t > 0.0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return quat_normalize(np.array([x, y, z, w], dtype=np.float64))


def matrix_from_quat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = as_quat(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def identity_quat() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def identity_pose() -> tuple[np.ndarray, np.ndarray]:
    return (
        np.zeros(3, dtype=np.float64),
        identity_quat(),
    )


def rotate_vector(rotation_xyzw: np.ndarray, vector: np.ndarray) -> np.ndarray:
    return matrix_from_quat(as_quat(rotation_xyzw)) @ as_vec3(vector)
