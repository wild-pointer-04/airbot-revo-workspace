"""SE(3) / quaternion helpers for OpenVR poses.

Convention
----------
- T_A_B is a 4x4 matrix that maps a point expressed in frame B into frame A:
      p_A = T_A_B @ p_B_homogeneous
- Quaternion storage is [x, y, z, w].
- Length unit is metres.
"""

from __future__ import annotations

import numpy as np

_EPS = 1e-12


def identity_transform() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


def openvr_matrix34_to_se3(m34) -> np.ndarray:
    """Convert OpenVR HmdMatrix34_t (3x4 row-major) to a 4x4 SE(3) matrix.

    OpenVR stores rows [r00 r01 r02 tx; r10 ... ty; r20 ... tz].
    """
    T = np.eye(4, dtype=np.float64)
    for r in range(3):
        row = m34[r]
        T[r, 0] = float(row[0])
        T[r, 1] = float(row[1])
        T[r, 2] = float(row[2])
        T[r, 3] = float(row[3])
    return T


def transform_translation(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    return T[:3, 3].copy()


def transform_rotation(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    return T[:3, :3].copy()


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion [x,y,z,w]; returns identity if near-zero."""
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < _EPS:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n


def enforce_quaternion_continuity(q: np.ndarray, q_prev: np.ndarray | None) -> np.ndarray:
    """Flip q sign if dot(q, q_prev) < 0 so consecutive samples stay continuous.

    q and -q represent the same rotation; flipping avoids filter/log jumps.
    Does not change the underlying SO(3) of an SE(3) matrix.
    """
    q = normalize_quaternion(q)
    if q_prev is None:
        return q
    prev = normalize_quaternion(q_prev)
    if float(np.dot(q, prev)) < 0.0:
        return -q
    return q


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """SO(3) matrix → unit quaternion [x, y, z, w] (Shepperd)."""
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion(np.array([x, y, z, w], dtype=np.float64))


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Unit quaternion [x,y,z,w] → SO(3)."""
    x, y, z, w = normalize_quaternion(q)
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


def se3_from_rt(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Build T from rotation matrix and translation vector."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_se3(T: np.ndarray) -> np.ndarray:
    """Invert SE(3): R^T, -R^T t."""
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]
    Rt = R.T
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rt
    out[:3, 3] = -Rt @ t
    return out


def assert_se3(T: np.ndarray, *, atol: float = 1e-6) -> None:
    """Raise if T is not a valid SE(3) matrix (for tests / debug)."""
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"SE(3) must be 4x4, got {T.shape}")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=atol):
        raise ValueError(f"SE(3) bottom row invalid: {T[3]}")
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=atol):
        raise ValueError("SE(3) rotation is not orthonormal")
    if abs(float(np.linalg.det(R)) - 1.0) > atol * 10:
        raise ValueError(f"SE(3) det(R)={np.linalg.det(R)}")
