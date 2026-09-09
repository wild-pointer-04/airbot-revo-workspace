"""Unit tests for SE(3) / quaternion helpers (no SteamVR required)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_openvr.pose_utils import (
    assert_se3,
    enforce_quaternion_continuity,
    identity_transform,
    invert_se3,
    normalize_quaternion,
    openvr_matrix34_to_se3,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    se3_from_rt,
)


class _FakeRow(list):
    pass


class _FakeMatrix34(list):
    """Minimal stand-in for openvr.HmdMatrix34_t indexing."""


def test_identity_roundtrip() -> None:
    T = identity_transform()
    assert_se3(T)
    np.testing.assert_allclose(T @ invert_se3(T), np.eye(4), atol=1e-9)


def test_invert_composition() -> None:
    R = quaternion_to_rotation_matrix([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)])
    t = np.array([0.3, -0.1, 1.2])
    T = se3_from_rt(R, t)
    I = T @ invert_se3(T)
    np.testing.assert_allclose(I, np.eye(4), atol=1e-9)


def test_world_body_hand_composition() -> None:
    """world→hand == (world→body) @ (body→hand)."""
    T_w_b = se3_from_rt(np.eye(3), [1.0, 0.0, 0.0])
    T_b_h = se3_from_rt(np.eye(3), [0.0, 0.2, 0.0])
    T_w_h = T_w_b @ T_b_h
    np.testing.assert_allclose(T_w_h[:3, 3], [1.0, 0.2, 0.0], atol=1e-9)
    # relative recover
    T_b_h_rec = invert_se3(T_w_b) @ T_w_h
    np.testing.assert_allclose(T_b_h_rec, T_b_h, atol=1e-9)


def test_body_relative_invariant_under_common_translation() -> None:
    """Body and hand both +1 m in X → relative hand unchanged."""
    T_w_b0 = se3_from_rt(np.eye(3), [0.0, 0.0, 0.0])
    T_w_h0 = se3_from_rt(np.eye(3), [0.1, 0.2, 0.3])
    rel0 = invert_se3(T_w_b0) @ T_w_h0

    shift = se3_from_rt(np.eye(3), [1.0, 0.0, 0.0])
    T_w_b1 = shift @ T_w_b0
    T_w_h1 = shift @ T_w_h0
    rel1 = invert_se3(T_w_b1) @ T_w_h1
    np.testing.assert_allclose(rel0, rel1, atol=1e-9)


def test_quaternion_continuity_flips_sign() -> None:
    q = normalize_quaternion([0.1, 0.2, 0.3, 0.9])
    flipped = enforce_quaternion_continuity(-q, q)
    np.testing.assert_allclose(flipped, q, atol=1e-9)


def test_rotation_matrix_quaternion_roundtrip() -> None:
    angles = [0.0, 0.3, -0.7, 1.2]
    for a in angles:
        c, s = np.cos(a / 2), np.sin(a / 2)
        q = normalize_quaternion([0.0, 0.0, s, c])
        R = quaternion_to_rotation_matrix(q)
        q2 = rotation_matrix_to_quaternion(R)
        # same rotation up to sign
        if float(np.dot(q, q2)) < 0.0:
            q2 = -q2
        np.testing.assert_allclose(q2, q, atol=1e-8)


def test_openvr_matrix34_to_se3() -> None:
    m34 = [
        [1.0, 0.0, 0.0, 0.5],
        [0.0, 1.0, 0.0, -0.2],
        [0.0, 0.0, 1.0, 1.1],
    ]
    T = openvr_matrix34_to_se3(m34)
    assert_se3(T)
    np.testing.assert_allclose(T[:3, 3], [0.5, -0.2, 1.1])
