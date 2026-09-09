"""Two-tracker SteamVR / libsurvive → teleop_standard mapping.

SteamVR (OpenXR Stage): +X right, +Y up, -Z forward.
Native libsurvive: +Z up, with body heading supplied by HeadBase.
teleop_standard:                      +X forward, +Y left, +Z up.

Body tracker = HMD. Handheld tracker = right controller.
VIVE library stores quaternions [w, x, y, z]; packets use [x, y, z, w].
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(MIDDLE))

from keyboard import KeyboardButtons
from packet_source import VivePacketSource, _wxyz_to_xyzw
from processing.head_base import HeadBase
from processing.math_pose import matrix_from_quat
from processing.vr_normalize import openxr_to_teleop_position, to_teleop_pose
from protocol.pico_packet import Pose
from vive_tracking.types import IDENTITY_QUATERNION, Quaternion, TrackerPose, Vec3


def _tracker(
    name: str,
    xyz: tuple[float, float, float],
    wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    *,
    serial: str = "MOCK",
) -> TrackerPose:
    return TrackerPose(
        serial=serial,
        logical_name=name,
        position_m=Vec3(*xyz),
        orientation=Quaternion(*wxyz),
        sensor_timestamp=None,
        host_monotonic_ns=1,
        tracking_valid=True,
        sequence_id=1,
        source="mock",
    )


def _packet(body: TrackerPose, hand: TrackerPose):
    source = VivePacketSource(
        manager=None,  # type: ignore[arg-type]
        head_name="body_hmd",
        hand_name="hand_controller",
        buttons=KeyboardButtons(),
    )
    return source._build(body, hand, KeyboardButtons().snapshot())


def _hand_rel(packet) -> Pose:
    hand = to_teleop_pose(packet.right_controller.pose, "steamvr")
    hmd = to_teleop_pose(packet.hmd.pose, "steamvr")
    return HeadBase().relative(hmd, True, hand)


def test_vive_aliases_use_openxr_stage_not_unity() -> None:
    """SteamVR aliases use OpenXR; libsurvive preserves its Z-up world."""
    raw = Pose(np.array([0.10, 0.20, 0.30]), np.array([0.0, 0.0, 0.0, 1.0]))
    steamvr = to_teleop_pose(raw, "steamvr")
    vive = to_teleop_pose(raw, "vive")
    libsurvive = to_teleop_pose(raw, "libsurvive")
    openxr = to_teleop_pose(raw, "openxr")
    unity = to_teleop_pose(raw, "unity")

    np.testing.assert_allclose(steamvr.position, openxr.position)
    np.testing.assert_allclose(vive.position, openxr.position)
    np.testing.assert_allclose(libsurvive.position, raw.position)
    # OpenXR (x,y,z) -> (-z,-x,y); Unity -> (z,-x,y).
    np.testing.assert_allclose(steamvr.position, [-0.30, -0.10, 0.20])
    np.testing.assert_allclose(unity.position, [0.30, -0.10, 0.20])
    assert not np.allclose(steamvr.position, unity.position)


def test_libsurvive_is_z_up_identity() -> None:
    raw = Pose(np.array([0.10, 0.20, 0.30]), np.array([0.0, 0.0, 0.0, 1.0]))
    result = to_teleop_pose(raw, "libsurvive")
    np.testing.assert_allclose(result.position, [0.10, 0.20, 0.30])
    np.testing.assert_allclose(result.rotation, raw.rotation)


def test_steamvr_basis_to_teleop() -> None:
    """A point 1 m along each SteamVR axis lands on the matching teleop axis."""
    cases = {
        "steamvr +X right": ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),  # teleop -Y = right
        "steamvr +Y up": ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),  # teleop +Z
        "steamvr -Z forward": ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),  # teleop +X
        "steamvr +Z back": ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
    }
    for label, (steamvr, teleop) in cases.items():
        got = openxr_to_teleop_position(steamvr)
        np.testing.assert_allclose(got, teleop, err_msg=label)


def test_wxyz_to_xyzw_identity_and_yaw() -> None:
    np.testing.assert_allclose(_wxyz_to_xyzw(IDENTITY_QUATERNION), [0.0, 0.0, 0.0, 1.0])
    # 90° about SteamVR +Y → wxyz (c, 0, s, 0), packet xyzw (0, s, 0, c)
    half = np.sqrt(2.0) / 2.0
    q = Quaternion(half, 0.0, half, 0.0)
    np.testing.assert_allclose(_wxyz_to_xyzw(q), [0.0, half, 0.0, half])


def test_hand_right_of_body_is_teleop_minus_y() -> None:
    body = _tracker("body_hmd", (0.0, 1.60, 0.0))
    hand = _tracker("hand_controller", (0.30, 1.60, 0.0))  # SteamVR +X = right
    rel = _hand_rel(_packet(body, hand))
    np.testing.assert_allclose(rel.position, [0.0, -0.30, 0.0], atol=1e-9)


def test_hand_forward_of_body_is_teleop_plus_x() -> None:
    body = _tracker("body_hmd", (0.0, 1.60, 0.0))
    hand = _tracker("hand_controller", (0.0, 1.60, -0.40))  # SteamVR -Z = forward
    rel = _hand_rel(_packet(body, hand))
    np.testing.assert_allclose(rel.position, [0.40, 0.0, 0.0], atol=1e-9)


def test_hand_above_body_is_teleop_plus_z() -> None:
    body = _tracker("body_hmd", (0.0, 1.60, 0.0))
    hand = _tracker("hand_controller", (0.0, 1.80, 0.0))  # SteamVR +Y = up
    rel = _hand_rel(_packet(body, hand))
    np.testing.assert_allclose(rel.position, [0.0, 0.0, 0.20], atol=1e-9)


def test_body_yaw_does_not_change_hand_relative() -> None:
    """Operator turns 90° left; both trackers rotate together. Hand-rel stays."""
    body0 = _tracker("body_hmd", (0.0, 1.60, 0.0))
    hand0 = _tracker("hand_controller", (0.30, 1.60, 0.0))
    rel0 = _hand_rel(_packet(body0, hand0))

    # SteamVR yaw +90° about +Y (right-hand): facing -X, right-of-body becomes -Z.
    half = np.sqrt(2.0) / 2.0
    yaw_wxyz = (half, 0.0, half, 0.0)
    yaw_xyzw = _wxyz_to_xyzw(Quaternion(*yaw_wxyz))
    rot = matrix_from_quat(yaw_xyzw)
    offset = rot @ np.array([0.30, 0.0, 0.0])
    body1 = _tracker("body_hmd", (0.0, 1.60, 0.0), yaw_wxyz)
    hand1 = _tracker(
        "hand_controller",
        (0.0 + offset[0], 1.60 + offset[1], 0.0 + offset[2]),
        yaw_wxyz,
    )
    rel1 = _hand_rel(_packet(body1, hand1))
    np.testing.assert_allclose(rel0.position, rel1.position, atol=1e-8)
    np.testing.assert_allclose(np.abs(np.dot(rel0.rotation, rel1.rotation)), 1.0, atol=1e-8)


def test_body_tracker_mount_rotation_is_applied_before_yaw_projection() -> None:
    """A tracker mounted with local +Z forward still gives a body +X heading."""
    from processing.math_pose import quat_from_matrix

    # b = E t: body +X is tracker +Z, body +Y is tracker +Y, body +Z is tracker -X.
    tracker_to_body = np.array(
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]
    )
    body = Pose(np.zeros(3), quat_from_matrix(tracker_to_body))
    hand = Pose(np.array([0.40, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
    relative = HeadBase(tracker_to_body).relative(body, True, hand)
    np.testing.assert_allclose(relative.position, [0.40, 0.0, 0.0], atol=1e-9)


def test_body_tracker_mount_accepts_full_matrix_from_config() -> None:
    """The measured, non-permutation mount format has the same E convention."""
    tracker_to_body = np.array(
        [
            [0.93969262, -0.34202014, 0.0],
            [0.34202014, 0.93969262, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    body = Pose(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    # A point directly in front of the body expressed in the tracking world.
    hand = Pose(0.40 * tracker_to_body.T[:, 0], np.array([0.0, 0.0, 0.0, 1.0]))

    relative = HeadBase.from_config(
        {"head_tracker_to_body": {"rotation_matrix": tracker_to_body.tolist()}}
    ).relative(body, True, hand)

    np.testing.assert_allclose(relative.position, [0.40, 0.0, 0.0], atol=1e-8)


def test_body_tracker_mount_keeps_the_legacy_axis_map_config() -> None:
    head_base = HeadBase.from_config(
        {
            "head_tracker_to_body": {
                "x": ["z", 1.0],
                "y": ["y", 1.0],
                "z": ["x", -1.0],
            }
        }
    )

    np.testing.assert_allclose(
        head_base._tracker_to_body,
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
    )


def test_identity_steamvr_pose_stays_identity_in_teleop() -> None:
    pose = to_teleop_pose(
        Pose(np.array([0.0, 1.6, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
        "steamvr",
    )
    np.testing.assert_allclose(pose.position, [0.0, 0.0, 1.6])
    # Identity SteamVR (-Z forward) becomes identity teleop (+X forward).
    np.testing.assert_allclose(
        np.abs(np.dot(pose.rotation, [0.0, 0.0, 0.0, 1.0])), 1.0, atol=1e-9
    )
    forward = matrix_from_quat(pose.rotation) @ np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(forward, [1.0, 0.0, 0.0], atol=1e-9)


def test_packet_assigns_body_to_hmd_and_hand_to_right() -> None:
    body = _tracker("body_hmd", (0.1, 1.5, -0.2), serial="MOCK-HEAD-001")
    hand = _tracker("hand_controller", (0.4, 1.4, -0.5), serial="MOCK-HAND-001")
    packet = _packet(body, hand)
    assert packet.hmd.valid and packet.right_controller.valid
    np.testing.assert_allclose(packet.hmd.position, [0.1, 1.5, -0.2])
    np.testing.assert_allclose(packet.right_controller.position, [0.4, 1.4, -0.5])
    # Packet rotation is xyzw; identity last component is w.
    np.testing.assert_allclose(packet.hmd.rotation, [0.0, 0.0, 0.0, 1.0])
    assert not packet.left_controller.valid
