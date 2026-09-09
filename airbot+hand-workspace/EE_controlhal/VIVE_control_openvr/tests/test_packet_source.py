"""Unit tests for OpenVRPacketSource with a fake manager (no SteamVR)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(MIDDLE))

from vive_openvr.keyboard_input import KeyboardButtons
from vive_openvr.packet_source import OpenVRPacketSource
from vive_openvr.pose_utils import se3_from_rt
from vive_openvr.types import TrackerPose, TrackingStatus


class _FakeManager:
    def __init__(self) -> None:
        self.started = False
        T_b = se3_from_rt(np.eye(3), np.array([0.0, 0.0, 1.0]))
        T_h = se3_from_rt(np.eye(3), np.array([0.2, 0.0, 1.0]))
        self._poses = {
            "body": TrackerPose(
                serial="LHR-BODY",
                role="body",
                timestamp=1.0,
                transform=T_b,
                position=T_b[:3, 3].copy(),
                quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
                pose_valid=True,
                device_connected=True,
                status=TrackingStatus.OK,
            ),
            "right_wrist": TrackerPose(
                serial="LHR-HAND",
                role="right_wrist",
                timestamp=1.0,
                transform=T_h,
                position=T_h[:3, 3].copy(),
                quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
                pose_valid=True,
                device_connected=True,
                status=TrackingStatus.OK,
            ),
        }

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def get_latest_pose(self, role: str):
        return self._poses.get(role)


def test_packet_source_builds_pico_packet() -> None:
    buttons = KeyboardButtons()
    buttons.set_buttons(trigger=1.0, home=True)
    source = OpenVRPacketSource(
        _FakeManager(),  # type: ignore[arg-type]
        head_role="body",
        hand_role="right_wrist",
        buttons=buttons,
    )
    source.start()
    try:
        packet = source.poll_latest()
        assert packet is not None
        assert packet.protocol == "pico_teleop"
        assert packet.hmd.valid
        assert packet.right_controller.valid
        assert packet.right_controller.trigger == 1.0
        assert packet.right_controller.primary_button is True
        np.testing.assert_allclose(packet.hmd.position, [0.0, 0.0, 1.0], atol=1e-9)
        np.testing.assert_allclose(packet.right_controller.position, [0.2, 0.0, 1.0], atol=1e-9)
        np.testing.assert_allclose(np.linalg.norm(packet.hmd.rotation), 1.0, atol=1e-6)
    finally:
        source.stop()


def test_stale_pose_marks_invalid() -> None:
    mgr = _FakeManager()
    stale = mgr._poses["right_wrist"].copy()
    stale.status = TrackingStatus.STALE
    stale.pose_valid = False
    mgr._poses["right_wrist"] = stale
    source = OpenVRPacketSource(mgr, buttons=KeyboardButtons())  # type: ignore[arg-type]
    source.start()
    try:
        packet = source.poll_latest()
        assert packet is not None
        assert packet.hmd.valid
        assert not packet.right_controller.valid
    finally:
        source.stop()
