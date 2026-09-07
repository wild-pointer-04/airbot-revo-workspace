from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(MIDDLE))

from keyboard import KeyboardButtons
from packet_source import VivePacketSource
from vive_tracking.tracker_manager import TrackerManager, create_backend, load_config


def test_mock_source_builds_pico_packet() -> None:
    cfg = load_config(ROOT / "config" / "tracker.mock.yaml")
    manager = TrackerManager(create_backend(cfg), cfg)
    buttons = KeyboardButtons()
    buttons.set_buttons(trigger=1.0, home=True)
    source = VivePacketSource(
        manager,
        head_name="body_hmd",
        hand_name="hand_controller",
        buttons=buttons,
    )
    source.start()
    try:
        packet = None
        for _ in range(20):
            packet = source.poll_latest()
            if packet is not None and packet.hmd.valid and packet.right_controller.valid:
                break
        assert packet is not None
        assert packet.protocol == "pico_teleop"
        assert packet.hmd.valid
        assert packet.right_controller.valid
        assert packet.right_controller.trigger == 1.0
        assert packet.right_controller.primary_button is True
        np.testing.assert_allclose(np.linalg.norm(packet.hmd.rotation), 1.0, atol=0.05)
        # xyzw: last component is w for identity-ish mock
        assert packet.right_controller.rotation.shape == (4,)
    finally:
        source.stop()
