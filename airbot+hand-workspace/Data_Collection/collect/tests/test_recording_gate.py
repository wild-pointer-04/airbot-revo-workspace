from __future__ import annotations

import sys
import time
from pathlib import Path

COLLECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COLLECT))

from collector.collector import Collector


def _cfg() -> dict:
    return {
        "task": "gate_test",
        "mock": True,
        "state_source": "local",
        "data_root": "/tmp",
        "dataset": {"repo_id": "gate_test", "fps": 10, "use_videos": False},
        "cameras": {
            "cam_global": {
                "type": "opencv",
                "device": "/dev/v4l/by-id/usb-LRCP_V1080P_LRCP_V1080P-video-index0",
                "width": 640,
                "height": 480,
                "fps": 30,
            }
        },
        "hand": {"enabled": True, "side": "right"},
        "recording_gate": {
            "enabled": True,
            "topic": "/teleop/hand_recording",
            "max_age_s": 0.5,
        },
        "rosbridge": {"host": "localhost", "port": 9090},
    }


def test_recording_gate_requires_a_fresh_true_message() -> None:
    collector = Collector(_cfg(), mock=True)
    # mock disables the live gate subscription, but the freshness helper still works.
    collector._gate_enabled = True
    collector._recording_gate_cb({"data": True})

    assert collector._gate_is_active(time.monotonic())
    collector._gate_arrival_s = time.monotonic() - 0.51
    assert not collector._gate_is_active(time.monotonic())


def test_recording_gate_ignores_non_bool_payload() -> None:
    collector = Collector(_cfg(), mock=True)
    collector._gate_enabled = True
    collector._recording_gate_cb({"data": "true"})
    assert not collector._gate_is_active(time.monotonic())
