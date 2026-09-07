"""ROS2 JSON state codec and buffer ingest."""

from __future__ import annotations

import sys
import time
from pathlib import Path

COLLECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COLLECT))

from collector.arm_adapter import missing_arm
from collector.ros2_codec import decode_state, encode_state
from collector.ros2_hub import Ros2StateHub
from collector.sync_buffer import SyncBuffer


class _Msg:
    def __init__(self, data: str) -> None:
        self.data = data


def test_arm_roundtrip() -> None:
    src = missing_arm()
    src["host_monotonic_ns"] = 123456789
    back = decode_state(encode_state(src))
    assert back["host_monotonic_ns"] == 123456789
    assert list(back["observation.state.arm_joint"]) == [0.0] * 6
    assert list(back["observation.state.arm_ee_position"]) == [0.0, 0.0, 0.0]
    assert abs(float(back["observation.state.arm_ee_orientation"][3]) - 1.0) < 1e-6


def test_hub_pushes_buffer() -> None:
    buf = SyncBuffer(["arm"], max_age_s=2.0)
    hub = Ros2StateHub(buf)
    ns = time.monotonic_ns()
    arm = missing_arm()
    arm["host_monotonic_ns"] = ns
    hub._on_arm(_Msg(encode_state(arm)))
    snap = buf.get_snapshot()
    assert snap["arm"] is not None
    assert snap["arm"]["observation.state.arm_valid"][0] == 0.0
