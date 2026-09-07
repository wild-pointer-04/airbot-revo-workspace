"""Arm state ingest over the rosbridge websocket path."""

from __future__ import annotations

import sys
import time
from pathlib import Path

COLLECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COLLECT))

import pytest

from collector.arm_adapter import missing_arm
from collector.ros2_codec import encode_state
from collector.rosbridge_hub import RosbridgeStateHub
from collector.sync_buffer import SyncBuffer


def _hub(buf: SyncBuffer) -> RosbridgeStateHub:
    return RosbridgeStateHub(buf, host="localhost", port=9090)


def test_decodes_websocket_payload_into_buffer() -> None:
    buf = SyncBuffer(["arm"], max_age_s=2.0)
    arm = missing_arm()
    arm["host_monotonic_ns"] = time.monotonic_ns()

    _hub(buf)._on_arm({"data": encode_state(arm)})

    snap = buf.get_snapshot()
    assert snap["arm"] is not None
    assert list(snap["arm"]["observation.state.arm_joint"]) == [0.0] * 6
    assert snap["arm"]["observation.state.arm_valid"][0] == 0.0


def test_foreign_monotonic_clock_falls_back_to_arrival_time() -> None:
    """The publisher runs in another container, so its monotonic origin differs.

    Trusting that stamp would make every frame look stale and drop the episode.
    """
    buf = SyncBuffer(["arm"], max_age_s=0.2)
    arm = missing_arm()
    arm["host_monotonic_ns"] = 1  # ~0 s, far from this process' monotonic clock

    _hub(buf)._on_arm({"data": encode_state(arm)})

    assert buf.get_snapshot()["arm"] is not None


def test_malformed_payload_is_dropped() -> None:
    buf = SyncBuffer(["arm"], max_age_s=2.0)
    hub = _hub(buf)

    hub._on_arm({"data": "not json"})
    hub._on_arm({"no_data_key": ""})

    assert buf.get_snapshot()["arm"] is None


def test_start_without_server_raises_with_hint() -> None:
    pytest.importorskip("roslibpy")
    hub = RosbridgeStateHub(
        SyncBuffer(["arm"], max_age_s=2.0),
        host="127.0.0.1",
        port=9099,  # nothing listening
        connect_timeout_s=1.0,
    )
    with pytest.raises(RuntimeError, match="rosbridge not reachable"):
        hub.start()
