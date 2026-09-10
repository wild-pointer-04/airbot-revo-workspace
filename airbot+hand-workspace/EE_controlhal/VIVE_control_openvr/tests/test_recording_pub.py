from __future__ import annotations

import sys
import time
from types import SimpleNamespace


ROOT = __import__("pathlib").Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_openvr.recording_pub import HandRecordingPublisher


def test_publishes_bool_over_rosbridge_and_finishes_released(monkeypatch) -> None:
    published: list[dict[str, bool]] = []

    class FakeRos:
        def __init__(self, *, host: str, port: int) -> None:
            assert (host, port) == ("bridge-host", 9010)
            self.is_connected = False
            self.terminated = False

        def run(self, *, timeout: float) -> None:
            assert timeout == 10.0
            self.is_connected = True

        def terminate(self) -> None:
            self.terminated = True

    class FakeTopic:
        def __init__(self, ros: FakeRos, name: str, message_type: str) -> None:
            assert ros.is_connected
            assert name == "/teleop/hand_recording"
            assert message_type == "std_msgs/Bool"
            self.unadvertised = False

        def publish(self, message: dict[str, bool]) -> None:
            published.append(message)

        def unadvertise(self) -> None:
            self.unadvertised = True

    fake_roslibpy = SimpleNamespace(
        Ros=FakeRos,
        Topic=FakeTopic,
        Message=lambda payload: payload,
    )
    monkeypatch.setitem(sys.modules, "roslibpy", fake_roslibpy)

    publisher = HandRecordingPublisher(
        lambda: True,
        host="bridge-host",
        port=9010,
        hz=200.0,
    )
    publisher.start()
    time.sleep(0.01)
    publisher.stop()

    assert published
    assert published[-1] == {"data": False}
