"""Subscribe to /arm/state over a rosbridge websocket and push into SyncBuffer.

Same payload as Ros2StateHub (std_msgs/String carrying JSON), so decode_state is
reused. This path exists because the teleop container runs ROS 2 Humble on Python
3.10 while the vendored LeRobot writer needs Python >= 3.12: the collector runs in
its own 3.12 environment and reaches ROS through rosbridge instead of rclpy.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import Optional

from collector.ros2_codec import ARM_TOPIC, decode_state
from collector.sync_buffer import SyncBuffer

log = logging.getLogger(__name__)

SRC_ARM = "arm"


class RosbridgeStateHub:
    def __init__(
        self,
        buffer: SyncBuffer,
        *,
        arm_topic: str = ARM_TOPIC,
        host: str = "localhost",
        port: int = 9090,
        connect_timeout_s: float = 10.0,
    ) -> None:
        self._buffer = buffer
        self._arm_topic = arm_topic
        self._host = host
        self._port = int(port)
        self._connect_timeout_s = float(connect_timeout_s)
        self._ros = None
        self._topic = None
        self._stale_warned = False

    def start(self) -> None:
        try:
            import roslibpy
        except ImportError as exc:
            raise RuntimeError(
                "roslibpy is required for state_source=rosbridge. "
                "Install it in the collector environment."
            ) from exc

        self._ros = roslibpy.Ros(host=self._host, port=self._port)
        try:
            self._ros.run(timeout=self._connect_timeout_s)
            connected = self._ros.is_connected
        except Exception:
            connected = False
        if not connected:
            self.stop()
            raise RuntimeError(
                f"rosbridge not reachable at ws://{self._host}:{self._port}. "
                "Start it in the teleop container: "
                "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
            )
        self._topic = roslibpy.Topic(
            self._ros,
            self._arm_topic,
            "std_msgs/String",
            throttle_rate=0,
            queue_length=1,
        )
        self._topic.subscribe(self._on_arm)
        log.info("rosbridge state: %s via ws://%s:%d", self._arm_topic, self._host, self._port)

    def stop(self) -> None:
        if self._topic is not None:
            try:
                self._topic.unsubscribe()
            except Exception:
                pass
            self._topic = None
        if self._ros is not None:
            try:
                self._ros.terminate()
            except Exception:
                pass
            self._ros = None

    def _on_arm(self, msg: dict) -> None:
        try:
            data = decode_state(msg["data"])
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
            log.warning("bad arm state: %s", exc)
            return
        ns = data.get("host_monotonic_ns")
        ts: Optional[float] = None
        if ns is not None:
            # The publisher stamps its own monotonic clock. Across processes that
            # shares an origin only inside one container; reject it when the skew
            # would make every frame look stale to SyncBuffer.
            candidate = int(ns) / 1_000_000_000.0
            if abs(candidate - time.monotonic()) < 1.0:
                ts = candidate
            elif not self._stale_warned:
                self._stale_warned = True
                log.info(
                    "publisher monotonic clock differs from this process; "
                    "using local arrival time for arm state"
                )
        self._buffer.update(SRC_ARM, data, timestamp=ts)

    @property
    def connected(self) -> bool:
        return bool(self._ros is not None and self._ros.is_connected)
