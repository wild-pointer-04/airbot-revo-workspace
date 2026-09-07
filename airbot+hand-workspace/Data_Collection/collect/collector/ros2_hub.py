"""Subscribe to /arm/state and push into SyncBuffer."""

from __future__ import annotations

import json
import logging
import threading
from typing import Any, Optional

from collector.ros2_codec import ARM_TOPIC, decode_state
from collector.sync_buffer import SyncBuffer

log = logging.getLogger(__name__)

SRC_ARM = "arm"


class Ros2StateHub:
    def __init__(
        self,
        buffer: SyncBuffer,
        *,
        arm_topic: str = ARM_TOPIC,
    ) -> None:
        self._buffer = buffer
        self._arm_topic = arm_topic
        self._node = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        from collector.ros_env import ensure_rclpy

        ensure_rclpy()
        import rclpy
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("collect_state")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._node.create_subscription(String, self._arm_topic, self._on_arm, qos)
        self._stop.clear()
        self._thread = threading.Thread(target=self._spin, name="collect-ros2", daemon=True)
        self._thread.start()
        log.info("ROS2 state: %s", self._arm_topic)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

    def _spin(self) -> None:
        import rclpy

        while not self._stop.is_set() and self._node is not None:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception:
                log.exception("ROS2 spin failed")
                break

    def _on_arm(self, msg: Any) -> None:
        try:
            data = decode_state(msg.data)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            log.warning("bad arm state: %s", exc)
            return
        ns = data.get("host_monotonic_ns")
        ts = (int(ns) / 1_000_000_000.0) if ns is not None else None
        self._buffer.update(SRC_ARM, data, timestamp=ts)
