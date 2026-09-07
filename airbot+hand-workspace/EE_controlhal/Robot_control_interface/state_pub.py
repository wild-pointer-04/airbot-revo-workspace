"""Publish arm joint + EE pose on /arm/state for data collection."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable

log = logging.getLogger(__name__)

ARM_TOPIC = "/arm/state"


def encode_state(data: dict[str, Any]) -> str:
    import json

    out: dict[str, Any] = {}
    for key, value in data.items():
        if hasattr(value, "tolist"):
            out[key] = value.tolist()
        elif isinstance(value, (int, float, str, bool)) or value is None:
            out[key] = value
        else:
            out[key] = value
    return json.dumps(out, ensure_ascii=False)


class ArmStatePublisher:
    def __init__(
        self,
        get_state: Callable[[], dict[str, Any]],
        *,
        topic: str = ARM_TOPIC,
        hz: float = 50.0,
    ) -> None:
        self._get_state = get_state
        self._topic = topic
        self._period = 1.0 / max(float(hz), 1.0)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._node = None
        self._pub = None

    def start(self) -> None:
        try:
            from ros_env import ensure_ros

            ensure_ros()
        except Exception:
            # AIRBOT SDK path does not need X5_ws; rclpy from the base ROS 2 install is enough.
            pass
        try:
            import rclpy
            from std_msgs.msg import String
        except ImportError as exc:
            log.warning("rclpy unavailable; arm state topic disabled: %s", exc)
            return
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("arm_state")
        self._pub = self._node.create_publisher(String, self._topic, 10)
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="arm-state-pub", daemon=True)
        self._thread.start()
        log.info("Publishing %s at %.0f Hz", self._topic, 1.0 / self._period)

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

    def _loop(self) -> None:
        import rclpy
        from std_msgs.msg import String

        next_tick = time.monotonic()
        while not self._stop.is_set() and self._node is not None and self._pub is not None:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(next_tick - now, 0.02))
                continue
            try:
                payload = self._get_state()
                msg = String()
                msg.data = encode_state(payload)
                self._pub.publish(msg)
                rclpy.spin_once(self._node, timeout_sec=0.0)
            except Exception:
                log.exception("arm state publish failed")
            next_tick = now + self._period
