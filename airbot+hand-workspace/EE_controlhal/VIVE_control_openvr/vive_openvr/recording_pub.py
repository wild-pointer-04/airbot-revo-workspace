"""Publish the CM6K right-pedal state for the hand-feedback collector."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable


log = logging.getLogger(__name__)

HAND_RECORDING_TOPIC = "/teleop/hand_recording"


class HandRecordingPublisher:
    """Publish a fresh Bool heartbeat so a collector can fail closed."""

    def __init__(
        self,
        recording_active: Callable[[], bool],
        *,
        topic: str = HAND_RECORDING_TOPIC,
        hz: float = 20.0,
    ) -> None:
        if hz <= 0:
            raise ValueError("hand-recording publish rate must be positive")
        self._recording_active = recording_active
        self._topic = topic
        self._period = 1.0 / hz
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._node = None
        self._publisher = None

    def start(self) -> None:
        import rclpy
        from std_msgs.msg import Bool

        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("hand_recording_gate")
        self._publisher = self._node.create_publisher(Bool, self._topic, 10)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="hand-recording-pub",
            daemon=True,
        )
        self._thread.start()
        log.info("Publishing %s at %.0f Hz", self._topic, 1.0 / self._period)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._publisher is not None:
            self._publish(False)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        self._publisher = None

    def _run(self) -> None:
        next_tick = time.monotonic()
        while not self._stop.is_set():
            self._publish(self._recording_active())
            next_tick += self._period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def _publish(self, active: bool) -> None:
        if self._publisher is None:
            return
        from std_msgs.msg import Bool

        message = Bool()
        message.data = bool(active)
        self._publisher.publish(message)
