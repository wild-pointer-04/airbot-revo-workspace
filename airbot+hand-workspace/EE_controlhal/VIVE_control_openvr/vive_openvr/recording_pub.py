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
        host: str = "localhost",
        port: int = 9090,
    ) -> None:
        if hz <= 0:
            raise ValueError("hand-recording publish rate must be positive")
        self._recording_active = recording_active
        self._topic = topic
        self._period = 1.0 / hz
        self._host = host
        self._port = int(port)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ros = None
        self._publisher = None

    def start(self) -> None:
        try:
            import roslibpy
        except ImportError as exc:
            raise RuntimeError("Install roslibpy in the VIVE environment.") from exc

        self._ros = roslibpy.Ros(host=self._host, port=self._port)
        try:
            self._ros.run(timeout=10.0)
            connected = self._ros.is_connected
        except Exception:
            connected = False
        if not connected:
            self.stop()
            raise RuntimeError(
                f"rosbridge is unavailable at ws://{self._host}:{self._port}"
            )
        self._publisher = roslibpy.Topic(
            self._ros,
            self._topic,
            "std_msgs/Bool",
        )
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="hand-recording-pub",
            daemon=True,
        )
        self._thread.start()
        log.info(
            "Publishing %s via ws://%s:%d at %.0f Hz",
            self._topic,
            self._host,
            self._port,
            1.0 / self._period,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._publisher is not None:
            try:
                self._publish(False)
                self._publisher.unadvertise()
            except Exception:
                pass
        self._publisher = None
        if self._ros is not None:
            try:
                self._ros.terminate()
            except Exception:
                pass
            self._ros = None

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
        import roslibpy

        self._publisher.publish(roslibpy.Message({"data": bool(active)}))
