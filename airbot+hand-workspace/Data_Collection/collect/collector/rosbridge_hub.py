"""Subscribe to /arm/state + Revo2 hand topics over rosbridge into SyncBuffer."""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import Optional

from collector.hand_adapter import (
    joint_state_to_observation,
    missing_hand,
    tactile_from_dynamic_joint_state,
)
from collector.ros2_codec import ARM_TOPIC, decode_state
from collector.sync_buffer import SyncBuffer

log = logging.getLogger(__name__)

SRC_ARM = "arm"
SRC_HAND = "hand"


class RosbridgeStateHub:
    def __init__(
        self,
        buffer: SyncBuffer,
        *,
        arm_topic: str = ARM_TOPIC,
        hand_side: str = "right",
        hand_topic: str | None = None,
        hand_tactile_topic: str | None = None,
        host: str = "localhost",
        port: int = 9090,
        connect_timeout_s: float = 10.0,
        enable_hand: bool = True,
    ) -> None:
        self._buffer = buffer
        self._arm_topic = arm_topic
        self._hand_side = hand_side
        self._hand_topic = hand_topic or (
            f"/revo2_{hand_side}/revo2_joint_state/joint_states"
        )
        self._hand_tactile_topic = hand_tactile_topic or (
            f"/revo2_{hand_side}/revo2_joint_state/dynamic_joint_states"
        )
        self._enable_hand = enable_hand
        self._host = host
        self._port = int(port)
        self._connect_timeout_s = float(connect_timeout_s)
        self._ros = None
        self._topics: list = []
        self._stale_warned = False
        self._latest_hand = missing_hand(hand_side)
        self._latest_tactile = self._latest_hand[
            f"observation.state.hand_{hand_side}_tactile"
        ].copy()
        self._hand_lock = threading.Lock()

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

        arm = roslibpy.Topic(
            self._ros,
            self._arm_topic,
            "std_msgs/String",
            throttle_rate=0,
            queue_length=1,
        )
        arm.subscribe(self._on_arm)
        self._topics.append(arm)
        log.info("rosbridge state: %s via ws://%s:%d", self._arm_topic, self._host, self._port)

        if self._enable_hand:
            hand = roslibpy.Topic(
                self._ros,
                self._hand_topic,
                "sensor_msgs/JointState",
                throttle_rate=0,
                queue_length=1,
            )
            hand.subscribe(self._on_hand)
            self._topics.append(hand)
            tactile = roslibpy.Topic(
                self._ros,
                self._hand_tactile_topic,
                "control_msgs/DynamicJointState",
                throttle_rate=0,
                queue_length=1,
            )
            tactile.subscribe(self._on_tactile)
            self._topics.append(tactile)
            log.info(
                "rosbridge hand: %s + %s",
                self._hand_topic,
                self._hand_tactile_topic,
            )

    def stop(self) -> None:
        for topic in self._topics:
            try:
                topic.unsubscribe()
            except Exception:
                pass
        self._topics = []
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

    def _on_hand(self, msg: dict) -> None:
        state = joint_state_to_observation(msg, self._hand_side)
        with self._hand_lock:
            state[
                f"observation.state.hand_{self._hand_side}_tactile"
            ] = self._latest_tactile.copy()
            self._latest_hand = state
            self._buffer.update(SRC_HAND, state)

    def _on_tactile(self, msg: dict) -> None:
        tactile = tactile_from_dynamic_joint_state(msg, self._hand_side)
        with self._hand_lock:
            self._latest_tactile = tactile
            state = dict(self._latest_hand)
            state[f"observation.state.hand_{self._hand_side}_tactile"] = tactile.copy()
            self._latest_hand = state
            self._buffer.update(SRC_HAND, state)

    @property
    def connected(self) -> bool:
        return bool(self._ros is not None and self._ros.is_connected)
