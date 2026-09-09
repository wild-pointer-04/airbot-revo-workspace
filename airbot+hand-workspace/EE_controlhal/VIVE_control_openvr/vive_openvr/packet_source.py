"""OpenVR trackers → PicoTeleopPacket (body = HMD, right_wrist = right controller).

Imports middle_interface protocol only; does not modify it.
Clutch / head-base / workspace stay inside TeleopPipeline.
"""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

from vive_openvr.keyboard_input import (
    KeyboardButtons,
    TeleopButtons,
    TeleopButtonSource,
)
from vive_openvr.tracker_manager import TrackerManager, load_config
from vive_openvr.types import TrackerPose, TrackingStatus

LOGGER = logging.getLogger(__name__)


def _ensure_middle_interface() -> None:
    middle = Path(__file__).resolve().parents[2] / "middle_interface"
    text = str(middle)
    if text not in sys.path:
        sys.path.insert(0, text)


_ensure_middle_interface()

from protocol.pico_packet import (  # noqa: E402
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    ControllerState,
    HeadState,
    PicoTeleopPacket,
)


def _idle_controller() -> ControllerState:
    return ControllerState(
        valid=False,
        position=np.zeros(3, dtype=np.float64),
        rotation=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        linear_velocity=np.zeros(3, dtype=np.float64),
        angular_velocity=np.zeros(3, dtype=np.float64),
        trigger=0.0,
        grip=0.0,
        primary_button=False,
        secondary_button=False,
        joystick=np.zeros(2, dtype=np.float64),
    )


def _pose_ok(pose: Optional[TrackerPose]) -> bool:
    if pose is None:
        return False
    if pose.status in {TrackingStatus.INVALID, TrackingStatus.DISCONNECTED, TrackingStatus.STALE}:
        return False
    return bool(pose.pose_valid and pose.device_connected)


class OpenVRPacketSource:
    """Compatible Vive-style packet source backed by SteamVR / OpenVR."""

    def __init__(
        self,
        manager: TrackerManager,
        *,
        head_role: str = "body",
        hand_role: str = "right_wrist",
        timeout_sec: float = 0.5,
        buttons: KeyboardButtons | TeleopButtonSource | None = None,
        own_buttons: bool | None = None,
    ) -> None:
        self._manager = manager
        self._head_role = str(head_role)
        self._hand_role = str(hand_role)
        self.timeout_sec = float(timeout_sec)
        self._buttons = buttons or TeleopButtonSource()
        self._seq = 0
        self._last_packet_mono = 0.0
        self._owns_buttons = (
            bool(own_buttons) if own_buttons is not None else buttons is None
        )

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "OpenVRPacketSource":
        vive = dict(config.get("vive") or config.get("openvr") or {})
        root = Path(__file__).resolve().parents[1]
        tracker_yaml = Path(vive.get("tracker_config") or (root / "config" / "tracker.yaml"))
        if not tracker_yaml.is_absolute():
            tracker_yaml = root / tracker_yaml
        loaded = load_config(tracker_yaml)
        manager = TrackerManager(loaded)
        return cls(
            manager,
            head_role=str(vive.get("head") or vive.get("body") or "body"),
            hand_role=str(vive.get("hand") or vive.get("right_wrist") or "right_wrist"),
            timeout_sec=float(
                vive.get("timeout_sec")
                or config.get("network", {}).get("timeout_sec", 0.5)
            ),
            buttons=TeleopButtonSource.from_teleop_config(config),
            own_buttons=True,
        )

    def start(self) -> None:
        self._manager.start()
        if self._owns_buttons:
            self._buttons.start()
        LOGGER.info(
            "OpenVR packet source head=%s hand=%s",
            self._head_role,
            self._hand_role,
        )

    def stop(self) -> None:
        if self._owns_buttons:
            self._buttons.stop()
        self._manager.stop()

    def timed_out(self) -> bool:
        if self._last_packet_mono <= 0.0:
            return False
        return (time.monotonic() - self._last_packet_mono) > self.timeout_sec

    def poll_latest(self) -> PicoTeleopPacket | None:
        head = self._manager.get_latest_pose(self._head_role)
        hand = self._manager.get_latest_pose(self._hand_role)
        if head is None and hand is None:
            return None
        buttons = self._buttons.snapshot()
        packet = self._build(head, hand, buttons)
        packet.stamp_receive(source_addr="vive_openvr")
        self._last_packet_mono = time.monotonic()
        return packet

    def _build(
        self,
        head: TrackerPose | None,
        hand: TrackerPose | None,
        buttons: TeleopButtons,
    ) -> PicoTeleopPacket:
        self._seq += 1
        now_ns = time.monotonic_ns()
        if _pose_ok(head):
            assert head is not None
            hmd = HeadState(
                valid=True,
                position=np.asarray(head.position, dtype=np.float64).copy(),
                rotation=np.asarray(head.quaternion, dtype=np.float64).copy(),
            )
        else:
            hmd = HeadState(
                valid=False,
                position=np.zeros(3, dtype=np.float64),
                rotation=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
            )
        right = _idle_controller()
        if _pose_ok(hand):
            assert hand is not None
            right.valid = True
            right.position = np.asarray(hand.position, dtype=np.float64).copy()
            right.rotation = np.asarray(hand.quaternion, dtype=np.float64).copy()
        right.trigger = float(buttons.trigger)
        right.grip = float(buttons.grip)
        right.primary_button = bool(buttons.home)
        left = _idle_controller()
        left.primary_button = bool(buttons.pause)
        return PicoTeleopPacket(
            protocol=PROTOCOL_NAME,
            version=PROTOCOL_VERSION,
            seq=self._seq,
            device_time_ns=now_ns,
            wall_time_unix_ms=int(time.time() * 1000),
            hmd=hmd,
            left_controller=left,
            right_controller=right,
        )
