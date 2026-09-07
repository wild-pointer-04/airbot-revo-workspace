"""Two VIVE trackers → PicoTeleopPacket (body = HMD, handheld = right controller)."""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

import numpy as np

from keyboard import KeyboardButtons, TeleopButtons
from vive_tracking.tracker_manager import TrackerManager, create_backend, load_config
from vive_tracking.types import TrackerPose

logger = logging.getLogger(__name__)


def _ensure_middle_interface() -> None:
    import sys

    middle = Path(__file__).resolve().parents[1] / "middle_interface"
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


def _wxyz_to_xyzw(orientation) -> np.ndarray:
    w, x, y, z = orientation.as_wxyz()
    return np.array([x, y, z, w], dtype=np.float64)


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


class VivePacketSource:
    def __init__(
        self,
        manager: TrackerManager,
        *,
        head_name: str,
        hand_name: str,
        timeout_sec: float = 0.5,
        buttons: KeyboardButtons | None = None,
    ) -> None:
        self._manager = manager
        self._head_name = head_name
        self._hand_name = hand_name
        self.timeout_sec = float(timeout_sec)
        self._buttons = buttons or KeyboardButtons()
        self._seq = 0
        self._last_packet_mono = 0.0
        self._owns_buttons = buttons is None

    @classmethod
    def from_config(cls, config: dict[str, Any], *, mock: bool = False) -> "VivePacketSource":
        vive = dict(config.get("vive") or {})
        root = Path(__file__).resolve().parent
        tracker_yaml = Path(vive.get("tracker_config") or (root / "config" / "tracker.yaml"))
        if not tracker_yaml.is_absolute():
            tracker_yaml = root / tracker_yaml
        loaded = load_config(tracker_yaml)
        if mock:
            loaded["backend"] = "mock"
        manager = TrackerManager(create_backend(loaded), loaded)
        return cls(
            manager,
            head_name=str(vive.get("head") or "body_hmd"),
            hand_name=str(vive.get("hand") or "hand_controller"),
            timeout_sec=float(vive.get("timeout_sec") or config.get("network", {}).get("timeout_sec", 0.5)),
        )

    def start(self) -> None:
        self._manager.start()
        if self._owns_buttons:
            self._buttons.start()
        logger.info(
            "VIVE source head=%s hand=%s",
            self._head_name,
            self._hand_name,
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
        head = self._manager.get_pose(self._head_name)
        hand = self._manager.get_pose(self._hand_name)
        if head is None and hand is None:
            return None
        buttons = self._buttons.snapshot()
        packet = self._build(head, hand, buttons)
        packet.stamp_receive(source_addr="vive")
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
        if head is not None and head.tracking_valid:
            hmd = HeadState(
                valid=True,
                position=np.array(head.position_m.as_list(), dtype=np.float64),
                rotation=_wxyz_to_xyzw(head.orientation),
            )
        else:
            hmd = HeadState(
                valid=False,
                position=np.zeros(3, dtype=np.float64),
                rotation=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
            )
        right = _idle_controller()
        if hand is not None and hand.tracking_valid:
            right.valid = True
            right.position = np.array(hand.position_m.as_list(), dtype=np.float64)
            right.rotation = _wxyz_to_xyzw(hand.orientation)
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
