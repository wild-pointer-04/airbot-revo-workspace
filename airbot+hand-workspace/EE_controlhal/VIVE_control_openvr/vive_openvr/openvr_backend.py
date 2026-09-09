"""OpenVR GenericTracker pose reader.

Responsibilities only:
  - init / shutdown OpenVR
  - discover GenericTracker devices by permanent serial
  - poll 6-DoF poses into a thread-safe latest-pose cache

Does not: robot control, body-relative math, packet building, IK.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

import numpy as np

from vive_openvr.pose_utils import (
    enforce_quaternion_continuity,
    openvr_matrix34_to_se3,
    rotation_matrix_to_quaternion,
    transform_translation,
)
from vive_openvr.types import DiscoveredTracker, TrackerPose, TrackingStatus

LOGGER = logging.getLogger(__name__)


class OpenVRInitError(RuntimeError):
    """SteamVR / OpenVR failed to initialize."""


class OpenVRBackend:
    def __init__(
        self,
        *,
        polling_rate_hz: float = 120.0,
        universe: str = "standing",
        role_by_serial: Optional[dict[str, str]] = None,
    ) -> None:
        self._polling_rate_hz = max(1.0, float(polling_rate_hz))
        self._universe_name = str(universe or "standing").lower()
        self._role_by_serial = dict(role_by_serial or {})
        self._openvr: Any = None
        self._vr_system: Any = None
        self._universe: Any = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._latest: dict[str, TrackerPose] = {}
        self._prev_quat: dict[str, np.ndarray] = {}
        self._serial_by_index: dict[int, str] = {}

    @property
    def is_running(self) -> bool:
        return self._running

    def initialize(self) -> None:
        if self._vr_system is not None:
            return
        try:
            import openvr
        except ImportError as exc:
            raise OpenVRInitError(
                "Python package 'openvr' is not installed. "
                "pip install openvr  (and start SteamVR first)."
            ) from exc

        app_type = getattr(openvr, "VRApplication_Background", None)
        if app_type is None:
            app_type = openvr.VRApplication_Other

        try:
            openvr.init(app_type)
        except Exception as exc:
            detail = str(exc).strip() or repr(exc)
            raise OpenVRInitError(
                "openvr.init failed. Is SteamVR running with Null Driver / "
                f"requireHmd=false? detail: {detail}"
            ) from exc

        try:
            system = openvr.VRSystem()
        except Exception as exc:
            try:
                openvr.shutdown()
            except Exception:
                pass
            raise OpenVRInitError(f"openvr.VRSystem() failed: {exc}") from exc

        if self._universe_name in {"seated", "trackinguniverseseated"}:
            universe = openvr.TrackingUniverseSeated
        else:
            universe = openvr.TrackingUniverseStanding

        self._openvr = openvr
        self._vr_system = system
        self._universe = universe
        LOGGER.info(
            "OpenVR initialized app_type=%s universe=%s",
            app_type,
            self._universe_name,
        )

    def start(self) -> None:
        if self._running:
            return
        self.initialize()
        self._stop.clear()
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop,
            name="openvr-tracker-reader",
            daemon=True,
        )
        self._thread.start()
        LOGGER.info("OpenVRBackend polling at %.1f Hz", self._polling_rate_hz)

    def stop(self) -> None:
        self._stop.set()
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._openvr is not None:
            try:
                self._openvr.shutdown()
                LOGGER.info("OpenVR shutdown complete")
            except Exception as exc:
                LOGGER.warning("openvr.shutdown raised: %s", exc)
            self._openvr = None
            self._vr_system = None

    def __enter__(self) -> "OpenVRBackend":
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    def list_discovered(self) -> list[DiscoveredTracker]:
        """Enumerate GenericTrackers once (does not require poll thread)."""
        self.initialize()
        assert self._openvr is not None and self._vr_system is not None
        out: list[DiscoveredTracker] = []
        poses = self._fetch_poses()
        for index in range(self._openvr.k_unMaxTrackedDeviceCount):
            cls = self._vr_system.getTrackedDeviceClass(index)
            if cls != self._openvr.TrackedDeviceClass_GenericTracker:
                continue
            serial = self._read_serial(index)
            if not serial:
                continue
            pose = poses[index] if index < len(poses) else None
            connected = bool(pose.bDeviceIsConnected) if pose is not None else False
            valid = bool(pose.bPoseIsValid) if pose is not None else False
            out.append(
                DiscoveredTracker(
                    serial=serial,
                    device_index=index,
                    connected=connected,
                    pose_valid=valid,
                    role=self._role_by_serial.get(serial, ""),
                )
            )
            self._serial_by_index[index] = serial
        return out

    def get_latest_pose(self, serial: str) -> Optional[TrackerPose]:
        with self._lock:
            pose = self._latest.get(serial)
            return None if pose is None else pose.copy()

    def get_all_latest(self) -> dict[str, TrackerPose]:
        with self._lock:
            return {k: v.copy() for k, v in self._latest.items()}

    def _poll_loop(self) -> None:
        period = 1.0 / self._polling_rate_hz
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self._poll_once()
            except Exception:
                LOGGER.exception("OpenVR poll failed")
            elapsed = time.monotonic() - t0
            remaining = period - elapsed
            if remaining > 0.0:
                self._stop.wait(remaining)

    def _poll_once(self) -> None:
        assert self._openvr is not None and self._vr_system is not None
        poses = self._fetch_poses()
        now = time.monotonic()
        for index in range(self._openvr.k_unMaxTrackedDeviceCount):
            cls = self._vr_system.getTrackedDeviceClass(index)
            if cls != self._openvr.TrackedDeviceClass_GenericTracker:
                continue
            serial = self._serial_by_index.get(index) or self._read_serial(index)
            if not serial:
                continue
            self._serial_by_index[index] = serial
            raw = poses[index]
            sample = self._pose_from_raw(serial, index, raw, now)
            with self._lock:
                self._latest[serial] = sample

    def _fetch_poses(self) -> Any:
        assert self._vr_system is not None and self._openvr is not None
        return self._vr_system.getDeviceToAbsoluteTrackingPose(
            self._universe,
            0.0,
            self._openvr.k_unMaxTrackedDeviceCount,
        )

    def _read_serial(self, index: int) -> str:
        assert self._vr_system is not None and self._openvr is not None
        try:
            raw = self._vr_system.getStringTrackedDeviceProperty(
                index, self._openvr.Prop_SerialNumber_String
            )
        except Exception:
            return ""
        if raw is None:
            return ""
        if isinstance(raw, bytes):
            text = raw.decode("utf-8", errors="replace")
        else:
            text = str(raw)
        return text.strip()

    def _pose_from_raw(
        self, serial: str, index: int, raw: Any, timestamp: float
    ) -> TrackerPose:
        connected = bool(raw.bDeviceIsConnected)
        valid = bool(raw.bPoseIsValid)
        tracking_result = int(raw.eTrackingResult)
        role = self._role_by_serial.get(serial, "")

        if not connected:
            status = TrackingStatus.DISCONNECTED
            T = np.eye(4, dtype=np.float64)
            quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        elif not valid:
            status = TrackingStatus.INVALID
            T = np.eye(4, dtype=np.float64)
            quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        else:
            status = TrackingStatus.OK
            T = openvr_matrix34_to_se3(raw.mDeviceToAbsoluteTracking)
            quat = rotation_matrix_to_quaternion(T[:3, :3])
            prev = self._prev_quat.get(serial)
            quat = enforce_quaternion_continuity(quat, prev)
            self._prev_quat[serial] = quat

        return TrackerPose(
            serial=serial,
            role=role,
            timestamp=timestamp,
            transform=T,
            position=transform_translation(T),
            quaternion=quat,
            pose_valid=valid and connected and status == TrackingStatus.OK,
            device_connected=connected,
            tracking_result=tracking_result,
            device_index=index,
            status=status,
        )
