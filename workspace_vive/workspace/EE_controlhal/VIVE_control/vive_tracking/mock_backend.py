"""Hardware-free pose source for development and tests."""

from __future__ import annotations

import csv
import logging
import math
import queue
import threading
import time
from pathlib import Path
from typing import Any, Optional

from vive_tracking.backend import TrackerBackend
from vive_tracking.exceptions import ConfigError
from vive_tracking.pose_utils import (
    axis_angle_to_quaternion,
    enforce_quaternion_continuity,
    interpolate_pose,
    normalize_quaternion,
)
from vive_tracking.timestamp import host_monotonic_ns, seconds_to_monotonic_ns
from vive_tracking.types import (
    IDENTITY_QUATERNION,
    Quaternion,
    TrackerDevice,
    TrackerPose,
    Vec3,
)

LOGGER = logging.getLogger(__name__)

KNOWN_MOCK_SERIALS = (
    "MOCK-HAND-001",
    "MOCK-BENCH-A",
    "MOCK-BENCH-B",
    "MOCK-LEFT-001",
    "MOCK-RIGHT-001",
    "MOCK-HEAD-001",
)

_BENCH_ROLES = {"bench_origin", "bench_x"}
_DEFAULT_BENCH_POS = {
    "bench_origin": (0.10, -0.20, 0.02),
    "bench_x": (1.30, -0.20, 0.02),
}

_STATIC_X = 0.40
_STATIC_Y = 0.00
_STATIC_Z = 0.50
_DEFAULT_RATE_HZ = 100.0
_NS_PER_S = 1_000_000_000


class MockBackend(TrackerBackend):
    """Emits poses on a dedicated reader thread.

    Modes: static, sinusoidal, trajectory, dropout.
    Dropout can also overlay any other mode via mock.dropout.enabled.
    """

    def __init__(self, config: dict[str, Any], trackers: list[dict[str, Any]]) -> None:
        mock_cfg = dict(config.get("mock") or {})
        queue_cfg = dict(config.get("queue") or {})
        self._mode = str(mock_cfg.get("mode", "sinusoidal")).lower()
        self._rate_hz = float(mock_cfg.get("update_rate_hz", _DEFAULT_RATE_HZ))
        if self._rate_hz <= 0.0:
            raise ConfigError("mock.update_rate_hz must be > 0")
        self._playback_speed = float(mock_cfg.get("playback_speed", 1.0))
        dropout_cfg = dict(mock_cfg.get("dropout") or {})
        self._dropout_enabled = bool(dropout_cfg.get("enabled", False)) or self._mode == "dropout"
        self._dropout_duration_s = float(dropout_cfg.get("duration_ms", 100)) / 1000.0
        self._dropout_period_s = float(dropout_cfg.get("period_s", 2.0))
        serials = dropout_cfg.get("serials") or []
        self._dropout_serials = {str(s) for s in serials}
        self._motion_mode = "sinusoidal" if self._mode == "dropout" else self._mode
        if self._motion_mode not in {"static", "sinusoidal", "trajectory"}:
            raise ConfigError(f"Unknown mock mode: {self._mode}")

        self._sin = dict(mock_cfg.get("sinusoidal") or {})
        self._static = dict(mock_cfg.get("static") or {})
        self._trajectory_csv = mock_cfg.get("trajectory_csv")
        self._trajectory: list[TrackerPose] = []

        self._trackers = [t for t in trackers if t.get("enabled", True)]
        if not self._trackers:
            raise ConfigError("No enabled trackers in config")

        max_size = int(queue_cfg.get("max_size", 1000))
        if max_size < 1:
            raise ConfigError("queue.max_size must be >= 1")
        self._queue: queue.Queue[TrackerPose] = queue.Queue(maxsize=max_size)
        self._latest: dict[str, TrackerPose] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._initialized = False
        self._drop_count = 0
        self._seq: dict[str, int] = {str(t["serial"]): 0 for t in self._trackers}
        self._prev_quat: dict[str, Quaternion] = {}
        self._t0 = 0.0

    def initialize(self) -> None:
        if self._motion_mode == "trajectory":
            self._trajectory = _load_trajectory(self._trajectory_csv)
        self._initialized = True
        LOGGER.info(
            "MockBackend initialized mode=%s rate=%.1f Hz trackers=%d",
            self._mode,
            self._rate_hz,
            len(self._trackers),
        )

    def start(self) -> None:
        if not self._initialized:
            self.initialize()
        if self._running:
            return
        self._stop.clear()
        self._t0 = time.monotonic()
        self._running = True
        self._thread = threading.Thread(target=self._run, name="mock-tracker-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        LOGGER.info("MockBackend stopped queue_drop_count=%d", self._drop_count)

    def is_running(self) -> bool:
        return self._running

    def list_devices(self) -> list[TrackerDevice]:
        devices: list[TrackerDevice] = []
        with self._lock:
            latest = dict(self._latest)
        for tracker in self._trackers:
            serial = str(tracker["serial"])
            pose = latest.get(serial)
            devices.append(
                TrackerDevice(
                    serial=serial,
                    logical_name=str(tracker.get("logical_name", serial)),
                    tracking_valid=bool(pose.tracking_valid) if pose is not None else False,
                    source="mock",
                )
            )
        return devices

    def get_latest_pose(self, serial: Optional[str] = None) -> Optional[TrackerPose]:
        with self._lock:
            if serial is None:
                if not self._latest:
                    return None
                return next(iter(self._latest.values()))
            return self._latest.get(serial)

    def wait_for_pose(self, timeout: Optional[float] = None) -> Optional[TrackerPose]:
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def queue_drop_count(self) -> int:
        return self._drop_count

    def _run(self) -> None:
        period = 1.0 / self._rate_hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            elapsed = now - self._t0
            for index, tracker in enumerate(self._trackers):
                serial = str(tracker["serial"])
                valid = self._tracking_valid(elapsed, serial)
                pose = self._make_pose(tracker, index, elapsed, valid)
                self._publish(pose)
            next_tick += period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                self._stop.wait(sleep_s)
            else:
                next_tick = time.monotonic()

    def _tracking_valid(self, elapsed: float, serial: str) -> bool:
        if not self._dropout_enabled:
            return True
        if self._dropout_serials and serial not in self._dropout_serials:
            return True
        cycle = elapsed % self._dropout_period_s
        return cycle > self._dropout_duration_s

    def _make_pose(
        self,
        tracker: dict[str, Any],
        index: int,
        elapsed: float,
        tracking_valid: bool,
    ) -> TrackerPose:
        serial = str(tracker["serial"])
        logical_name = str(tracker.get("logical_name", serial))
        role = str(tracker.get("role") or "hand").lower()
        if role in _BENCH_ROLES:
            position, orientation = self._landmark_pose(tracker, role)
        else:
            position, orientation = self._sample_motion(index, elapsed)
        orientation = enforce_quaternion_continuity(orientation, self._prev_quat.get(serial))
        self._prev_quat[serial] = orientation
        self._seq[serial] += 1
        return TrackerPose(
            serial=serial,
            logical_name=logical_name,
            position_m=position,
            orientation=orientation,
            sensor_timestamp=None,
            host_monotonic_ns=host_monotonic_ns(),
            tracking_valid=tracking_valid,
            sequence_id=self._seq[serial],
            source="mock",
        )

    def _landmark_pose(self, tracker: dict[str, Any], role: str) -> tuple[Vec3, Quaternion]:
        default = _DEFAULT_BENCH_POS.get(role, (_STATIC_X, _STATIC_Y, _STATIC_Z))
        pos = dict(tracker.get("mock_position") or {})
        return (
            Vec3(
                float(pos.get("x", default[0])),
                float(pos.get("y", default[1])),
                float(pos.get("z", default[2])),
            ),
            IDENTITY_QUATERNION,
        )

    def _sample_motion(self, index: int, elapsed: float) -> tuple[Vec3, Quaternion]:
        phase = 0.35 * index
        if self._motion_mode == "static":
            return (
                Vec3(
                    float(self._static.get("x", _STATIC_X)),
                    float(self._static.get("y", _STATIC_Y)),
                    float(self._static.get("z", _STATIC_Z)),
                ),
                IDENTITY_QUATERNION,
            )
        if self._motion_mode == "trajectory":
            return self._sample_trajectory(elapsed)
        x0 = float(self._sin.get("x0", _STATIC_X))
        y0 = float(self._sin.get("y0", _STATIC_Y))
        z0 = float(self._sin.get("z0", _STATIC_Z))
        amp_x = float(self._sin.get("ax", 0.05))
        amp_y = float(self._sin.get("by", 0.03))
        amp_z = float(self._sin.get("cz", 0.02))
        omega = float(self._sin.get("omega", 1.0))
        omega2 = float(self._sin.get("omega2", 0.7))
        t = elapsed * self._playback_speed
        position = Vec3(
            x0 + amp_x * math.sin(omega * t + phase),
            y0 + amp_y * math.cos(omega * t + phase),
            z0 + amp_z * math.sin(omega2 * t + phase),
        )
        angle = 0.25 * math.sin(omega * t + phase)
        orientation = axis_angle_to_quaternion(Vec3(0.0, 0.0, 1.0), angle)
        return position, orientation

    def _sample_trajectory(self, elapsed: float) -> tuple[Vec3, Quaternion]:
        if not self._trajectory:
            raise ConfigError("Trajectory CSV produced no samples")
        if len(self._trajectory) == 1:
            pose = self._trajectory[0]
            return pose.position_m, pose.orientation
        t0 = self._trajectory[0].host_monotonic_ns
        t_end = self._trajectory[-1].host_monotonic_ns
        duration_ns = max(1, t_end - t0)
        query_ns = t0 + int((elapsed * self._playback_speed * _NS_PER_S) % duration_ns)
        before = self._trajectory[0]
        after = self._trajectory[-1]
        for pose in self._trajectory:
            if pose.host_monotonic_ns <= query_ns:
                before = pose
            if pose.host_monotonic_ns >= query_ns:
                after = pose
                break
        blended = interpolate_pose(before, after, query_ns)
        return blended.position_m, blended.orientation

    def _publish(self, pose: TrackerPose) -> None:
        with self._lock:
            self._latest[pose.serial] = pose
        try:
            self._queue.put_nowait(pose)
        except queue.Full:
            self._drop_count += 1
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(pose)
            except queue.Full:
                LOGGER.warning("Mock pose queue still full after drop serial=%s", pose.serial)


def _load_trajectory(path_value: Optional[str]) -> list[TrackerPose]:
    if not path_value:
        raise ConfigError("mock.trajectory_csv is required for trajectory mode")
    path = Path(path_value)
    if not path.is_file():
        raise ConfigError(f"Trajectory CSV not found: {path}")
    poses: list[TrackerPose] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader):
            timestamp_s = float(row["timestamp"])
            poses.append(
                TrackerPose(
                    serial="trajectory",
                    logical_name="trajectory",
                    position_m=Vec3(float(row["x"]), float(row["y"]), float(row["z"])),
                    orientation=normalize_quaternion(
                        Quaternion(
                            w=float(row["qw"]),
                            x=float(row["qx"]),
                            y=float(row["qy"]),
                            z=float(row["qz"]),
                        )
                    ),
                    sensor_timestamp=timestamp_s,
                    host_monotonic_ns=seconds_to_monotonic_ns(timestamp_s),
                    tracking_valid=True,
                    sequence_id=index + 1,
                    source="trajectory",
                )
            )
    if not poses:
        raise ConfigError(f"Trajectory CSV is empty: {path}")
    LOGGER.info("Loaded %d trajectory samples from %s", len(poses), path)
    return poses
