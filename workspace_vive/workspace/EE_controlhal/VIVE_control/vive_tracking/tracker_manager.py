"""Public facade for pose acquisition. Upper layers never import pysurvive."""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import Any, Optional, Union

import yaml

from vive_tracking.backend import TrackerBackend
from vive_tracking.exceptions import ConfigError, DeviceNotFoundError
from vive_tracking.timestamp import mark_stale_if_old
from vive_tracking.types import TrackerDevice, TrackerPose

LOGGER = logging.getLogger(__name__)

PoseCallback = Callable[[TrackerPose], None]


class TrackerManager:
    def __init__(self, backend: TrackerBackend, config: dict[str, Any]) -> None:
        self._backend = backend
        self._config = config
        self._trackers = [t for t in config.get("trackers", []) if t.get("enabled", True)]
        if not self._trackers:
            raise ConfigError("No enabled trackers in config")
        self._serial_by_name = {
            str(t["logical_name"]): str(t["serial"]) for t in self._trackers
        }
        self._name_by_serial = {
            str(t["serial"]): str(t["logical_name"]) for t in self._trackers
        }
        self._lock = threading.Lock()
        self._last_valid: dict[str, TrackerPose] = {}
        self._callbacks: list[PoseCallback] = []
        self._dispatch_stop = threading.Event()
        self._dispatch_thread: Optional[threading.Thread] = None
        self._started = False
        bench_cfg = dict(config.get("bench") or {})
        self._stale_s = float(bench_cfg.get("stale_s", 0.25))

    @classmethod
    def from_config(cls, path: Union[str, Path]) -> TrackerManager:
        config = load_config(path)
        backend = create_backend(config)
        return cls(backend, config)

    def start(self) -> None:
        self._backend.initialize()
        self._backend.start()
        self._started = True
        self._maybe_start_dispatch()
        LOGGER.info("TrackerManager started backend=%s", type(self._backend).__name__)

    def stop(self) -> None:
        self._dispatch_stop.set()
        if self._dispatch_thread is not None:
            self._dispatch_thread.join(timeout=2.0)
            self._dispatch_thread = None
        self._backend.stop()
        self._started = False
        LOGGER.info("TrackerManager stopped")

    def is_running(self) -> bool:
        return self._backend.is_running()

    def add_callback(self, callback: PoseCallback) -> None:
        self._callbacks.append(callback)
        if self._started:
            self._maybe_start_dispatch()

    def list_devices(self) -> list[TrackerDevice]:
        return self._backend.list_devices()

    def get_pose(self, logical_name: str) -> Optional[TrackerPose]:
        serial = self._serial_by_name.get(logical_name)
        if serial is None:
            raise DeviceNotFoundError(f"Unknown logical_name: {logical_name}")
        pose = self._backend.get_latest_pose(serial)
        if pose is None:
            return None
        return self._annotate(pose)

    def get_all_poses(self) -> dict[str, TrackerPose]:
        out: dict[str, TrackerPose] = {}
        for logical_name in self._serial_by_name:
            pose = self.get_pose(logical_name)
            if pose is not None:
                out[logical_name] = pose
        return out

    def wait_for_pose(self, timeout: Optional[float] = None) -> Optional[TrackerPose]:
        pose = self._backend.wait_for_pose(timeout)
        if pose is None:
            return None
        return self._annotate(pose)

    def last_valid_pose(self, logical_name: str) -> Optional[TrackerPose]:
        serial = self._serial_by_name.get(logical_name)
        if serial is None:
            raise DeviceNotFoundError(f"Unknown logical_name: {logical_name}")
        with self._lock:
            return self._last_valid.get(serial)

    def queue_drop_count(self) -> int:
        return self._backend.queue_drop_count()

    def __enter__(self) -> TrackerManager:
        self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.stop()

    def _annotate(self, pose: TrackerPose) -> TrackerPose:
        logical_name = self._name_by_serial.get(pose.serial, pose.logical_name)
        annotated = replace(pose, logical_name=logical_name)
        annotated = mark_stale_if_old(annotated, self._stale_s)
        if annotated.tracking_valid:
            with self._lock:
                self._last_valid[annotated.serial] = annotated
        return annotated

    def _maybe_start_dispatch(self) -> None:
        if not self._callbacks or self._dispatch_thread is not None:
            return
        self._dispatch_stop.clear()
        self._dispatch_thread = threading.Thread(
            target=self._dispatch_loop,
            name="tracker-callback-dispatch",
            daemon=True,
        )
        self._dispatch_thread.start()

    def _dispatch_loop(self) -> None:
        """Poll latest-pose cache. Never drain wait_for_pose()."""
        last_seq: dict[str, int] = {}
        while not self._dispatch_stop.is_set():
            for logical_name in self._serial_by_name:
                try:
                    pose = self.get_pose(logical_name)
                except DeviceNotFoundError:
                    continue
                if pose is None:
                    continue
                if last_seq.get(pose.serial) == pose.sequence_id:
                    continue
                last_seq[pose.serial] = pose.sequence_id
                for callback in list(self._callbacks):
                    try:
                        callback(pose)
                    except Exception:
                        LOGGER.exception("Pose callback failed serial=%s", pose.serial)
            self._dispatch_stop.wait(0.001)


def load_config(path: Union[str, Path]) -> dict[str, Any]:
    config_path = Path(path)
    if not config_path.is_file():
        raise ConfigError(f"Config not found: {config_path}")
    with config_path.open() as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ConfigError(f"Config must be a mapping: {config_path}")
    return data


def create_backend(config: dict[str, Any]) -> TrackerBackend:
    kind = str(config.get("backend", "mock")).lower()
    trackers = list(config.get("trackers") or [])
    if kind == "mock":
        from vive_tracking.mock_backend import MockBackend

        return MockBackend(config, trackers)
    if kind == "libsurvive":
        from vive_tracking.libsurvive_backend import LibsurviveBackend

        return LibsurviveBackend(config, trackers)
    raise ConfigError(f"Unknown backend: {kind}")


def setup_logging(level: str = "INFO") -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )
