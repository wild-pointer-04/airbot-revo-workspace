"""Role ↔ permanent serial mapping over OpenVRBackend."""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any, Optional, Union

import yaml

from vive_openvr.openvr_backend import OpenVRBackend
from vive_openvr.pose_filter import PoseFilter
from vive_openvr.tracker_extrinsics import (
    apply_pose_extrinsics,
    extrinsics_for_role,
    load_role_extrinsics,
)
from vive_openvr.types import DiscoveredTracker, TrackerPose, TrackingStatus

LOGGER = logging.getLogger(__name__)


class ConfigError(ValueError):
    pass


class RoleNotFoundError(KeyError):
    pass


def load_config(path: Union[str, Path]) -> dict[str, Any]:
    path = Path(path)
    with path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ConfigError(f"config {path} must be a mapping")
    return data


def enabled_roles(config: dict[str, Any]) -> dict[str, str]:
    """Return {role: serial} for enabled trackers with a non-empty serial."""
    trackers = config.get("trackers") or {}
    if not isinstance(trackers, dict):
        raise ConfigError("trackers must be a mapping of role -> {serial, enabled}")
    out: dict[str, str] = {}
    for role, row in trackers.items():
        if not isinstance(row, dict):
            raise ConfigError(f"trackers.{role} must be a mapping")
        if not bool(row.get("enabled", True)):
            continue
        serial = str(row.get("serial") or "").strip()
        if not serial:
            raise ConfigError(
                f"trackers.{role} is enabled but serial is empty"
            )
        out[str(role)] = serial
    if not out:
        raise ConfigError("no enabled trackers in config")
    return out


class TrackerManager:
    """Maps configured roles to OpenVR poses by permanent LHR- serial."""

    def __init__(self, config: dict[str, Any], backend: Optional[OpenVRBackend] = None) -> None:
        self._config = config
        self._role_to_serial = enabled_roles(config)
        self._serial_to_role = {s: r for r, s in self._role_to_serial.items()}
        tracking = dict(config.get("tracking") or {})
        self._stale_timeout_s = float(tracking.get("stale_timeout_ms", 100)) / 1000.0
        self._backend = backend or OpenVRBackend(
            polling_rate_hz=float(tracking.get("polling_rate_hz", 120)),
            universe=str(tracking.get("universe", "standing")),
            role_by_serial=dict(self._serial_to_role),
        )
        self._extrinsics = load_role_extrinsics(config)
        filt = dict(config.get("filter") or {})
        self._filter = PoseFilter(
            enabled=bool(filt.get("enabled", False)),
            position_alpha=float(filt.get("position_alpha", 0.25)),
            orientation_alpha=float(
                filt.get("orientation_alpha", filt.get("rotation_alpha", 0.25))
            ),
            filter_orientation=bool(filt.get("filter_orientation", True)),
        )
        self._started = False

    @classmethod
    def from_config(cls, path: Union[str, Path]) -> "TrackerManager":
        return cls(load_config(path))

    @property
    def backend(self) -> OpenVRBackend:
        return self._backend

    def start(self) -> None:
        self._backend.start()
        self._started = True
        LOGGER.info(
            "TrackerManager started roles=%s filter=%s",
            sorted(self._role_to_serial),
            self._filter.enabled,
        )

    def stop(self) -> None:
        self._backend.stop()
        self._filter.reset()
        self._started = False
        LOGGER.info("TrackerManager stopped")

    def __enter__(self) -> "TrackerManager":
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    def list_expected(self) -> dict[str, str]:
        return dict(self._role_to_serial)

    def list_discovered(self) -> list[DiscoveredTracker]:
        discovered = self._backend.list_discovered()
        return [
            DiscoveredTracker(
                serial=d.serial,
                device_index=d.device_index,
                connected=d.connected,
                pose_valid=d.pose_valid,
                role=self._serial_to_role.get(d.serial, ""),
            )
            for d in discovered
        ]

    def missing_expected(self) -> list[tuple[str, str]]:
        """Return [(role, serial), ...] configured but not currently discovered."""
        found = {d.serial for d in self.list_discovered()}
        return [
            (role, serial)
            for role, serial in self._role_to_serial.items()
            if serial not in found
        ]

    def get_latest_pose(self, role: str) -> Optional[TrackerPose]:
        serial = self._role_to_serial.get(role)
        if serial is None:
            raise RoleNotFoundError(f"unknown or disabled role: {role!r}")
        pose = self._backend.get_latest_pose(serial)
        if pose is None:
            return None
        age = time.monotonic() - float(pose.timestamp)
        if age > self._stale_timeout_s and pose.status == TrackingStatus.OK:
            stale = pose.copy()
            stale.status = TrackingStatus.STALE
            stale.pose_valid = False
            stale.role = role
            pose = stale
        elif pose.role != role:
            annotated = pose.copy()
            annotated.role = role
            pose = annotated
        T_ext = extrinsics_for_role(self._extrinsics, role)
        pose = apply_pose_extrinsics(pose, T_ext)
        return self._filter.update(role, pose)

    def get_all_poses(self) -> dict[str, TrackerPose]:
        out: dict[str, TrackerPose] = {}
        for role in self._role_to_serial:
            pose = self.get_latest_pose(role)
            if pose is not None:
                out[role] = pose
        return out
