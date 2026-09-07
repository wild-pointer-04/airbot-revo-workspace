"""VIVE Tracker 6DoF acquisition. Quaternion order is [w, x, y, z]."""

from vive_tracking.exceptions import (
    BackendInitError,
    BackendNotAvailableError,
    ConfigError,
    DeviceNotFoundError,
    ViveTrackingError,
)
from vive_tracking.tracker_manager import TrackerManager, load_config, setup_logging
from vive_tracking.types import Pose6DoF, Quaternion, TrackerDevice, TrackerPose, Vec3

__all__ = [
    "BackendInitError",
    "BackendNotAvailableError",
    "ConfigError",
    "DeviceNotFoundError",
    "Pose6DoF",
    "Quaternion",
    "TrackerDevice",
    "TrackerManager",
    "TrackerPose",
    "Vec3",
    "ViveTrackingError",
    "load_config",
    "setup_logging",
]
