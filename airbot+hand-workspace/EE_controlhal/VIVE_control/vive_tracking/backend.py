"""Tracker backend interface. TrackerManager must not import pysurvive."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from vive_tracking.types import TrackerDevice, TrackerPose


class TrackerBackend(ABC):
    """Producer of TrackerPose samples.

    Each backend owns a reader thread. Console I/O, CSV, and diagnostics
    must never run on that thread.
    """

    @abstractmethod
    def initialize(self) -> None:
        ...

    @abstractmethod
    def start(self) -> None:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...

    @abstractmethod
    def is_running(self) -> bool:
        ...

    @abstractmethod
    def list_devices(self) -> list[TrackerDevice]:
        ...

    @abstractmethod
    def get_latest_pose(self, serial: Optional[str] = None) -> Optional[TrackerPose]:
        ...

    @abstractmethod
    def wait_for_pose(self, timeout: Optional[float] = None) -> Optional[TrackerPose]:
        ...

    @abstractmethod
    def queue_drop_count(self) -> int:
        ...
