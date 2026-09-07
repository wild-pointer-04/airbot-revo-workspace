"""Host vs sensor time.

Sensor Capture / Sensor Time
    Comes from the tracker / libsurvive pose timecode when the backend
    exposes it. Stored as sensor_timestamp. If unavailable, it is None.
    This module never invents a sensor clock.

Host Receive Time
    time.monotonic_ns() sampled in this process immediately after a pose
    is obtained. Stored as host_monotonic_ns. Monotonic, not NTP/UTC.

Do not use time.time() as the primary sync source.
"""

from __future__ import annotations

import time
from dataclasses import replace
from typing import Optional

from vive_tracking.types import TrackerPose


def host_monotonic_ns() -> int:
    """Host receive timestamp in nanoseconds (monotonic clock)."""
    return time.monotonic_ns()


def monotonic_ns_to_seconds(ns: int) -> float:
    return ns / 1_000_000_000.0


def seconds_to_monotonic_ns(seconds: float) -> int:
    return int(seconds * 1_000_000_000.0)


def mark_stale_if_old(
    pose: TrackerPose,
    stale_s: float,
    now_ns: Optional[int] = None,
) -> TrackerPose:
    """Set tracking_valid=False when the sample is older than stale_s.

    Does not replace the numbers with last_valid. Callers must treat
    tracking_valid=False as a missing observation.
    """
    if stale_s <= 0.0 or not pose.tracking_valid:
        return pose
    now = host_monotonic_ns() if now_ns is None else now_ns
    age_s = monotonic_ns_to_seconds(now - pose.host_monotonic_ns)
    if age_s > stale_s:
        return replace(pose, tracking_valid=False)
    return pose
