"""Thread-safe multi-source latest-value buffer with max-age windows."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Optional


@dataclass
class SourceEntry:
    timestamp: float
    data: Any


class SyncBuffer:
    """Sources push asynchronously; the recorder pulls aligned snapshots."""

    def __init__(
        self,
        source_names: list[str],
        max_age_s: float = 0.1,
        max_age_by_source: Optional[dict[str, float]] = None,
    ):
        self._sources = {name: None for name in source_names}
        self._lock = threading.Lock()
        self._max_age = max_age_s
        overrides = max_age_by_source or {}
        unknown = set(overrides) - set(source_names)
        if unknown:
            raise ValueError(f"max_age_by_source has unknown sources: {sorted(unknown)}")
        self._max_age_by_source = dict(overrides)

    def _age_limit(self, source: str) -> float:
        return self._max_age_by_source.get(source, self._max_age)

    def update(self, source: str, data: Any, timestamp: Optional[float] = None):
        ts = timestamp if timestamp is not None else time.monotonic()
        with self._lock:
            self._sources[source] = SourceEntry(timestamp=ts, data=data)

    def get_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        snapshot = {}
        with self._lock:
            for name, entry in self._sources.items():
                if entry is None:
                    snapshot[name] = None
                elif (now - entry.timestamp) > self._age_limit(name):
                    snapshot[name] = None
                else:
                    snapshot[name] = entry.data
        return snapshot
