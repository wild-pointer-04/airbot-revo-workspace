#!/usr/bin/env python3
"""Collect pose diagnostics and print PASS / WARNING / FAIL."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_tracking.diagnostics import PoseDiagnostics
from vive_tracking.exceptions import BackendNotAvailableError, ViveTrackingError
from vive_tracking.tracker_manager import TrackerManager, load_config, setup_logging


def main() -> int:
    parser = argparse.ArgumentParser(description="Run VIVE Tracker diagnostics")
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "tracker.yaml")
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()

    config = load_config(args.config)
    setup_logging(str((config.get("logging") or {}).get("level", "INFO")))
    thresholds = dict(config.get("diagnostics") or {})
    diag = PoseDiagnostics(thresholds)

    try:
        manager = TrackerManager.from_config(args.config)
        manager.start()
    except BackendNotAvailableError as exc:
        print(str(exc).strip())
        return 1
    except ViveTrackingError as exc:
        print(f"Failed to start tracker backend: {exc}")
        return 1

    t0 = time.monotonic()
    try:
        while (time.monotonic() - t0) < args.duration:
            pose = manager.wait_for_pose(timeout=0.5)
            if pose is None:
                continue
            diag.add(pose)
    except KeyboardInterrupt:
        print("\nInterrupted, summarizing collected samples.")
    finally:
        diag.set_queue_drop_count(manager.queue_drop_count())
        manager.stop()

    report = diag.summarize()
    print(report.format())
    if report.verdict == "FAIL":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
