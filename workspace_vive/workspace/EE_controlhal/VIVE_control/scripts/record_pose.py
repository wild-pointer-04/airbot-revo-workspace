#!/usr/bin/env python3
"""Record raw tracker poses to CSV."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_tracking.exceptions import BackendNotAvailableError, ViveTrackingError
from vive_tracking.recorder import PoseRecorder
from vive_tracking.tracker_manager import TrackerManager, load_config, setup_logging


def main() -> int:
    parser = argparse.ArgumentParser(description="Record VIVE Tracker poses to CSV")
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "tracker.yaml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()

    config = load_config(args.config)
    setup_logging(str((config.get("logging") or {}).get("level", "INFO")))

    try:
        manager = TrackerManager.from_config(args.config)
        manager.start()
    except BackendNotAvailableError as exc:
        print(str(exc).strip())
        return 1
    except ViveTrackingError as exc:
        print(f"Failed to start tracker backend: {exc}")
        return 1

    recorder = PoseRecorder(args.output)
    t0 = time.monotonic()
    try:
        recorder.open()
        while (time.monotonic() - t0) < args.duration:
            pose = manager.wait_for_pose(timeout=0.5)
            if pose is None:
                continue
            recorder.write(pose)
    except KeyboardInterrupt:
        print("\nInterrupted, flushing recorder.")
    finally:
        elapsed = recorder.elapsed_s
        frames = recorder.count
        rate = recorder.average_rate_hz
        recorder.close()
        manager.stop()

    print(f"output: {args.output}")
    print(f"frames: {frames}")
    print(f"duration_s: {elapsed:.3f}")
    print(f"average_rate_hz: {rate:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
