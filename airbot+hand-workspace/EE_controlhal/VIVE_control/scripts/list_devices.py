#!/usr/bin/env python3
"""List configured / discovered VIVE trackers."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_tracking.exceptions import BackendNotAvailableError, ViveTrackingError
from vive_tracking.tracker_manager import TrackerManager, load_config, setup_logging


def main() -> int:
    parser = argparse.ArgumentParser(description="List VIVE Tracker devices")
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "tracker.yaml")
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Seconds to wait for a first pose; libsurvive needs to decode OOTX "
        "and run its global solve before any pose appears (~5-10 s from cold).",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    setup_logging(str((config.get("logging") or {}).get("level", "INFO")))
    backend_name = str(config.get("backend", "mock"))
    print(f"Backend: {backend_name}")
    print()

    try:
        manager = TrackerManager.from_config(args.config)
        manager.start()
    except BackendNotAvailableError as exc:
        print(str(exc).strip())
        return 1
    except ViveTrackingError as exc:
        print(f"Failed to start tracker backend: {exc}")
        return 1

    try:
        print(f"Waiting up to {args.timeout:.0f}s for a first pose...")
        got_pose = manager.wait_for_pose(timeout=args.timeout) is not None
        devices = manager.list_devices()
    finally:
        manager.stop()

    print()
    if not devices:
        print("No VIVE Tracker detected.")
        return 0
    if not got_pose:
        print("Trackers enumerated but no pose solved yet — check base station line of sight.")
        print()

    print("Detected Trackers:")
    print()
    for device in devices:
        tracking = "VALID" if device.tracking_valid else "INVALID"
        print("logical_name:")
        print(device.logical_name)
        print()
        print("serial:")
        print(device.serial)
        print()
        print("tracking:")
        print(tracking)
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
