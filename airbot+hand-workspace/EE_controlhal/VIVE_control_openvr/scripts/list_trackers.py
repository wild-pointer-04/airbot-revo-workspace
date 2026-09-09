#!/usr/bin/env python3
"""List OpenVR GenericTrackers and match them to configured roles."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vive_openvr.openvr_backend import OpenVRInitError
from vive_openvr.tracker_manager import TrackerManager


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config" / "tracker.yaml",
    )
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="Seconds to wait after start before reading poses",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    try:
        manager = TrackerManager.from_config(args.config)
    except Exception as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    try:
        manager.start()
    except OpenVRInitError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    try:
        time.sleep(max(0.0, float(args.settle)))
        discovered = manager.list_discovered()
        expected = manager.list_expected()
        by_serial = {d.serial: d for d in discovered}

        print("Detected Generic Trackers:\n")
        if not discovered:
            print("(none)")
        for d in discovered:
            print(f"role: {d.role or '(unmapped)'}")
            print(f"serial: {d.serial}")
            print(f"index: {d.device_index}")
            print(f"connected: {str(d.connected).lower()}")
            print(f"valid: {str(d.pose_valid).lower()}")
            print()

        missing = []
        for role, serial in expected.items():
            if serial not in by_serial:
                print(f"Expected tracker {serial} not found  (role={role})")
                missing.append((role, serial))
            else:
                d = by_serial[serial]
                pose = manager.get_latest_pose(role)
                status = pose.status.value if pose is not None else "NO_SAMPLE"
                print(
                    f"Mapped  role={role}  serial={serial}  "
                    f"connected={d.connected}  valid={d.pose_valid}  status={status}"
                )

        if missing:
            return 3
        return 0
    finally:
        manager.stop()


if __name__ == "__main__":
    raise SystemExit(main())
