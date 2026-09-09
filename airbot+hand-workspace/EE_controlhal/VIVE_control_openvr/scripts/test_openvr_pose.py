#!/usr/bin/env python3
"""Print one role's OpenVR pose at a low display rate (default 10 Hz)."""

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
from vive_openvr.tracker_manager import RoleNotFoundError, TrackerManager


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "tracker.yaml")
    parser.add_argument("--role", default="right_wrist")
    parser.add_argument("--hz", type=float, default=10.0, help="Display refresh rate")
    parser.add_argument("--duration", type=float, default=0.0, help="0 = until Ctrl-C")
    parser.add_argument("--verbose", action="store_true")
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

    if args.role not in manager.list_expected():
        print(
            f"role {args.role!r} not in enabled trackers: {sorted(manager.list_expected())}",
            file=sys.stderr,
        )
        return 2

    try:
        manager.start()
    except OpenVRInitError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    period = 1.0 / max(0.1, float(args.hz))
    t0 = time.monotonic()
    n = 0
    last_ts = 0.0
    try:
        while True:
            if args.duration > 0.0 and (time.monotonic() - t0) >= args.duration:
                break
            try:
                pose = manager.get_latest_pose(args.role)
            except RoleNotFoundError as exc:
                print(str(exc), file=sys.stderr)
                return 2

            now = time.monotonic()
            if pose is None:
                print(f"[{now - t0:7.2f}s] waiting for {args.role} ...")
            else:
                dt = pose.timestamp - last_ts if last_ts > 0.0 else 0.0
                rate = (1.0 / dt) if dt > 1e-6 else 0.0
                last_ts = pose.timestamp
                n += 1
                p = pose.position
                q = pose.quaternion
                print(
                    f"[{now - t0:7.2f}s] rate≈{rate:5.1f} Hz  "
                    f"status={pose.status.value}  valid={pose.pose_valid}  "
                    f"connected={pose.device_connected}"
                )
                print(
                    f"  serial={pose.serial}  index={pose.device_index}  "
                    f"tracking_result={pose.tracking_result}"
                )
                print(f"  position xyz  {p[0]:+.4f}  {p[1]:+.4f}  {p[2]:+.4f}")
                print(
                    f"  quaternion xyzw  {q[0]:+.5f}  {q[1]:+.5f}  "
                    f"{q[2]:+.5f}  {q[3]:+.5f}"
                )
                print()
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        manager.stop()
        print(f"frames displayed: {n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
