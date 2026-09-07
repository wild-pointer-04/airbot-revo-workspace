#!/usr/bin/env python3
"""Print the latest pose at a low console rate. Reader thread stays at full rate."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vive_tracking.exceptions import BackendNotAvailableError, ViveTrackingError
from vive_tracking.tracker_manager import TrackerManager, load_config, setup_logging


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor VIVE Tracker pose")
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "tracker.yaml")
    parser.add_argument("--logical-name", default="hand_controller")
    parser.add_argument("--duration", type=float, default=None, help="Seconds; omit to run until Ctrl+C")
    args = parser.parse_args()

    config = load_config(args.config)
    setup_logging("WARNING")
    console_hz = float((config.get("logging") or {}).get("console_rate_hz", 10))
    period = 1.0 / max(console_hz, 0.1)

    try:
        manager = TrackerManager.from_config(args.config)
        manager.start()
    except BackendNotAvailableError as exc:
        print(str(exc).strip())
        return 1
    except ViveTrackingError as exc:
        print(f"Failed to start tracker backend: {exc}")
        return 1

    prev_seq: int | None = None
    prev_ns: int | None = None
    rate = 0.0
    t0 = time.monotonic()
    try:
        while True:
            if args.duration is not None and (time.monotonic() - t0) >= args.duration:
                break
            pose = manager.get_pose(args.logical_name)
            if pose is None:
                time.sleep(period)
                continue
            if prev_seq is not None and prev_ns is not None:
                dt_s = (pose.host_monotonic_ns - prev_ns) / 1_000_000_000.0
                dseq = pose.sequence_id - prev_seq
                if dt_s > 0.0 and dseq >= 0:
                    rate = dseq / dt_s
            prev_seq = pose.sequence_id
            prev_ns = pose.host_monotonic_ns
            tracking = "VALID" if pose.tracking_valid else "INVALID"
            sensor = "None" if pose.sensor_timestamp is None else f"{pose.sensor_timestamp}"
            print("\033[2J\033[H", end="")
            print("Tracker:")
            print(pose.logical_name)
            print()
            print("Serial:")
            print(pose.serial)
            print()
            print("Position [m]:")
            print(f"x = {pose.position_m.x:.3f}")
            print(f"y = {pose.position_m.y:.3f}")
            print(f"z = {pose.position_m.z:.3f}")
            print()
            print("Quaternion [wxyz]:")
            print(f"w = {pose.orientation.w:.3f}")
            print(f"x = {pose.orientation.x:.3f}")
            print(f"y = {pose.orientation.y:.3f}")
            print(f"z = {pose.orientation.z:.3f}")
            print()
            print("Host monotonic:")
            print(pose.host_monotonic_ns)
            print()
            print("Sensor timestamp:")
            print(sensor)
            print()
            print("Tracking:")
            print(tracking)
            print()
            print("Update rate:")
            print(f"{rate:.1f} Hz")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        manager.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
