#!/usr/bin/env python3
"""Static tracking quality: leave the tracker still, measure jitter."""

from __future__ import annotations

import argparse
import csv
import logging
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vive_openvr.openvr_backend import OpenVRInitError
from vive_openvr.pose_utils import enforce_quaternion_continuity, normalize_quaternion
from vive_openvr.tracker_manager import RoleNotFoundError, TrackerManager


def _quat_angle(a: np.ndarray, b: np.ndarray) -> float:
    a = normalize_quaternion(a)
    b = normalize_quaternion(b)
    dot = abs(float(np.dot(a, b)))
    dot = min(1.0, max(0.0, dot))
    return 2.0 * float(math.acos(dot))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "tracker.yaml")
    parser.add_argument("--role", default="right_wrist")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=120.0, help="Sample rate")
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Output CSV (default logs/tracking_quality_TIMESTAMP.csv)",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    print("Keep the tracker completely still on a desk for the full duration.")
    try:
        manager = TrackerManager.from_config(args.config)
    except Exception as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    if args.role not in manager.list_expected():
        print(f"role {args.role!r} not enabled", file=sys.stderr)
        return 2

    try:
        manager.start()
    except OpenVRInitError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    period = 1.0 / max(1.0, float(args.hz))
    rows: list[dict] = []
    prev_q: np.ndarray | None = None
    t0 = time.monotonic()
    try:
        while (time.monotonic() - t0) < float(args.duration):
            try:
                pose = manager.get_latest_pose(args.role)
            except RoleNotFoundError as exc:
                print(str(exc), file=sys.stderr)
                return 2
            now = time.monotonic()
            if pose is None:
                time.sleep(period)
                continue
            q = normalize_quaternion(pose.quaternion)
            q = enforce_quaternion_continuity(q, prev_q)
            prev_q = q
            rows.append(
                {
                    "t": now - t0,
                    "timestamp": pose.timestamp,
                    "x": float(pose.position[0]),
                    "y": float(pose.position[1]),
                    "z": float(pose.position[2]),
                    "qx": float(q[0]),
                    "qy": float(q[1]),
                    "qz": float(q[2]),
                    "qw": float(q[3]),
                    "pose_valid": int(bool(pose.pose_valid)),
                    "connected": int(bool(pose.device_connected)),
                    "status": pose.status.value,
                }
            )
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nInterrupted early.")
    finally:
        manager.stop()

    if not rows:
        print("no samples collected", file=sys.stderr)
        return 1

    out = args.csv
    if out is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = ROOT / "logs" / f"tracking_quality_{stamp}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    pos = np.array([[r["x"], r["y"], r["z"]] for r in rows], dtype=np.float64)
    quats = np.array([[r["qx"], r["qy"], r["qz"], r["qw"]] for r in rows], dtype=np.float64)
    valid = np.array([r["pose_valid"] for r in rows], dtype=bool)
    stale = np.array([r["status"] == "STALE" for r in rows], dtype=bool)
    t = np.array([r["t"] for r in rows], dtype=np.float64)
    dt = np.diff(t)
    eff_hz = float(1.0 / np.median(dt)) if len(dt) else 0.0

    jumps_pos = np.linalg.norm(np.diff(pos, axis=0), axis=1) if len(pos) > 1 else np.array([0.0])
    jumps_ang = (
        np.array([_quat_angle(quats[i], quats[i + 1]) for i in range(len(quats) - 1)])
        if len(quats) > 1
        else np.array([0.0])
    )
    # Angular std vs mean quaternion (geodesic to first sample as proxy).
    ang_from_mean = np.array([_quat_angle(quats[0], q) for q in quats])

    print()
    print(f"role: {args.role}")
    print(f"sample count: {len(rows)}")
    print(f"effective Hz: {eff_hz:.1f}")
    print(f"duration s: {t[-1] - t[0]:.2f}")
    print(f"position mean xyz: {pos.mean(axis=0)}")
    print(f"position std  xyz: {pos.std(axis=0)}")
    print(f"position peak-to-peak xyz: {pos.max(axis=0) - pos.min(axis=0)}")
    print(f"rotation angular std (rad vs first): {float(ang_from_mean.std()):.6f}")
    print(f"invalid frame count: {int((~valid).sum())}")
    print(f"invalid rate: {float((~valid).mean()):.4f}")
    print(f"stale frame count: {int(stale.sum())}")
    print(f"largest position jump m: {float(jumps_pos.max()):.6f}")
    print(f"largest rotation jump rad: {float(jumps_ang.max()):.6f}")
    print(f"csv: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
