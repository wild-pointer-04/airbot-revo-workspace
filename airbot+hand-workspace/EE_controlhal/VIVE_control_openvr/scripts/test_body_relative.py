#!/usr/bin/env python3
"""Live body vs hand world poses and body-relative hand.

Case A — walk with body+hand together: relative hand should stay nearly fixed.
Case B — body still, move hand: relative hand should change.

Optional --axis-test: body still, move hand forward / left / up and read Δxyz.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vive_openvr.openvr_backend import OpenVRInitError
from vive_openvr.relative_pose import BodyRelativePose
from vive_openvr.tracker_manager import TrackerManager


def _fmt_xyz(p: np.ndarray) -> str:
    return f"{p[0]:+.4f}  {p[1]:+.4f}  {p[2]:+.4f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "tracker.yaml")
    parser.add_argument("--body", default="body")
    parser.add_argument("--hand", default="right_wrist")
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--axis-test",
        action="store_true",
        help="Prompted axis check: record relative Δ while body stays still",
    )
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

    try:
        manager.start()
    except OpenVRInitError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    br = BodyRelativePose()
    period = 1.0 / max(0.1, float(args.hz))
    t0 = time.monotonic()
    baseline = None

    if args.axis_test:
        print(
            "Axis test:\n"
            "  1) Keep body tracker still\n"
            "  2) Move hand forward / left / up relative to body\n"
            "  3) Watch body-relative Δxyz (printed vs first valid sample)\n"
            "Do NOT invent yaw+180 here — record measured deltas for T_map later.\n"
        )

    try:
        while True:
            if args.duration > 0.0 and (time.monotonic() - t0) >= args.duration:
                break
            body = manager.get_latest_pose(args.body)
            hand = manager.get_latest_pose(args.hand)
            rel = br.compute(body, hand)
            now = time.monotonic() - t0
            if body is None or hand is None or rel is None:
                print(f"[{now:7.2f}s] waiting for body+hand ...")
                time.sleep(period)
                continue
            print(f"[{now:7.2f}s] body_ok={rel.body_valid} hand_ok={rel.hand_valid}")
            print(f"  body world xyz   {_fmt_xyz(body.position)}")
            print(f"  hand world xyz   {_fmt_xyz(hand.position)}")
            print(f"  body→hand xyz    {_fmt_xyz(rel.position)}")
            print(
                f"  body→hand quat   "
                f"{rel.quaternion[0]:+.4f} {rel.quaternion[1]:+.4f} "
                f"{rel.quaternion[2]:+.4f} {rel.quaternion[3]:+.4f}"
            )
            if args.axis_test and rel.body_valid and rel.hand_valid:
                if baseline is None:
                    baseline = rel.position.copy()
                    print("  [axis-test] baseline relative position captured")
                else:
                    delta = rel.position - baseline
                    print(f"  [axis-test] Δxyz vs baseline  {_fmt_xyz(delta)}")
            print()
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        manager.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
