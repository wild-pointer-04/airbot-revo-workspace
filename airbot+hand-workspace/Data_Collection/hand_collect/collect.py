#!/usr/bin/env python3
"""Record pedal-gated Revo2 hand-feedback episodes into a LeRobot dataset."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from recorder import Revo2HandRecorder, load_config


def main() -> int:
    directory = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Revo2 physical-hand feedback collector")
    parser.add_argument("--config", type=Path, default=directory / "config" / "revo2_right.yaml")
    parser.add_argument(
        "--seconds",
        type=float,
        default=None,
        help="Optional collector lifetime in seconds; omit for continuous pedal-gated recording.",
    )
    parser.add_argument("--data-root", type=Path, default=None)
    args = parser.parse_args()
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive")

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    recorder = Revo2HandRecorder(load_config(args.config), data_root=args.data_root)
    recorder.run(args.seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
