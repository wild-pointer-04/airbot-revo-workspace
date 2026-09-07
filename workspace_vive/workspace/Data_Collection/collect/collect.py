#!/usr/bin/env python3
"""Unique entry for AIRBOT multimodal collection.

  python3 collect/collect.py --task pick_cup
  python3 collect/collect.py --mock --seconds 1 --task mock_smoke
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

COLLECT_DIR = Path(__file__).resolve().parent
if str(COLLECT_DIR) not in sys.path:
    sys.path.insert(0, str(COLLECT_DIR))

from collector.paths import ensure_imports

ensure_imports()
from collector.collector import Collector, load_collect_config


def main(argv: list[str] | None = None) -> int:
    default_config = COLLECT_DIR / "config" / "collect.yaml"
    parser = argparse.ArgumentParser(description="AIRBOT multimodal collector")
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--task", type=str, default=None, help="Task id (also LeRobot task name)")
    parser.add_argument("--mock", action="store_true", help="No hardware: mock cams + arm state")
    parser.add_argument(
        "--seconds",
        type=float,
        default=None,
        help="Non-interactive: record this many seconds then save and exit",
    )
    parser.add_argument("--data-root", type=Path, default=None)
    ns, extra = parser.parse_known_args(argv)
    if "--ros-args" in extra:
        extra = extra[: extra.index("--ros-args")]

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    config = load_collect_config(ns.config)
    if ns.task:
        config["task"] = ns.task
        ds = dict(config.get("dataset") or {})
        ds["repo_id"] = ns.task
        config["dataset"] = ds
    if ns.mock:
        config["mock"] = True
        config["state_source"] = "local"
    if ns.data_root:
        config["data_root"] = str(ns.data_root)

    collector = Collector(config, mock=bool(config.get("mock")))
    collector.start_session()
    collector.run(seconds=ns.seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
