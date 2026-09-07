#!/usr/bin/env python3
"""VIVE two-tracker teleop using the PICO middle_interface pipeline."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent
MIDDLE = ROOT.parent / "middle_interface"
COMMAND = ROOT.parent / "Robot_control_interface"
for path in (str(ROOT), str(MIDDLE), str(COMMAND)):
    if path not in sys.path:
        sys.path.insert(0, path)

from packet_source import VivePacketSource
from processing.pipeline import TeleopPipeline
from processing.safety import TeleopState
from robot import create_adapter

logger = logging.getLogger("vive_teleop")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="VIVE tracker teleop for AIRBOT Play")
    parser.add_argument("--config", default=str(ROOT / "config" / "teleop.yaml"))
    parser.add_argument("--mock", action="store_true", help="Mock trackers; dummy arm unless config says otherwise")
    parser.add_argument(
        "--dummy-arm",
        action="store_true",
        help="Force DummyRobotAdapter even if teleop.yaml asks for command_layer",
    )
    parser.add_argument(
        "--record",
        nargs="?",
        const="auto",
        default=None,
        help="Write mapping JSONL (VIVE in + arm EE). Default path: data/mapping_YYYYmmdd-HHMMSS.jsonl",
    )
    return parser.parse_args()


def load_config(path: str) -> dict[str, Any]:
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"config {path} must be a mapping")
    return data


def setup_logging(config: dict[str, Any]) -> None:
    level_name = str((config.get("logging") or {}).get("level", "INFO")).upper()
    logging.basicConfig(
        level=getattr(logging, level_name, logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )


def run(config: dict[str, Any], *, mock: bool, dummy_arm: bool, record_path: str | None = None) -> None:
    if dummy_arm:
        robot = dict(config.get("robot") or {})
        robot["adapter"] = "dummy"
        config["robot"] = robot
    if mock:
        vive = dict(config.get("vive") or {})
        vive["tracker_config"] = str(ROOT / "config" / "tracker.mock.yaml")
        config["vive"] = vive

    adapter_name = str((config.get("robot") or {}).get("adapter", "dummy")).upper()
    logger.warning("ROBOT ADAPTER: %s", adapter_name)
    adapter = create_adapter(config)
    adapter.connect()
    pipeline = TeleopPipeline(config, adapter.get_ee_pose)
    source = VivePacketSource.from_config(config, mock=mock)
    source.start()

    pub = None
    arm_cfg = dict(config.get("arm_state") or {})
    if bool(arm_cfg.get("enabled", True)):
        from state_pub import ARM_TOPIC, ArmStatePublisher

        pub = ArmStatePublisher(
            adapter.get_arm_state,
            topic=str(arm_cfg.get("topic") or ARM_TOPIC),
            hz=float(arm_cfg.get("hz", 50)),
        )
        pub.start()

    logger.info(
        "[TELEOP] %s  Space=deadman  h=home  p=pause  g=gripper  head_base=%s yaw=%s offset=%.0f  vr_space=%s",
        pipeline.safety.state.value,
        pipeline.head_base_enabled,
        pipeline.head_base.yaw_source,
        pipeline.head_base.yaw_offset_deg,
        pipeline.vr_space,
    )
    recorder = None
    if record_path:
        from datetime import datetime

        from mapping_log import MappingRecorder

        vive = dict(config.get("vive") or {})
        path = Path(record_path)
        if record_path == "auto":
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            path = ROOT / "data" / f"mapping_{stamp}.jsonl"
        serial_by_name: dict[str, str] = {}
        tracker_yaml = Path(vive.get("tracker_config") or (ROOT / "config" / "tracker.yaml"))
        if not tracker_yaml.is_absolute():
            tracker_yaml = ROOT / tracker_yaml
        if tracker_yaml.is_file():
            loaded = yaml.safe_load(tracker_yaml.read_text(encoding="utf-8")) or {}
            for row in loaded.get("trackers") or []:
                serial_by_name[str(row.get("logical_name"))] = str(row.get("serial"))
        head_name = str(vive.get("head") or "body_hmd")
        hand_name = str(vive.get("hand") or "hand_controller")
        recorder = MappingRecorder(
            path,
            meta={
                "head": head_name,
                "hand": hand_name,
                "vr_space": pipeline.vr_space,
                "head_base": pipeline.head_base_enabled,
                "head_base_yaw": pipeline.head_base.yaw_source,
                "head_serial": serial_by_name.get(head_name),
                "hand_serial": serial_by_name.get(hand_name),
            },
        )
        logger.info("[RECORD] mapping → %s", recorder.path)
    last_status = 0.0
    saw_packet = False
    try:
        while True:
            packet = source.poll_latest()
            if packet is None:
                if source.timed_out() and pipeline.safety.state not in {TeleopState.DISCONNECTED}:
                    result = pipeline.on_timeout()
                    if result.should_stop:
                        adapter.stop()
                    pipeline.safety.state = TeleopState.DISCONNECTED
                    logger.info("[VIVE] Timeout > %.2fs", source.timeout_sec)
                now = time.monotonic()
                if now - last_status >= 1.0:
                    last_status = now
                    logger.info("[VIVE] waiting for trackers")
                time.sleep(0.01)
                continue

            if not saw_packet:
                saw_packet = True
                logger.info("[VIVE] tracking head=%s hand=%s", packet.hmd.valid, packet.right_controller.valid)

            result = pipeline.process(packet)
            if result.should_go_home:
                adapter.go_home()
            elif result.command is not None and result.command.enabled:
                adapter.send_ee_target(result.command)
            elif result.should_stop:
                adapter.stop()

            if recorder is not None:
                recorder.maybe_write(
                    pipeline=pipeline,
                    packet=packet,
                    result=result,
                    ee_pose=adapter.get_ee_pose(),
                )

            now = time.monotonic()
            if now - last_status >= 1.0:
                last_status = now
                logger.info(
                    "[VIVE] seq=%s state=%s clutch=%s head=%s hand=%s",
                    packet.seq,
                    result.state.value,
                    result.clutch_active,
                    packet.hmd.valid,
                    packet.right_controller.valid,
                )
            time.sleep(0.005)
    except KeyboardInterrupt:
        logger.info("shutting down")
    finally:
        if recorder is not None:
            recorder.close()
            logger.info("[RECORD] wrote %s samples → %s", recorder.count, recorder.path)
        if pub is not None:
            pub.stop()
        adapter.stop()
        adapter.disconnect()
        source.stop()


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    setup_logging(config)
    run(config, mock=args.mock, dummy_arm=args.dummy_arm, record_path=args.record)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
