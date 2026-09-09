#!/usr/bin/env python3
"""SteamVR / OpenVR VIVE teleop using the existing middle_interface pipeline.

Run on the Ubuntu host (SteamVR must already be up). Does not modify
middle_interface behavior.

Default robot adapter is dummy (see config/teleop.yaml). Use --real-arm only
when airbot_py can reach play runtime :50051 (typically inside the combined
container, or a host install of the SDK).
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
COMMAND = ROOT.parent / "Robot_control_interface"
for path in (str(ROOT), str(MIDDLE), str(COMMAND)):
    if path not in sys.path:
        sys.path.insert(0, path)

from processing.pipeline import TeleopPipeline
from processing.safety import TeleopState
from robot import create_adapter
from vive_openvr.openvr_backend import OpenVRInitError
from vive_openvr.packet_source import OpenVRPacketSource

logger = logging.getLogger("vive_openvr_teleop")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config" / "teleop.yaml"))
    parser.add_argument(
        "--dummy-arm",
        action="store_true",
        help="Force DummyRobotAdapter (default in teleop.yaml already)",
    )
    parser.add_argument(
        "--real-arm",
        action="store_true",
        help="Use command_layer → AIRBOT Play (needs airbot_py + :50051)",
    )
    parser.add_argument(
        "--print-only",
        action="store_true",
        help="Process pipeline but never send_ee_target (safety dry-run)",
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


def run(
    config: dict[str, Any],
    *,
    dummy_arm: bool,
    real_arm: bool,
    print_only: bool,
) -> None:
    if dummy_arm and real_arm:
        raise SystemExit("use only one of --dummy-arm / --real-arm")
    robot = dict(config.get("robot") or {})
    if real_arm:
        robot["adapter"] = "command_layer"
    elif dummy_arm:
        robot["adapter"] = "dummy"
    config["robot"] = robot

    adapter_name = str(robot.get("adapter", "dummy")).upper()
    logger.warning("ROBOT ADAPTER: %s  print_only=%s", adapter_name, print_only)
    adapter = create_adapter(config)
    adapter.connect()
    pipeline = TeleopPipeline(config, adapter.get_ee_pose)
    source = OpenVRPacketSource.from_config(config)
    try:
        source.start()
    except OpenVRInitError:
        adapter.disconnect()
        raise

    pub = None
    arm_cfg = dict(config.get("arm_state") or {})
    if bool(arm_cfg.get("enabled", False)):
        try:
            from state_pub import ARM_TOPIC, ArmStatePublisher

            pub = ArmStatePublisher(
                adapter.get_arm_state,
                topic=str(arm_cfg.get("topic") or ARM_TOPIC),
                hz=float(arm_cfg.get("hz", 50)),
            )
            pub.start()
        except Exception as exc:
            logger.warning("arm_state publisher unavailable: %s", exc)

    logger.info(
        "[TELEOP] %s  LeftPedal(hold)=deadman  Space=toggle  h=home  p=pause  g=gripper  "
        "head_base=%s yaw=%s offset=%.0f  vr_space=%s",
        pipeline.safety.state.value,
        pipeline.head_base_enabled,
        pipeline.head_base.yaw_source,
        pipeline.head_base.yaw_offset_deg,
        pipeline.vr_space,
    )

    last_status = 0.0
    saw_packet = False
    try:
        while True:
            packet = source.poll_latest()
            if packet is None:
                if source.timed_out() and pipeline.safety.state not in {
                    TeleopState.DISCONNECTED
                }:
                    result = pipeline.on_timeout()
                    if result.should_stop and not print_only:
                        adapter.stop()
                    pipeline.safety.state = TeleopState.DISCONNECTED
                    logger.info("[OPENVR] Timeout > %.2fs", source.timeout_sec)
                now = time.monotonic()
                if now - last_status >= 1.0:
                    last_status = now
                    logger.info("[OPENVR] waiting for trackers")
                time.sleep(0.01)
                continue

            if not saw_packet:
                saw_packet = True
                logger.info(
                    "[OPENVR] tracking head=%s hand=%s",
                    packet.hmd.valid,
                    packet.right_controller.valid,
                )

            result = pipeline.process(packet)
            if print_only:
                if result.command is not None and result.command.enabled:
                    p = result.command.position
                    logger.info(
                        "[PRINT] target %+0.3f %+0.3f %+0.3f  state=%s clutch=%s",
                        p[0],
                        p[1],
                        p[2],
                        result.state.value,
                        result.clutch_active,
                    )
            elif result.should_go_home:
                adapter.go_home()
            elif result.command is not None and result.command.enabled:
                adapter.send_ee_target(result.command)
            elif result.should_stop:
                adapter.stop()

            now = time.monotonic()
            if now - last_status >= 1.0:
                last_status = now
                logger.info(
                    "[OPENVR] seq=%s state=%s clutch=%s head=%s hand=%s",
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
        if pub is not None:
            pub.stop()
        adapter.stop()
        adapter.disconnect()
        source.stop()


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    setup_logging(config)
    try:
        run(
            config,
            dummy_arm=args.dummy_arm,
            real_arm=args.real_arm,
            print_only=args.print_only,
        )
    except OpenVRInitError as exc:
        logger.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
