"""middle_interface entry: UDP → process → DummyRobotAdapter."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Any

import yaml

from processing.pipeline import TeleopPipeline
from processing.safety import TeleopState
from pose_recorder import PoseRecorder
from robot import create_adapter

_COMMAND = Path(__file__).resolve().parent.parent / "Robot_control_interface"
if str(_COMMAND) not in sys.path:
    sys.path.insert(0, str(_COMMAND))

logger = logging.getLogger("middle_interface")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PICO teleop middle interface")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "config" / "default.yaml"),
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
    level = getattr(logging, level_name, logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )


def run(config: dict[str, Any]) -> None:
    from receiver.udp_receiver import UdpReceiver

    adapter_name = str((config.get("robot") or {}).get("adapter", "dummy")).upper()
    if adapter_name == "DUMMY":
        logger.warning("ROBOT ADAPTER: DUMMY")
    else:
        logger.warning("ROBOT ADAPTER: REAL (%s)", adapter_name)

    adapter = create_adapter(config)
    adapter.connect()
    pipeline = TeleopPipeline(config, adapter.get_ee_pose)
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
    recorder = PoseRecorder.from_config(
        config.get("recording"), pipeline.controller
    )

    net = config.get("network") or {}
    receiver = UdpReceiver(
        host=str(net.get("listen_ip", "0.0.0.0")),
        port=int(net.get("port", 15000)),
        timeout_sec=float(net.get("timeout_sec", 0.5)),
    )
    receiver.bind()

    last_status = 0.0
    saw_packet = False
    logger.info(
        "[TELEOP] %s  deadman=%s  gripper=%s  home=%s  head_base=%s  vr_space=%s",
        pipeline.safety.state.value,
        pipeline.deadman_source,
        pipeline.gripper_source,
        pipeline.home_button,
        pipeline.head_base_enabled,
        pipeline.vr_space,
    )

    try:
        while True:
            packet = receiver.poll_latest()
            if packet is None:
                if receiver.timed_out() and pipeline.safety.state not in {
                    TeleopState.DISCONNECTED,
                }:
                    result = pipeline.on_timeout()
                    if result.should_stop:
                        adapter.stop()
                    if pipeline.safety.state != TeleopState.DISCONNECTED:
                        pipeline.safety.state = TeleopState.DISCONNECTED
                        logger.info("[PICO] Timeout > %.2fs", receiver.timeout_sec)
                        logger.info("[TELEOP] DISCONNECTED")
                _maybe_status(receiver, pipeline, last_status)
                last_status = _bump_status(last_status)
                continue

            if not saw_packet:
                saw_packet = True
                logger.info("[PICO] Connected from %s", packet.source_addr)

            actual_pose = None
            if recorder is not None:
                try:
                    actual_pose = adapter.get_ee_pose()
                except Exception:
                    logger.exception("[RECORD] failed to read actual robot TCP")

            result = pipeline.process(packet)
            action = "hold"
            if result.should_go_home:
                try:
                    adapter.go_home()
                    action = "go_home"
                except Exception:
                    logger.exception(
                        "[TELEOP] go_home failed; staying in teleop. "
                        "Move the arm into joint limits before pressing A"
                    )
                    action = "go_home_failed"
            elif result.command is not None and result.command.enabled:
                adapter.send_ee_target(result.command)
                action = "send"
            elif result.should_stop:
                adapter.stop()
                action = "stop"

            if recorder is not None:
                recorder.record(packet, actual_pose, result, action)

            now = time.monotonic()
            if now - last_status >= 1.0:
                last_status = now
                _log_rx(receiver, packet, result)
    except KeyboardInterrupt:
        logger.info("shutting down")
    finally:
        if pub is not None:
            pub.stop()
        adapter.stop()
        adapter.disconnect()
        receiver.close()
        if recorder is not None:
            recorder.close()


def _log_rx(receiver, packet, result) -> None:
    stats = receiver.stats
    logger.info(
        "[PICO] RX: %.1f Hz seq=%s loss=%s src=%s  state=%s",
        stats.receive_rate_hz,
        stats.last_seq,
        stats.packet_loss,
        stats.last_source,
        result.state.value,
    )
    cmd = result.command
    if cmd is None:
        logger.info("[ROBOT] CMD HOLD  tracking=%s  clutch=%s", result.tracking_ok, result.clutch_active)
        return
    logger.info(
        "[ROBOT] CMD SEND pos=%s rot=%s gripper=%.2f src=%s",
        _fmt_vec(cmd.position),
        _fmt_vec(cmd.rotation),
        cmd.gripper,
        cmd.source,
    )


def _fmt_vec(value):
    return [round(float(x), 3) for x in value]


def _maybe_status(receiver, pipeline, last_status: float) -> None:
    now = time.monotonic()
    if now - last_status < 1.0:
        return
    if pipeline.safety.state == TeleopState.DISCONNECTED:
        logger.info("[PICO] waiting for packets")


def _bump_status(last_status: float) -> float:
    now = time.monotonic()
    return now if now - last_status >= 1.0 else last_status


def main() -> None:
    args = parse_args()
    config = load_config(args.config)
    setup_logging(config)
    run(config)


if __name__ == "__main__":
    sys.exit(main())
