"""60 Hz table of the Cartesian command sent to the robot adapter."""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

import numpy as np
import yaml

from processing.pipeline import TeleopPipeline
from processing.safety import TeleopState
from protocol.pico_packet import Pose
from protocol.robot_command import RobotCommand
from receiver.udp_receiver import UdpReceiver
from robot import create_adapter

HOME = "\033[H\033[J"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="60 Hz table of VR signal after teleop → robot command"
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[1] / "config" / "default.yaml"),
    )
    parser.add_argument("--hz", type=float, default=60.0)
    return parser.parse_args()


def _fmt3(values) -> str:
    return "  ".join(f"{float(v):8.3f}" for v in values)


def _row(name: str, pos, rot, extra: str) -> str:
    return f"{name:<8}  {_fmt3(pos)}  {_fmt3(rot)}  {extra}"


def render(
    *,
    hz: float,
    seq,
    loss: int,
    state: str,
    reason: str,
    sent: bool,
    clutch: bool,
    tracking: bool,
    deadman: float,
    grip_raw: float,
    command: RobotCommand | None,
    ee_now: Pose | None,
    home: Pose | None,
    baseline: Pose | None = None,
    delta_pos=None,
) -> str:
    send_txt = "SEND" if sent else "HOLD"
    header = (
        f"ROBOT CMD  {send_txt}  {hz:5.1f} Hz  seq={seq}  loss={loss}  "
        f"state={state}  reason={reason}\n"
        f"deadman={deadman:4.2f}  clutch={'ON ' if clutch else 'off'}  "
        f"grip={grip_raw:4.2f}  tracking={'OK' if tracking else 'LOST'}\n"
        f"{'stream':<8}  "
        f"{'x':>8}  {'y':>8}  {'z':>8}  "
        f"{'qx':>8}  {'qy':>8}  {'qz':>8}  {'qw':>8}  extra\n"
        + "-" * 108
    )
    rows = [header]
    ident = np.array([0.0, 0.0, 0.0, 1.0])
    if baseline is not None and sent:
        rows.append(_row("EE_base", baseline.position, baseline.rotation, "frozen TCP"))
    if delta_pos is not None and sent:
        rows.append(_row("d_vr", delta_pos, ident, "head-base delta"))
    if sent and command is not None:
        extra = f"grip={command.gripper:4.2f}  en={int(command.enabled)}  {command.source}"
        rows.append(_row("EE_cmd", command.position, command.rotation, extra))
        if home is not None:
            rows.append(
                _row("d_home", command.position - home.position, ident, "cmd - dummy home")
            )
    elif command is not None:
        extra = f"grip={command.gripper:4.2f}  last sent (not updating)"
        rows.append(_row("EE_hold", command.position, command.rotation, extra))
    else:
        rows.append("EE_cmd    (not sending — press right index trigger)")
    if ee_now is not None:
        rows.append(_row("EE_now", ee_now.position, ee_now.rotation, "adapter pose"))
    return "\n".join(rows)


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(levelname)s %(message)s")
    with open(args.config, encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError(f"config {args.config} must be a mapping")

    adapter = create_adapter(config)
    adapter.connect()
    pipeline = TeleopPipeline(config, adapter.get_ee_pose)
    home = adapter.get_ee_pose()

    net = config.get("network") or {}
    interval = 1.0 / max(args.hz, 1.0)
    link_timeout = float(net.get("timeout_sec", 0.5))
    rx = UdpReceiver(
        host=str(net.get("listen_ip", "0.0.0.0")),
        port=int(net.get("port", 15000)),
        timeout_sec=link_timeout,
    )
    rx.bind()

    last_command: RobotCommand | None = None
    last_sent = False
    last_tracking = False
    last_deadman = 0.0
    last_grip = 0.0
    last_baseline: Pose | None = None
    last_delta = None
    last_print = 0.0
    try:
        while True:
            packet = rx.poll_latest(timeout=interval)
            if packet is None:
                if rx.timed_out() and pipeline.safety.state != TeleopState.DISCONNECTED:
                    result = pipeline.on_timeout()
                    if result.should_stop:
                        adapter.stop()
                    last_sent = False
                    last_baseline = None
                    last_delta = None
            else:
                result = pipeline.process(packet)
                ctrl = packet.controller(pipeline.controller)
                last_deadman = float(
                    ctrl.trigger if pipeline.deadman_source == "trigger" else ctrl.grip
                )
                last_grip = float(ctrl.grip)
                last_tracking = result.tracking_ok
                last_baseline = result.baseline
                last_delta = result.delta_pos
                if result.should_go_home:
                    adapter.go_home()
                    home = adapter.get_ee_pose()
                    last_command = None
                    last_sent = False
                elif result.command is not None and result.command.enabled:
                    adapter.send_ee_target(result.command)
                    last_command = result.command
                    last_sent = True
                else:
                    last_sent = False
                    if result.should_stop:
                        adapter.stop()

            now = time.monotonic()
            if now - last_print < interval:
                continue
            last_print = now
            stats = rx.stats
            print(
                HOME
                + render(
                    hz=stats.receive_rate_hz,
                    seq=stats.last_seq,
                    loss=stats.packet_loss,
                    state=pipeline.safety.state.value,
                    reason=pipeline.safety.reason.value,
                    sent=last_sent,
                    clutch=pipeline.clutch.active,
                    tracking=last_tracking,
                    deadman=last_deadman,
                    grip_raw=last_grip,
                    command=last_command,
                    ee_now=adapter.get_ee_pose() if adapter.is_connected() else None,
                    home=home,
                    baseline=last_baseline,
                    delta_pos=last_delta,
                ),
                flush=True,
            )
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    finally:
        adapter.stop()
        adapter.disconnect()
        rx.close()


if __name__ == "__main__":
    main()
