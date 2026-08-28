"""Print the Cartesian command that would be sent to the robot."""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

import yaml

from processing.pipeline import TeleopPipeline
from processing.safety import TeleopState
from receiver.udp_receiver import UdpReceiver
from robot import create_adapter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robot-command monitor")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[1] / "config" / "default.yaml"),
    )
    parser.add_argument("--interval", type=float, default=0.5)
    return parser.parse_args()


def _fmt(values) -> list[float]:
    return [round(float(x), 3) for x in values]


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.WARNING)
    with open(args.config, encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError(f"config {args.config} must be a mapping")

    adapter = create_adapter(config)
    adapter.connect()
    pipeline = TeleopPipeline(config, adapter.get_ee_pose)
    net = config.get("network") or {}
    rx = UdpReceiver(
        host=str(net.get("listen_ip", "0.0.0.0")),
        port=int(net.get("port", 15000)),
        timeout_sec=args.interval,
    )
    rx.bind()
    print(
        f"packet_monitor robot-cmd on {net.get('listen_ip', '0.0.0.0')}:{int(net.get('port', 15000))}",
        flush=True,
    )
    last_print = 0.0
    last_cmd = None
    last_sent = False
    try:
        while True:
            packet = rx.poll_latest()
            if packet is None:
                if rx.timed_out() and pipeline.safety.state != TeleopState.DISCONNECTED:
                    result = pipeline.on_timeout()
                    if result.should_stop:
                        adapter.stop()
                    pipeline.safety.state = TeleopState.DISCONNECTED
                    last_sent = False
            else:
                result = pipeline.process(packet)
                if result.should_go_home:
                    adapter.go_home()
                    last_cmd = None
                    last_sent = False
                elif result.command is not None and result.command.enabled:
                    adapter.send_ee_target(result.command)
                    last_cmd = result.command
                    last_sent = True
                else:
                    last_sent = False
                    if result.should_stop:
                        adapter.stop()

            now = time.monotonic()
            if now - last_print < args.interval:
                continue
            last_print = now
            stats = rx.stats
            if last_cmd is None:
                print(
                    f"RX {stats.receive_rate_hz:.1f} Hz  seq={stats.last_seq}  "
                    f"state={pipeline.safety.state.value}  SEND=no  waiting for deadman",
                    flush=True,
                )
                continue
            print(
                f"RX {stats.receive_rate_hz:.1f} Hz  seq={stats.last_seq}  "
                f"loss={stats.packet_loss}  state={pipeline.safety.state.value}  "
                f"{'SEND' if last_sent else 'HOLD'}  "
                f"pos={_fmt(last_cmd.position)}  rot={_fmt(last_cmd.rotation)}  "
                f"gripper={last_cmd.gripper:.2f}  src={last_cmd.source}",
                flush=True,
            )
    except KeyboardInterrupt:
        print("stopped", flush=True)
    finally:
        adapter.stop()
        adapter.disconnect()
        rx.close()


if __name__ == "__main__":
    main()
