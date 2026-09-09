#!/usr/bin/env python3
"""Print PicoTeleopPacket / optional TeleopPipeline target (no real arm motion).

Uses DummyRobotAdapter. Space/h/p/g still work if stdin is a TTY.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
MIDDLE = ROOT.parent / "middle_interface"
COMMAND = ROOT.parent / "Robot_control_interface"
for path in (str(ROOT), str(MIDDLE), str(COMMAND)):
    if path not in sys.path:
        sys.path.insert(0, path)

from vive_openvr.keyboard_input import KeyboardButtons
from vive_openvr.openvr_backend import OpenVRInitError
from vive_openvr.packet_source import OpenVRPacketSource


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"config {path} must be a mapping")
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config" / "teleop.yaml",
    )
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--pipeline",
        action="store_true",
        help="Also run TeleopPipeline with dummy arm and print targets",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    config = load_config(args.config)
    # Force dummy — this script must not move a real arm.
    robot = dict(config.get("robot") or {})
    robot["adapter"] = "dummy"
    config["robot"] = robot

    buttons = KeyboardButtons()
    source = OpenVRPacketSource.from_config(config)
    # Replace auto-owned keyboard with ours so we can inject if needed.
    source._buttons = buttons
    source._owns_buttons = True

    try:
        source.start()
    except OpenVRInitError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    pipeline = None
    adapter = None
    if args.pipeline:
        from processing.pipeline import TeleopPipeline
        from robot import create_adapter

        adapter = create_adapter(config)
        adapter.connect()
        pipeline = TeleopPipeline(config, adapter.get_ee_pose)
        print(
            f"[PIPELINE] state={pipeline.safety.state.value}  "
            f"vr_space={pipeline.vr_space}  head_base={pipeline.head_base_enabled}"
        )
        print("Space=deadman  h=home  p=pause  g=gripper")

    period = 1.0 / max(0.1, float(args.hz))
    t0 = time.monotonic()
    try:
        while True:
            if args.duration > 0.0 and (time.monotonic() - t0) >= args.duration:
                break
            packet = source.poll_latest()
            now = time.monotonic() - t0
            if packet is None:
                print(f"[{now:7.2f}s] waiting for trackers ...")
                time.sleep(period)
                continue
            hp = packet.hmd.position
            rp = packet.right_controller.position
            print(
                f"[{now:7.2f}s] seq={packet.seq}  "
                f"hmd={packet.hmd.valid}  hand={packet.right_controller.valid}  "
                f"trig={packet.right_controller.trigger:.1f}  "
                f"grip={packet.right_controller.grip:.1f}"
            )
            print(f"  hmd xyz   {hp[0]:+.4f} {hp[1]:+.4f} {hp[2]:+.4f}")
            print(f"  hand xyz  {rp[0]:+.4f} {rp[1]:+.4f} {rp[2]:+.4f}")
            if pipeline is not None and adapter is not None:
                result = pipeline.process(packet)
                cmd = result.command
                if cmd is not None and cmd.enabled:
                    print(
                        f"  target xyz  {cmd.position[0]:+.4f} "
                        f"{cmd.position[1]:+.4f} {cmd.position[2]:+.4f}  "
                        f"state={result.state.value} clutch={result.clutch_active}"
                    )
                else:
                    print(
                        f"  target —  state={result.state.value} "
                        f"clutch={result.clutch_active} stop={result.should_stop}"
                    )
            print()
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if adapter is not None:
            adapter.stop()
            adapter.disconnect()
        source.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
