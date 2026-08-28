"""Send synthetic pico_teleop_v1 UDP packets without a headset."""

from __future__ import annotations

import argparse
import math
import socket
import time

import numpy as np

from protocol.pico_packet import (
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    ControllerState,
    HeadState,
    PicoTeleopPacket,
)


def _controller(
    *,
    valid: bool,
    position: np.ndarray,
    rotation: np.ndarray | None = None,
    trigger: float = 0.0,
    grip: float = 0.0,
) -> ControllerState:
    zero3 = np.zeros(3, dtype=np.float64)
    rot = (
        np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        if rotation is None
        else rotation
    )
    return ControllerState(
        valid=valid,
        position=position,
        rotation=rot,
        linear_velocity=zero3,
        angular_velocity=zero3,
        trigger=trigger,
        grip=grip,
        primary_button=False,
        secondary_button=False,
        joystick=np.zeros(2, dtype=np.float64),
    )


def build_packet(seq: int, t: float, grip: float, trigger: float) -> PicoTeleopPacket:
    right_pos = np.array(
        [
            0.10 * math.sin(2.0 * math.pi * 0.2 * t),
            1.20,
            0.30 + 0.05 * math.cos(2.0 * math.pi * 0.2 * t),
        ],
        dtype=np.float64,
    )
    yaw = 0.3 * math.sin(2.0 * math.pi * 0.1 * t)
    right_rot = np.array(
        [0.0, math.sin(yaw / 2.0), 0.0, math.cos(yaw / 2.0)],
        dtype=np.float64,
    )
    now_ns = time.monotonic_ns()
    wall_ms = int(time.time() * 1000)
    return PicoTeleopPacket(
        protocol=PROTOCOL_NAME,
        version=PROTOCOL_VERSION,
        seq=seq,
        device_time_ns=now_ns,
        wall_time_unix_ms=wall_ms,
        hmd=HeadState(
            valid=True,
            position=np.array([0.0, 1.6, 0.0], dtype=np.float64),
            rotation=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        ),
        left_controller=_controller(
            valid=True,
            position=np.array([-0.20, 1.10, 0.25], dtype=np.float64),
        ),
        right_controller=_controller(
            valid=True,
            position=right_pos,
            rotation=right_rot,
            trigger=trigger,
            grip=grip,
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mock PICO UDP sender")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15000)
    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument(
        "--trigger",
        type=float,
        default=0.8,
        help="index trigger / deadman (0-1). Default 0.8 so teleop engages.",
    )
    parser.add_argument(
        "--trigger-cycle",
        type=float,
        default=0.0,
        help="if >0, toggle trigger every N seconds (press/release)",
    )
    parser.add_argument(
        "--grip",
        type=float,
        default=0.0,
        help="side grip / gripper (0-1)",
    )
    parser.add_argument(
        "--grip-cycle",
        type=float,
        default=0.0,
        help="if >0, toggle grip every N seconds",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / max(args.rate, 1.0)
    seq = 0
    t0 = time.monotonic()
    print(
        f"mock_pico_sender -> {args.host}:{args.port} rate={args.rate} Hz "
        f"trigger={args.trigger} grip={args.grip}",
        flush=True,
    )
    try:
        while True:
            t = time.monotonic() - t0
            trigger = args.trigger
            if args.trigger_cycle > 0:
                trigger = args.trigger if int(t / args.trigger_cycle) % 2 == 0 else 0.0
            grip = args.grip
            if args.grip_cycle > 0:
                grip = args.grip if int(t / args.grip_cycle) % 2 == 0 else 0.0
            seq += 1
            packet = build_packet(seq, t, grip, trigger)
            sock.sendto(packet.to_json().encode("utf-8"), (args.host, args.port))
            time.sleep(interval)
    except KeyboardInterrupt:
        print("stopped", flush=True)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
