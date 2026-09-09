#!/usr/bin/env python3
"""Listen to SayoDevice CM6K (or any path) and print keycodes for pedal mapping.

Example:
  python3 scripts/probe_pedal.py
  python3 scripts/probe_pedal.py --device /dev/input/by-id/usb-SayoDevice_CM6K_*-event-kbd

If Permission denied: install docs/99-sayodevice-pedal.rules then replug, or
  sudo usermod -aG input $USER && newgrp input
"""

from __future__ import annotations

import argparse
import glob
import sys
import time
from pathlib import Path


def resolve_device(pattern: str) -> str:
    matches = sorted(glob.glob(pattern)) if any(c in pattern for c in "*?[") else []
    if Path(pattern).exists():
        return pattern
    if matches:
        return matches[0]
    for cand in sorted(glob.glob("/dev/input/by-id/usb-SayoDevice_CM6K_*-event-kbd")):
        return cand
    raise SystemExit(f"no pedal device for pattern={pattern!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device",
        default="/dev/input/by-id/usb-SayoDevice_CM6K_*-event-kbd",
    )
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()

    try:
        from evdev import InputDevice, categorize, ecodes
    except ImportError:
        print("pip install evdev", file=sys.stderr)
        return 1

    path = resolve_device(args.device)
    try:
        dev = InputDevice(path)
    except PermissionError:
        print(
            f"Permission denied opening {path}\n"
            "Fix: sudo cp docs/99-sayodevice-pedal.rules /etc/udev/rules.d/ && "
            "sudo udevadm control --reload-rules && sudo udevadm trigger\n"
            "Or: sudo usermod -aG input $USER  (then re-login)",
            file=sys.stderr,
        )
        return 1

    print(f"Listening on {path} ({dev.name}) for {args.seconds:.0f}s")
    print("Press left / middle / right pedals; note KEY_* for teleop.yaml")
    deadline = time.monotonic() + float(args.seconds)
    while time.monotonic() < deadline:
        event = dev.read_one()
        if event is None:
            time.sleep(0.01)
            continue
        if event.type != ecodes.EV_KEY:
            continue
        key = categorize(event)
        state = {0: "UP", 1: "DOWN", 2: "HOLD"}.get(key.keystate, str(key.keystate))
        print(f"  {key.keycode}  {state}")
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
