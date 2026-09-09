#!/usr/bin/env bash
# Install udev rule so ubuntu can read SayoDevice CM6K without being in group input.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RULE_SRC="$ROOT/docs/99-sayodevice-pedal.rules"
RULE_DST="/etc/udev/rules.d/99-sayodevice-pedal.rules"
if [[ ! -f "$RULE_SRC" ]]; then
  echo "missing $RULE_SRC" >&2
  exit 1
fi
sudo cp "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=input
echo "Installed $RULE_DST — if still Permission denied, unplug/replug the pedal."
ls -l /dev/input/by-id/usb-SayoDevice_CM6K_*-event-kbd 2>/dev/null || true
