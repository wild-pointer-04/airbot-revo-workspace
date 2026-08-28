#!/usr/bin/env bash
# USB: PICO localhost:15000 → Ubuntu :15000
set -euo pipefail
if ! command -v adb >/dev/null; then
  echo "adb not found. Install with: sudo apt-get update && sudo apt-get install -y adb"
  exit 1
fi
adb start-server
adb devices -l
adb reverse tcp:15000 tcp:15000
adb reverse --list
echo
echo "Unity PicoTeleopManager Receiver Ip = 127.0.0.1  Port = 15000"
echo "Keep python3 app.py running. Stop mock_pico_sender before the real headset."
