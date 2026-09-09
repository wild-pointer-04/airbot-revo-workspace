#!/usr/bin/env bash
# Stop SteamVR processes started for OpenVR tracker work.
set -euo pipefail

if ! pgrep -x vrmonitor >/dev/null 2>&1 && ! pgrep -x vrserver >/dev/null 2>&1; then
  echo "SteamVR does not appear to be running."
  exit 0
fi

echo "Stopping SteamVR (vrserver / vrmonitor / vrwebhelper)..."
# Graceful first
pkill -x vrmonitor 2>/dev/null || true
pkill -x vrserver 2>/dev/null || true
pkill -x vrwebhelper 2>/dev/null || true
sleep 2
# Force leftover
pkill -9 -x vrmonitor 2>/dev/null || true
pkill -9 -x vrserver 2>/dev/null || true
pkill -9 -x vrwebhelper 2>/dev/null || true

if pgrep -x vrserver >/dev/null 2>&1; then
  echo "Some SteamVR processes still alive:" >&2
  pgrep -a -x 'vrserver|vrmonitor' >&2 || true
  exit 1
fi
echo "SteamVR stopped."
