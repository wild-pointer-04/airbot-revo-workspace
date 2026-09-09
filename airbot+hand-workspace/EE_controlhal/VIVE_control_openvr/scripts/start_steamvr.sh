#!/usr/bin/env bash
# Start SteamVR on the host for OpenVR tracker tests.
# See docs/steamvr_setup.md for Null Driver / Room Setup notes.
set -euo pipefail

STEAMVR_ROOT="${STEAMVR_ROOT:-$HOME/.local/share/Steam/steamapps/common/SteamVR}"
VRSTARTUP="${STEAMVR_ROOT}/bin/vrstartup.sh"
STEAM_BIN="${STEAM_BIN:-$(command -v steam || true)}"

# SteamVR needs a graphical display. Cursor/SSH terminals often have DISPLAY unset.
if [[ -z "${DISPLAY:-}" ]]; then
  if [[ -S /tmp/.X11-unix/X1 ]]; then
    export DISPLAY=:1
  elif [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
  fi
  if [[ -n "${DISPLAY:-}" ]]; then
    echo "DISPLAY was unset; using DISPLAY=${DISPLAY}"
  else
    echo "DISPLAY is unset and no /tmp/.X11-unix/X* found." >&2
    echo "Open a desktop terminal or: export DISPLAY=:1" >&2
    exit 1
  fi
fi

is_running() {
  pgrep -x vrmonitor >/dev/null 2>&1 || pgrep -x vrserver >/dev/null 2>&1
}

close_room_setup() {
  # Close Null-Driver Room Setup popup. Match the real binary argv only.
  local i pid
  for i in $(seq 1 12); do
    pid="$(
      ps -eo pid=,args= | awk '
        /tools\/steamvr_room_setup\/linux64\/steamvr_room_setup -popupwindow/ {print $1; exit}
      '
    )"
    if [[ -n "${pid}" ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
      echo "Closed SteamVR Room Setup popup (pid ${pid})."
      return 0
    fi
    sleep 0.5
  done
  return 0
}

if is_running; then
  echo "SteamVR already running (vrmonitor/vrserver found)."
  close_room_setup || true
  exit 0
fi

# Prefer steam AppID so Steam runtime / library paths are correct.
if [[ -n "${STEAM_BIN}" ]]; then
  echo "Starting SteamVR via: steam -applaunch 250820"
  nohup "${STEAM_BIN}" -applaunch 250820 >/tmp/steamvr_launch.log 2>&1 &
else
  if [[ ! -x "${VRSTARTUP}" ]]; then
    echo "SteamVR not found at ${STEAMVR_ROOT}" >&2
    echo "Install SteamVR (AppID 250820) or set STEAMVR_ROOT." >&2
    exit 1
  fi
  echo "Starting SteamVR via: ${VRSTARTUP}"
  nohup "${VRSTARTUP}" >/tmp/steamvr_launch.log 2>&1 &
fi

echo -n "Waiting for vrserver"
for i in $(seq 1 60); do
  if is_running; then
    echo " — OK"
    close_room_setup || true
    echo "Log: /tmp/steamvr_launch.log"
    echo "Next: python3 scripts/list_trackers.py"
    exit 0
  fi
  echo -n "."
  sleep 1
done

echo
echo "Timed out waiting for SteamVR. Check /tmp/steamvr_launch.log" >&2
echo "Also confirm Null Driver / requireHmd=false (docs/steamvr_setup.md)." >&2
exit 1
