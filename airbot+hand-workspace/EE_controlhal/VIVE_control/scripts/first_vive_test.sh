#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${LIBSURVIVE_SRC:-${HOME}/third_party/libsurvive}"
cd "${ROOT}"

fail_step() {
  local step="$1"
  local cause="$2"
  local next="$3"
  echo
  echo "FAILED STEP: ${step}"
  echo "Possible cause: ${cause}"
  echo "Next command: ${next}"
  exit 1
}

echo "=== first VIVE hardware test ==="
echo "Project: ${ROOT}"
echo

echo "STEP 1  check libsurvive"
if [[ ! -d "${SRC_DIR}" ]]; then
  fail_step "check libsurvive" "source tree missing" "./scripts/install_libsurvive.sh"
fi
if ! python3 -c "import pysurvive" 2>/dev/null && \
   ! ldconfig -p 2>/dev/null | grep -q libsurvive; then
  fail_step "check libsurvive" "pysurvive and libsurvive.so both missing" "./scripts/install_libsurvive.sh"
fi
echo "  libsurvive source: ${SRC_DIR}"
echo

echo "STEP 2  check udev"
if ! ls /etc/udev/rules.d/*vive* /etc/udev/rules.d/*survive* >/dev/null 2>&1; then
  fail_step "check udev" "udev rules not installed" "./scripts/install_udev_rules.sh"
fi
echo "  udev rules present"
echo

echo "STEP 3  check USB dongle"
if command -v lsusb >/dev/null 2>&1; then
  lsusb || true
  echo "  If the dongle is plugged in and no HTC/Valve line appears, continue anyway:"
  echo "  VID/PID names in lsusb are not a hard presence test."
else
  echo "  lsusb not installed; skipping USB listing."
fi
echo

echo "STEP 4  operator checklist"
echo "  - Turn on Base Station 2.0"
echo "  - Turn on VIVE Tracker 3.0"
echo "  - Insert VIVE Tracker Dongle"
echo "  - Close SteamVR (do not run it during tracking)"
echo
read -r -p "Press Enter when the hardware is on, or Ctrl+C to abort..."
echo

echo "STEP 5  survive-cli"
SURVIVE_CLI="$(command -v survive-cli || true)"
if [[ -z "${SURVIVE_CLI}" ]]; then
  for candidate in \
      "${SRC_DIR}/survive-cli" \
      "${SRC_DIR}/bin/survive-cli" \
      "${SRC_DIR}/build/survive-cli" \
      /usr/local/bin/survive-cli
  do
    if [[ -x "${candidate}" ]]; then
      SURVIVE_CLI="${candidate}"
      break
    fi
  done
fi
if [[ -z "${SURVIVE_CLI}" ]]; then
  fail_step "run survive-cli" "survive-cli binary not found" "./scripts/install_libsurvive.sh"
fi
echo "  running: timeout 8s ${SURVIVE_CLI}"
if ! timeout 8s "${SURVIVE_CLI}"; then
  status=$?
  if [[ "${status}" -ne 124 ]]; then
    fail_step "run survive-cli" "survive-cli exited with status ${status}" \
      "${SURVIVE_CLI}   # watch for lighthouse / tracker lines"
  fi
fi
echo

echo "STEP 6  Python list_devices.py"
if ! python3 scripts/list_devices.py; then
  fail_step "python list_devices.py" "backend failed to start or import" \
    "python3 scripts/list_devices.py"
fi
echo

echo "STEP 7  monitor_pose.py (3 seconds)"
if ! python3 scripts/monitor_pose.py --duration 3; then
  fail_step "monitor_pose.py" "no pose stream" \
    "python3 scripts/monitor_pose.py --duration 5"
fi
echo

echo "STEP 8  diagnostics (5 seconds)"
if ! python3 scripts/run_diagnostics.py --duration 5; then
  status=$?
  if [[ "${status}" -eq 2 ]]; then
    echo "  Diagnostics returned FAIL. That can be a tracking-quality issue, not a missing install."
  else
    fail_step "diagnostics" "diagnostics script crashed" \
      "python3 scripts/run_diagnostics.py --duration 10"
  fi
fi

echo
echo "First VIVE test finished."
echo "Copy the printed LHR-XXXXXXXX into config/tracker.yaml and set backend: libsurvive"
