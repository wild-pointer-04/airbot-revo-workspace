#!/usr/bin/env bash
set -u

SRC_DIR="${LIBSURVIVE_SRC:-${HOME}/third_party/libsurvive}"

ok() { printf '  [OK]   %s\n' "$1"; }
miss() { printf '  [MISS] %s\n' "$1"; }
info() { printf '  [INFO] %s\n' "$1"; }

echo "=== hardware check (report only) ==="
echo

echo "Linux version"
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  ok "${PRETTY_NAME:-unknown} kernel $(uname -r)"
else
  miss "/etc/os-release"
fi
echo

echo "USB devices"
if command -v lsusb >/dev/null 2>&1; then
  if lsusb >/tmp/vive_lsusb.txt 2>/dev/null; then
    ok "lsusb ran"
    info "VID/PID mismatch is NOT treated as 'device absent'"
    grep -Ei 'vive|htc|valve|28de|0bb4|28ae' /tmp/vive_lsusb.txt || true
    echo "  (full lsusb follows)"
    sed 's/^/  /' /tmp/vive_lsusb.txt
  else
    miss "lsusb failed"
  fi
else
  miss "lsusb not installed"
fi
echo

echo "hidraw devices"
if ls /dev/hidraw* >/dev/null 2>&1; then
  ok "$(ls /dev/hidraw* | tr '\n' ' ')"
  ls -l /dev/hidraw* | sed 's/^/  /'
else
  miss "no /dev/hidraw* nodes"
fi
echo

echo "libusb"
if ldconfig -p 2>/dev/null | grep -q libusb-1.0; then
  ok "libusb-1.0 present"
else
  miss "libusb-1.0 not found in ldconfig"
fi
echo

echo "libsurvive"
if [[ -d "${SRC_DIR}/.git" ]]; then
  ok "source at ${SRC_DIR}"
else
  miss "source not found at ${SRC_DIR}"
fi
if ldconfig -p 2>/dev/null | grep -q libsurvive; then
  ok "libsurvive.so in linker cache"
else
  miss "libsurvive.so not in linker cache"
fi
echo

echo "survive-cli"
if command -v survive-cli >/dev/null 2>&1; then
  ok "$(command -v survive-cli)"
else
  found=""
  for candidate in \
      "${SRC_DIR}/survive-cli" \
      "${SRC_DIR}/bin/survive-cli" \
      "${SRC_DIR}/build/survive-cli" \
      /usr/local/bin/survive-cli
  do
    if [[ -x "${candidate}" ]]; then
      found="${candidate}"
      break
    fi
  done
  if [[ -n "${found}" ]]; then
    ok "${found} (not in PATH)"
  else
    miss "survive-cli not found"
  fi
fi
echo

echo "udev rules"
if ls /etc/udev/rules.d/*vive* /etc/udev/rules.d/*survive* >/dev/null 2>&1; then
  ok "vive/survive rules in /etc/udev/rules.d/"
  ls /etc/udev/rules.d/*vive* /etc/udev/rules.d/*survive* 2>/dev/null | sed 's/^/  /'
else
  miss "no vive/survive udev rules in /etc/udev/rules.d/"
fi
echo

echo "Python binding"
if python3 -c "import pysurvive" 2>/dev/null; then
  ok "import pysurvive"
  python3 -c "import pysurvive,inspect,sys; print('  module:', pysurvive.__file__)"
else
  miss "import pysurvive failed"
fi

echo
echo "This script only reports. A missing VID/PID does not prove the dongle is absent."
