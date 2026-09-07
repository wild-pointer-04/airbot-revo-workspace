#!/usr/bin/env bash
set -euo pipefail

SRC_DIR="${LIBSURVIVE_SRC:-${HOME}/third_party/libsurvive}"

echo "=== install_udev_rules ==="

if [[ ! -d "${SRC_DIR}" ]]; then
  echo "FAILED STEP: locate libsurvive source"
  echo "Possible cause: libsurvive is not cloned yet."
  echo "Next command: ./scripts/install_libsurvive.sh"
  exit 1
fi

mapfile -t RULES < <(find "${SRC_DIR}" -type f -name '*.rules' | sort)
if [[ ${#RULES[@]} -eq 0 ]]; then
  echo "FAILED STEP: find udev rules"
  echo "Possible cause: unexpected libsurvive tree layout."
  echo "Next command: find ${SRC_DIR} -name '*.rules'"
  exit 1
fi

for rule in "${RULES[@]}"; do
  base="$(basename "${rule}")"
  echo "Installing ${base} from ${rule}"
  sudo cp "${rule}" "/etc/udev/rules.d/${base}"
done

sudo udevadm control --reload-rules
sudo udevadm trigger

echo
echo "udev rules installed from ${SRC_DIR}"
echo "Please unplug and reconnect the VIVE dongle."
