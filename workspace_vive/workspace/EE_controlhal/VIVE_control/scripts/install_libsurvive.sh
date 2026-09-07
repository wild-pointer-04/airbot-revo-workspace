#!/usr/bin/env bash
set -euo pipefail

# Build libsurvive from source. Does not delete an existing checkout.
# Ubuntu 24.04: do NOT install libatlas-base-dev (conflicts with liblapacke).

SRC_DIR="${LIBSURVIVE_SRC:-${HOME}/third_party/libsurvive}"
JOBS="${JOBS:-$(nproc)}"

echo "=== install_libsurvive ==="
echo "source dir: ${SRC_DIR}"

sudo apt-get update
sudo apt-get install -y \
  git \
  build-essential \
  cmake \
  pkg-config \
  zlib1g-dev \
  libx11-dev \
  libusb-1.0-0-dev \
  freeglut3-dev \
  liblapacke-dev \
  libopenblas-dev \
  python3-dev \
  python3-pip

mkdir -p "$(dirname "${SRC_DIR}")"

if [[ -d "${SRC_DIR}/.git" ]]; then
  echo "Existing libsurvive checkout found; leaving it in place."
  git -C "${SRC_DIR}" submodule update --init --recursive
elif [[ -e "${SRC_DIR}" ]]; then
  echo "Refusing to overwrite existing path: ${SRC_DIR}"
  echo "Set LIBSURVIVE_SRC to a new directory if you need a fresh clone."
  exit 1
else
  git clone --recursive https://github.com/collabora/libsurvive.git "${SRC_DIR}"
fi

cd "${SRC_DIR}"

if [[ -f Makefile ]]; then
  make -j"${JOBS}"
else
  cmake -S . -B build
  cmake --build build -j"${JOBS}"
fi

if [[ -f CMakeLists.txt ]]; then
  cmake --install build 2>/dev/null || sudo make install 2>/dev/null || true
fi

PY_STATUS="not installed"
if python3 -c "import pysurvive" 2>/dev/null; then
  PY_STATUS="import ok (already present)"
else
  if python3 -m pip install --user .; then
    if python3 -c "import pysurvive" 2>/dev/null; then
      PY_STATUS="import ok (pip install .)"
    else
      PY_STATUS="pip install completed but import still fails"
    fi
  else
    PY_STATUS="pip install failed; try: cd ${SRC_DIR} && python3 -m pip install --user ."
  fi
fi

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

echo
echo "=== libsurvive install summary ==="
echo "libsurvive location: ${SRC_DIR}"
echo "survive-cli location: ${SURVIVE_CLI:-NOT FOUND}"
echo "Python binding status: ${PY_STATUS}"
echo
echo "Next:"
echo "  ./scripts/install_udev_rules.sh"
echo "  then unplug/replug the VIVE dongle"
