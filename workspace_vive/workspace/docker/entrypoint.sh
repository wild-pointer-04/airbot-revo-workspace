#!/bin/bash
set -e
# shellcheck source=/dev/null
source /opt/airbot_env.sh
cd "${AIRBOT_WORKSPACE:-/home/dt/manus-Revo2-Revo3/airbot+hand-workspace}"
exec "$@"
