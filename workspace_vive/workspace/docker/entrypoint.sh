#!/bin/bash
set -e
# shellcheck source=/dev/null
source /opt/airbot_env.sh
cd "${AIRBOT_WORKSPACE:-/home/ubuntu/workspace}"
exec "$@"
