#!/bin/bash
# Run the AIRBOT command layer inside the container SDK environment.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
set +u
cd "${ROOT}"
if [ "$#" -eq 0 ]; then
  exec python3 app.py --config config/airbot_sdk.yaml
fi
if [[ "$1" == -* ]]; then
  exec python3 app.py "$@"
fi
exec python3 "$@"
