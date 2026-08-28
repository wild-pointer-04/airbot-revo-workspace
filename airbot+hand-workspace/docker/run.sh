#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE="airbot-hand-workspace:jazzy"
NAME="airbot-hand-workspace"

if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || true
fi

state="$(docker inspect -f '{{.State.Status}}' "${NAME}" 2>/dev/null || true)"
if [ "${state}" = running ]; then
    exec docker exec -it "${NAME}" bash
elif [ -n "${state}" ]; then
    docker start "${NAME}" >/dev/null
    exec docker exec -it "${NAME}" bash
fi

if [ "${BUILD:-0}" = 1 ] || ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    docker compose -f "${SCRIPT_DIR}/compose.yaml" build
fi

docker compose -f "${SCRIPT_DIR}/compose.yaml" up -d
exec docker exec -it "${NAME}" bash
