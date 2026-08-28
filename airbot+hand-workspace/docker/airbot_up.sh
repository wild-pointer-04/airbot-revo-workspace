#!/bin/bash
set -euo pipefail

WORKSPACE="${AIRBOT_WORKSPACE:-/home/ubuntu/airbot+hand-workspace}"
STATE_DIR="${WORKSPACE}/.runtime"
LOG_DIR="${WORKSPACE}/logs/airbot"
CID_FILE="${STATE_DIR}/airbot_server.cid"
PID_FILE="${STATE_DIR}/airbot_server.pid"
LOG_PID_FILE="${STATE_DIR}/airbot_server.log.pid"
RUNTIME_IMAGE="${AIRBOT_RUNTIME_IMAGE:-registry.cn-shanghai.aliyuncs.com/discover-robotics/airbot-runtime:5.1.6}"
CONTAINER_NAME="airbot-play-runtime"
PORT=50051
CAN_IFACE=""
ACTION=start

usage() {
    echo "Usage: airbot_up.sh [start|stop|status|logs] [--can canX] [--port PORT]"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        start|stop|status|logs) ACTION="$1"; shift ;;
        --can) CAN_IFACE="${2:?missing CAN interface}"; shift 2 ;;
        --port) PORT="${2:?missing port}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

mkdir -p "${STATE_DIR}" "${LOG_DIR}"

runtime_exists() {
    docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1
}

runtime_running() {
    [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || true)" = true ]
}

case "${ACTION}" in
    stop)
        if runtime_exists; then
            docker stop "${CONTAINER_NAME}" >/dev/null
            echo "AIRBOT driver stopped."
        else
            echo "AIRBOT driver is not running."
        fi
        rm -f "${CID_FILE}" "${PID_FILE}" "${LOG_PID_FILE}"
        exit 0
        ;;
    status)
        if runtime_running; then
            echo "AIRBOT driver is running: $(cat "${CID_FILE}" 2>/dev/null || true)"
            echo "Host PID: $(cat "${PID_FILE}" 2>/dev/null || true)"
            docker ps --filter "name=^/${CONTAINER_NAME}$" --format 'CAN driver: {{.Status}}'
            exit 0
        fi
        echo "AIRBOT driver is stopped."
        exit 1
        ;;
    logs)
        exec docker logs -f "${CONTAINER_NAME}"
        ;;
esac

if runtime_running; then
    echo "AIRBOT driver is already running." >&2
    exit 1
fi
if runtime_exists; then
    docker rm "${CONTAINER_NAME}" >/dev/null
fi

mapfile -t CAN_INTERFACES < <(ip -o link show type can | awk -F': ' '{print $2}' | cut -d@ -f1 | sort -u)
if [ -z "${CAN_IFACE}" ]; then
    if [ "${#CAN_INTERFACES[@]}" -eq 1 ]; then
        CAN_IFACE="${CAN_INTERFACES[0]}"
    elif [ "${#CAN_INTERFACES[@]}" -eq 0 ]; then
        echo "No SocketCAN interface found." >&2
        exit 1
    else
        echo "Multiple CAN interfaces found: ${CAN_INTERFACES[*]}" >&2
        echo "Select one explicitly, for example: airbot_up.sh start --can can0" >&2
        exit 2
    fi
fi

if ! printf '%s\n' "${CAN_INTERFACES[@]}" | grep -Fxq "${CAN_IFACE}"; then
    echo "CAN interface does not exist: ${CAN_IFACE}" >&2
    exit 1
fi
if pgrep -af 'X5Controller|arx_x5|bringup_single_arm' >/dev/null 2>&1; then
    echo "An old ARX/X5 controller process is running. Stop it before AIRBOT." >&2
    exit 1
fi
if ss -ltn "sport = :${PORT}" | tail -n +2 | grep -q .; then
    echo "TCP port ${PORT} is already in use." >&2
    exit 1
fi

TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="${LOG_DIR}/${TIMESTAMP}.log"
CID="$(docker run -d --rm \
    --name "${CONTAINER_NAME}" \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v /run/udev:/run/udev:ro \
    -v /sys:/sys:ro \
    "${RUNTIME_IMAGE}" \
    /bin/bash -lc "source /root/ros2_ws/install/setup.bash && exec ros2 run fsm fsm_node -i '${CAN_IFACE}' -p '${PORT}'")"

echo "${CID}" > "${CID_FILE}"
docker inspect -f '{{.State.Pid}}' "${CONTAINER_NAME}" > "${PID_FILE}"
docker logs -f "${CONTAINER_NAME}" > "${LOG_FILE}" 2>&1 &
echo "$!" > "${LOG_PID_FILE}"
sleep 2
if ! runtime_running; then
    echo "AIRBOT driver exited during startup. See ${LOG_FILE}" >&2
    tail -n 40 "${LOG_FILE}" >&2 || true
    exit 1
fi

echo "AIRBOT driver started: CAN=${CAN_IFACE}, port=${PORT}"
echo "Log: ${LOG_FILE}"
