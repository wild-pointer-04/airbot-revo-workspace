#!/bin/bash
# VIVE 臂遥操 + Revo2 右手 + rosbridge + 触觉回传 一键启动。
#
# 用法：
#   ./scripts/teleop_vive_hand_up.sh            # 起手链路后前台跑 VIVE --real-arm
#   ./scripts/teleop_vive_hand_up.sh --dummy-arm # 臂不发真机命令（调试）
#   ./scripts/teleop_vive_hand_up.sh stop        # 停手链路 / rosbridge / 触觉桥（不停臂 sidecar）
#   ./scripts/teleop_vive_hand_up.sh status
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS_ROOT="$(cd "${ROOT}/.." && pwd)"
VIVE_DIR="${ROOT}/EE_controlhal/VIVE_control_openvr"
WORK_CONTAINER="${WORK_CONTAINER:-airbot-revo-humble}"
MANUS_WS="/home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting"
LOG_DIR="${ROOT}/logs"
PID_DIR="${LOG_DIR}/teleop_vive_hand"
HAND_PID_FILE="${PID_DIR}/hand_pipeline.pid"
HAPTIC_PID_FILE="${PID_DIR}/haptic_bridge.pid"
ROSBRIDGE_PID_FILE="${PID_DIR}/rosbridge.pid"

ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$*"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }
step() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

mkdir -p "${PID_DIR}"

container_running() {
    [ "$(docker inspect -f '{{.State.Running}}' "${WORK_CONTAINER}" 2>/dev/null)" = true ]
}

# Precise PID helpers: never broad-pkill (container uses PidMode=host).
_read_pid() {
    local f="$1"
    [ -f "$f" ] || return 1
    local p
    p="$(tr -d ' \n\r' <"$f")"
    [ -n "$p" ] && [ -d "/proc/$p" ] || return 1
    echo "$p"
}

_kill_pidfile() {
    local f="$1" name="$2"
    local p
    if ! p="$(_read_pid "$f")"; then
        rm -f "$f"
        return 0
    fi
    warn "Stopping ${name} (PID ${p})"
    kill -INT "$p" 2>/dev/null || true
    for _ in $(seq 1 15); do
        [ -d "/proc/$p" ] || break
        sleep 0.5
    done
    if [ -d "/proc/$p" ]; then
        warn "${name} still alive, SIGTERM"
        kill -TERM "$p" 2>/dev/null || true
        sleep 1
    fi
    if [ -d "/proc/$p" ]; then
        warn "${name} still alive, SIGKILL"
        kill -KILL "$p" 2>/dev/null || true
    fi
    rm -f "$f"
    ok "${name} stopped"
}

_container_bash() {
    docker exec -u 0 "${WORK_CONTAINER}" bash -lc "$*"
}

_start_rosbridge() {
    if ss -ltn 2>/dev/null | grep -q ':9090'; then
        ok "rosbridge already listening on :9090"
        return 0
    fi
    step "Start rosbridge"
    # shellcheck disable=SC2016
    docker exec -d -u 0 "${WORK_CONTAINER}" bash -lc \
        'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml' \
        >/dev/null 2>&1
    # Find the launch process PID on the host (PidMode=host).
    sleep 1
    local p
    p="$(ps -eo pid=,args= | awk '/rosbridge_websocket_launch\.xml/ && !/awk/ {print $1; exit}')"
    if [ -n "$p" ]; then
        echo "$p" >"${ROSBRIDGE_PID_FILE}"
        ok "rosbridge PID ${p}"
    else
        warn "rosbridge launch started but PID not found; check :9090"
    fi
    for _ in $(seq 1 20); do
        ss -ltn 2>/dev/null | grep -q ':9090' && { ok "rosbridge :9090 up"; return 0; }
        sleep 0.5
    done
    bad "rosbridge did not open :9090"
    return 1
}

_start_hand_pipeline() {
    step "Start Revo2 right-hand pipeline (container)"
    if p="$(_read_pid "${HAND_PID_FILE}")"; then
        warn "hand pipeline already running (PID ${p})"
        return 0
    fi
    local log="${PID_DIR}/hand_pipeline.log"
    # Launch in background inside the container; capture host PID of docker-exec child.
    nohup docker exec -u 0 "${WORK_CONTAINER}" bash -lc "
        source /opt/ros/humble/setup.bash
        source ${MANUS_WS}/install/setup.bash
        exec ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
          hand_mode:=right \
          controller_backend:=ros2_control \
          launch_plot:=false \
          read_touch_status:=true \
          launch_haptic:=true
    " >"${log}" 2>&1 &
    local p=$!
    echo "$p" >"${HAND_PID_FILE}"
    ok "hand pipeline docker-exec PID ${p} (log ${log})"
    sleep 2
    if ! [ -d "/proc/$p" ]; then
        bad "hand pipeline exited early; see ${log}"
        return 1
    fi
}

_start_haptic_standalone_if_needed() {
    # Prefer launch_haptic inside real_hand_pipeline. If the installed launch
    # does not yet know launch_haptic, fall back to a standalone node.
    if grep -q 'launch_haptic' "${WS_ROOT}/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_capabilities/manus_revo2_retarget/launch/real_hand_pipeline_launch.py" 2>/dev/null; then
        return 0
    fi
    step "Start tactile→Manus haptic bridge (standalone)"
    local log="${PID_DIR}/haptic_bridge.log"
    nohup docker exec -u 0 "${WORK_CONTAINER}" bash -lc "
        source /opt/ros/humble/setup.bash
        source ${MANUS_WS}/install/setup.bash
        exec ros2 run manus_revo2_retarget tactile_to_vibration --ros-args \
          -p hand_side:=right -p glove_index:=0
    " >"${log}" 2>&1 &
    echo $! >"${HAPTIC_PID_FILE}"
    ok "haptic bridge PID $!"
}

show_status() {
    step "Status"
    container_running && ok "container ${WORK_CONTAINER}" || bad "container down"
    [ "$(docker inspect -f '{{.State.Running}}' airbot-play-runtime 2>/dev/null)" = true ] \
        && ok "airbot-play-runtime" || warn "airbot-play-runtime not running"
    ss -ltn 2>/dev/null | grep -q ':9090' && ok "rosbridge :9090" || bad "rosbridge down"
    if p="$(_read_pid "${HAND_PID_FILE}")"; then ok "hand pipeline PID ${p}"; else bad "hand pipeline not tracked"; fi
    if p="$(_read_pid "${HAPTIC_PID_FILE}")"; then ok "haptic bridge PID ${p}"; fi
    if pgrep -f 'run_vive_teleop.py' >/dev/null 2>&1; then
        warn "run_vive_teleop.py already running"
    else
        printf '    run_vive_teleop.py not running\n'
    fi
}

stop_all() {
    step "Stop unified teleop helpers"
    _kill_pidfile "${HAND_PID_FILE}" "hand pipeline"
    _kill_pidfile "${HAPTIC_PID_FILE}" "haptic bridge"
    # Only stop rosbridge if we started it via this script.
    if [ -f "${ROSBRIDGE_PID_FILE}" ]; then
        _kill_pidfile "${ROSBRIDGE_PID_FILE}" "rosbridge"
    else
        warn "rosbridge PID not tracked by this script; leave :9090 alone"
    fi
}

CMD="${1:-start}"
ARM_FLAG="--real-arm"
case "${CMD}" in
    status) show_status; exit 0 ;;
    stop) stop_all; exit 0 ;;
    --dummy-arm) ARM_FLAG="--dummy-arm"; CMD=start ;;
    start) ;;
    *)
        if [ "${CMD}" = "--real-arm" ]; then
            ARM_FLAG="--real-arm"
            CMD=start
        else
            echo "Usage: $0 [start|stop|status|--real-arm|--dummy-arm]"
            exit 2
        fi
        ;;
esac

# Allow trailing flag after start
if [ "${1:-}" = "start" ] && [ "${2:-}" = "--dummy-arm" ]; then
    ARM_FLAG="--dummy-arm"
fi
if [ "${1:-}" = "start" ] && [ "${2:-}" = "--real-arm" ]; then
    ARM_FLAG="--real-arm"
fi

step "Prerequisites"
if ! container_running; then
    bad "Start the combined container first: ${WS_ROOT}/bringup.sh --no-build"
    exit 1
fi
ok "container ${WORK_CONTAINER}"

if [ "${ARM_FLAG}" = "--real-arm" ]; then
    if [ "$(docker inspect -f '{{.State.Running}}' airbot-play-runtime 2>/dev/null)" != true ]; then
        warn "airbot-play-runtime not up — run ${ROOT}/teleop_up.sh first for CAN/sidecar"
    else
        ok "airbot-play-runtime"
    fi
fi

_start_rosbridge || exit 1
_start_hand_pipeline || exit 1
_start_haptic_standalone_if_needed

step "Start VIVE teleop on host (${ARM_FLAG})"
printf '  Left pedal = arm deadman; right pedal = recording gate (/teleop/hand_recording)\n'
printf '  Ctrl-C stops VIVE only; then run: %s stop\n' "$0"

cleanup_on_exit() {
    printf '\n'
    warn "VIVE stopped. Hand pipeline / rosbridge still running — use: $0 stop"
}
trap cleanup_on_exit EXIT

cd "${VIVE_DIR}"
# Prefer host SteamVR OpenVR path; --real-arm needs airbot_py reaching :50051.
python3 scripts/run_vive_teleop.py ${ARM_FLAG}
exit_code=$?
trap - EXIT
exit "${exit_code}"
