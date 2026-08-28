#!/bin/bash
# 拉起 Humble 合容器：AIRBOT 臂客户端 + Revo2 手栈并行。
# 官方 CAN 驱动仍是 sidecar airbot-play-runtime。
#
# 用法：
#   ./bringup_combined.sh              # 构建（如需要）并启动工作容器
#   ./bringup_combined.sh --stop-old   # 先停旧工作容器再启动
#   ./bringup_combined.sh --no-build   # 不重建镜像
#   ./bringup_combined.sh stop         # 停合容器（AIRBOT sidecar 保留）
#   ./bringup_combined.sh status
set -uo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
MERGED_ROOT="${MERGED_ROOT:-/home/ubuntu/airbot-revo-workspace}"
AIRBOT_WS="${HOST_AIRBOT_WS:-${MERGED_ROOT}/airbot+hand-workspace}"
MANUS_WS="${HOST_MANUS_WS:-${MERGED_ROOT}/manus-Revo2-Revo3}"
# 容器内路径保持不变；上面是宿主机合并目录。
CONTAINER_AIRBOT_WS="/home/ubuntu/airbot+hand-workspace"
CONTAINER_MANUS_WS="/home/dt/manus-Revo2-Revo3"
IMAGE="airbot-revo:humble"
NAME="airbot-revo-humble"
OLD_ARM="airbot-hand-workspace"
OLD_HAND="manus-revo-humble"
CAN_IFACE="${CAN_IFACE:-can0}"
COMPOSE_FILE="${ROOT}/docker-compose.combined.yml"
DOCKERFILE="${ROOT}/Dockerfile.combined"

STOP_OLD=0
DO_BUILD=1
ACTION=start

ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$*"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }
step() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

usage() {
    sed -n '2,12p' "$0" | sed 's/^# \?//'
}

have_compose() {
    docker compose version >/dev/null 2>&1 && return 0
    command -v docker-compose >/dev/null 2>&1 && return 0
    return 1
}

compose_up() {
    if docker compose version >/dev/null 2>&1; then
        docker compose -f "${COMPOSE_FILE}" up -d "$@"
    else
        docker-compose -f "${COMPOSE_FILE}" up -d "$@"
    fi
}

container_status() {
    docker inspect -f '{{.State.Status}}' "$1" 2>/dev/null || true
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        start|stop|status) ACTION="$1"; shift ;;
        --stop-old) STOP_OLD=1; shift ;;
        --no-build) DO_BUILD=0; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

case "${ACTION}" in
    status)
        step "状态"
        [ "$(container_status "${NAME}")" = running ] \
            && ok "工作容器 ${NAME} 运行中" || bad "工作容器 ${NAME} 未运行"
        [ "$(container_status airbot-play-runtime)" = running ] \
            && ok "AIRBOT 驱动 sidecar 运行中" || warn "AIRBOT 驱动 sidecar 未运行"
        [ "$(container_status "${OLD_ARM}")" = running ] \
            && warn "旧臂容器 ${OLD_ARM} 仍在运行（会抢设备）"
        [ "$(container_status "${OLD_HAND}")" = running ] \
            && warn "旧手容器 ${OLD_HAND} 仍在运行（会抢设备）"
        exit 0
        ;;
    stop)
        step "停止合容器 ${NAME}"
        if docker inspect "${NAME}" >/dev/null 2>&1; then
            docker stop "${NAME}" >/dev/null && ok "已停止" || bad "停止失败"
        else
            warn "容器不存在"
        fi
        exit 0
        ;;
esac

if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || true
fi

step "1/4 旧工作容器"
for old in "${OLD_ARM}" "${OLD_HAND}"; do
    if [ "$(container_status "${old}")" = running ]; then
        if [ "${STOP_OLD}" = 1 ]; then
            docker stop "${old}" >/dev/null && ok "已停止 ${old}" || bad "停止 ${old} 失败"
        else
            warn "${old} 正在运行，会和合容器抢 /dev、CAN、gRPC、UDP"
            printf '      先停掉: docker stop %s\n' "${old}"
            printf '      或: %s --stop-old\n' "$0"
        fi
    else
        ok "${old} 未运行"
    fi
done

step "2/4 镜像 ${IMAGE}"
if [ "${DO_BUILD}" = 1 ] || ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    if ! docker image inspect manus-revo:humble >/dev/null 2>&1; then
        bad "缺少底镜像 manus-revo:humble，先构建手侧镜像"
        printf '      cd %s && docker build -t manus-revo:humble -f Dockerfile .\n' "${ROOT}"
        exit 1
    fi
    docker build -t "${IMAGE}" -f "${DOCKERFILE}" "${ROOT}" || exit 1
    ok "镜像已构建"
else
    ok "跳过构建（--no-build）"
fi

step "3/4 工作容器 ${NAME}"
state="$(container_status "${NAME}")"
if [ "${state}" = running ]; then
    ok "已在运行"
elif [ -n "${state}" ]; then
    docker start "${NAME}" >/dev/null && ok "已启动（原为 ${state}）" || exit 1
else
    if have_compose; then
        compose_up || exit 1
        ok "compose 已创建并启动"
    else
        warn "没有 docker compose 插件，用 docker run 等价启动"
        docker run -d \
            --name "${NAME}" \
            --hostname airbot-revo \
            --user 0:0 \
            --network host \
            --privileged \
            --ipc host \
            --pid host \
            --security-opt seccomp=unconfined \
            --security-opt apparmor=unconfined \
            --cap-add NET_ADMIN \
            --cap-add NET_RAW \
            --cap-add SYS_ADMIN \
            --cap-add SYS_MODULE \
            -e "DISPLAY=${DISPLAY:-:1}" \
            -e QT_X11_NO_MITSHM=1 \
            -e ROS_DOMAIN_ID=100 \
            -e ROS_DISTRO=humble \
            -e HOME=/root \
            -e USER=root \
            -e "AIRBOT_WORKSPACE=${CONTAINER_AIRBOT_WS}" \
            -e AIRBOT_RUNTIME_IMAGE=registry.cn-shanghai.aliyuncs.com/discover-robotics/airbot-runtime:5.1.6 \
            -v "${MANUS_WS}:${CONTAINER_MANUS_WS}" \
            -v "${AIRBOT_WS}:${CONTAINER_AIRBOT_WS}" \
            -v /var/run/docker.sock:/var/run/docker.sock \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v /etc/localtime:/etc/localtime:ro \
            -v /dev:/dev \
            -v /run/udev:/run/udev:ro \
            -v /sys:/sys:ro \
            -w /home/dt/manus-Revo2-Revo3 \
            "${IMAGE}" \
            sleep infinity || exit 1
        ok "docker run 已创建并启动"
    fi
fi

step "4/4 AIRBOT 驱动 sidecar"
if [ "$(container_status airbot-play-runtime)" = running ]; then
    ok "airbot-play-runtime 已在运行"
elif ip -br link show "${CAN_IFACE}" 2>/dev/null | grep -q UP; then
    bash "${AIRBOT_WS}/docker/airbot_up.sh" start --can "${CAN_IFACE}" \
        && ok "已启动 sidecar" \
        || warn "sidecar 启动失败（没有臂也可以先跑手）"
else
    warn "${CAN_IFACE} 未 UP，跳过 AIRBOT 驱动。有 CAN 后再跑:"
    printf '      %s/docker/airbot_up.sh start --can %s\n' "${AIRBOT_WS}" "${CAN_IFACE}"
fi

cat <<TXT

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
合容器已就绪：${NAME}（ROS 2 Humble）
两套控制链独立，请开两个终端。

【臂】PICO → AIRBOT（工作容器名要用合容器）

  WORK_CONTAINER=${NAME} ${AIRBOT_WS}/teleop_up.sh

  docker exec -it -w ${CONTAINER_AIRBOT_WS}/EE_controlhal/middle_interface \\
    ${NAME} python3 app.py --config config/airbot.yaml

【手】MANUS → Revo2（必须 root，USB 权限）

  docker exec -it -u 0 ${NAME} bash
  source /opt/ros/humble/setup.bash
  source /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/install/setup.bash
  ros2 launch manus_revo2_retarget pipeline_launch.py \\
    hand_mode:=both launch_manus_publisher:=true controller_backend:=ros2_control

不要同时再跑 ${OLD_ARM} 或 ${OLD_HAND}。
AIRBOT 官方驱动容器 airbot-play-runtime 是 sidecar，不是第二套业务容器。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TXT
