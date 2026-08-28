#!/bin/bash
# PICO + AIRBOT Play 遥操作一键启动。
#
# 按依赖顺序拉起：CAN 检查 → 工作容器 → AIRBOT 驱动 → adb 转发 → 自检。
# 全部就绪后提示手动启动 app.py（它要占着终端看日志）。
#
# 用法：
#   ./teleop_up.sh              # 全部启动 + 自检
#   ./teleop_up.sh stop         # 停驱动（工作容器保留）
#   ./teleop_up.sh status       # 只看状态，不改任何东西
#   ./teleop_up.sh restart      # 干净重启驱动（先杀 app.py 再重启）
set -uo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
MID="${ROOT}/EE_controlhal/middle_interface"
UNITY="${ROOT}/EE_controlhal/PICO_control/pico_teleop_unity"
# 合容器：默认 airbot-revo-humble。旧 Jazzy 容器已移除。
WORK_CONTAINER="${WORK_CONTAINER:-airbot-revo-humble}"
COMBINED_CONTAINER="airbot-revo-humble"
COMBINED_BRINGUP="${COMBINED_BRINGUP:-/home/ubuntu/airbot-revo-workspace/manus-Revo2-Revo3/bringup_combined.sh}"
export AIRBOT_WORKSPACE="${AIRBOT_WORKSPACE:-${ROOT}}"
DRIVER_CONTAINER="airbot-play-runtime"
CAN_IFACE="can0"
PORT=50051
UDP_PORT=15000
APK_PKG="com.egoarx.picoteleopue"

ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$*"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }
step() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

# app.py 必须先停：SDK 客户端连着时重启驱动会留下悬挂的 gRPC 会话，
# 新驱动起来后 controller_manager 的接口状态就是脏的，切模式必然失败。
#
# 容器是 PidMode=host，容器内 pgrep 能看到宿主机全部进程 —— 包括 docker exec
# 自己那条含 "app.py" 的命令行。所以用 pgrep 会咬到自己，必须按可执行文件名
# 匹配（-x python3）再筛真正的脚本参数。
app_pids() {
    docker exec "${WORK_CONTAINER}" bash -lc \
        'ps -eo pid=,comm=,args= | awk '"'"'$2 ~ /^python3(\.[0-9]+)?$/ && $0 ~ /app\.py --config/ {print $1}'"'"'' \
        2>/dev/null | tr -d '\r'
}

kill_app() {
    local pids
    pids="$(app_pids)"
    if [ -z "${pids}" ]; then
        return 0
    fi
    warn "检测到 app.py 在跑（PID ${pids//$'\n'/ }），先停掉（避免驱动接口状态被污染）"
    for p in ${pids}; do
        kill -INT "${p}" 2>/dev/null || true
    done
    for _ in $(seq 1 10); do
        [ -z "$(app_pids)" ] && break
        sleep 1
    done
    pids="$(app_pids)"
    if [ -n "${pids}" ]; then
        warn "Ctrl-C 没停下来，强制结束"
        for p in ${pids}; do kill -KILL "${p}" 2>/dev/null || true; done
        sleep 1
    fi
    ok "app.py 已停"
}

show_status() {
    step "状态"
    ip -br link show "${CAN_IFACE}" 2>/dev/null | grep -q UP \
        && ok "CAN ${CAN_IFACE} UP" || bad "CAN ${CAN_IFACE} 不可用"
    [ "$(docker inspect -f '{{.State.Running}}' "${WORK_CONTAINER}" 2>/dev/null)" = true ] \
        && ok "工作容器运行中" || bad "工作容器未运行"
    [ "$(docker inspect -f '{{.State.Running}}' "${DRIVER_CONTAINER}" 2>/dev/null)" = true ] \
        && ok "AIRBOT 驱动运行中" || bad "AIRBOT 驱动未运行"
    ss -ltn 2>/dev/null | grep -q ":${PORT}" \
        && ok "gRPC ${PORT} 监听中" || bad "gRPC ${PORT} 无监听"
    adb devices 2>/dev/null | sed -n '2p' | grep -q device \
        && ok "头显已连接" || bad "头显未连接（USB 调试模式可能又退回默认了）"
    adb reverse --list 2>/dev/null | grep -q "tcp:${UDP_PORT}" \
        && ok "adb 转发 ${UDP_PORT} 已建立" || bad "adb 转发未建立"
    if [ -n "$(app_pids)" ]; then
        warn "app.py 正在运行"
    else
        printf '    app.py 未运行\n'
    fi
}

case "${1:-start}" in
    status) show_status; exit 0 ;;
    stop)
        kill_app
        step "停止 AIRBOT 驱动"
        bash "${ROOT}/docker/airbot_up.sh" stop
        exit 0
        ;;
    restart)
        kill_app
        step "重启 AIRBOT 驱动"
        bash "${ROOT}/docker/airbot_up.sh" stop >/dev/null 2>&1 || true
        sleep 5
        ;;
    start) ;;
    *) echo "用法: $0 [start|stop|status|restart]"; exit 2 ;;
esac

# ---------- 1. CAN ----------
step "1/5 CAN 总线"
if ! ip -br link show "${CAN_IFACE}" 2>/dev/null | grep -q UP; then
    bad "${CAN_IFACE} 不存在或未 UP"
    lsusb | grep -qi "USB to CAN" \
        && warn "USB2CAN 适配器在，但 ${CAN_IFACE} 没起来 —— 检查 udev / 重插适配器" \
        || warn "没检测到 USB2CAN 适配器 —— 检查 USB 连接"
    exit 1
fi
ok "${CAN_IFACE} UP"

# ---------- 2. 工作容器 ----------
step "2/5 工作容器"
state="$(docker inspect -f '{{.State.Status}}' "${WORK_CONTAINER}" 2>/dev/null || true)"
if [ "${state}" = running ]; then
    ok "已在运行"
elif [ -n "${state}" ]; then
    docker start "${WORK_CONTAINER}" >/dev/null && ok "已启动（原为 ${state}）"
else
    if [ "${WORK_CONTAINER}" = "${COMBINED_CONTAINER}" ]; then
        warn "合容器不存在，请先跑: ${COMBINED_BRINGUP}"
        exit 1
    fi
    warn "容器不存在，用 compose 创建"
    docker compose -f "${ROOT}/docker/compose.yaml" up -d >/dev/null 2>&1 \
        && ok "已创建并启动" || { bad "创建失败，手动跑: docker compose -f docker/compose.yaml up -d"; exit 1; }
fi

# ---------- 3. AIRBOT 驱动 ----------
step "3/5 AIRBOT 驱动"
kill_app
if [ "$(docker inspect -f '{{.State.Running}}' "${DRIVER_CONTAINER}" 2>/dev/null)" = true ]; then
    ok "已在运行（要干净重启用: $0 restart）"
else
    bash "${ROOT}/docker/airbot_up.sh" start --can "${CAN_IFACE}" || exit 1
    printf '    等待 gRPC 端口 '
    for _ in $(seq 1 30); do
        ss -ltn 2>/dev/null | grep -q ":${PORT}" && break
        printf '.'; sleep 1
    done
    printf '\n'
    # 端口亮起后驱动还要初始化控制器和 CAN 握手，这段时间连接会超时。
    printf '    等待驱动初始化（约 15 秒）'
    for _ in $(seq 1 15); do printf '.'; sleep 1; done
    printf '\n'
fi
ss -ltn 2>/dev/null | grep -q ":${PORT}" \
    && ok "gRPC ${PORT} 监听中" || { bad "gRPC ${PORT} 起不来，看日志: docker logs ${DRIVER_CONTAINER}"; exit 1; }

# ---------- 4. PICO ----------
step "4/5 PICO 头显"
if ! command -v adb >/dev/null 2>&1; then
    bad "宿主机没有 adb: sudo apt-get install -y adb"
    exit 1
fi
adb start-server >/dev/null 2>&1
if ! adb devices 2>/dev/null | sed -n '2p' | grep -q device; then
    bad "adb 看不到头显"
    if lsusb | grep -qi "Pico\|PICO 4"; then
        warn "USB 上有头显，但没暴露 adb 接口 —— 头显的 USB 模式退回默认了"
        printf '      去头显里：设置 → 开发者 → 打开 USB 调试，\n'
        printf '      并把 USB 连接方式改成「调试」（不是传输文件/仅充电），\n'
        printf '      然后重插线、在头显里点「允许 USB 调试」。\n'
    else
        warn "USB 上没看到头显 —— 检查数据线（不能是只供电的线）"
    fi
    exit 1
fi
ok "头显已连接: $(adb devices | sed -n '2p' | awk '{print $1}')"
adb shell pm list packages 2>/dev/null | grep -q "${APK_PKG}" \
    && ok "PicoTeleop 已安装" \
    || { warn "PicoTeleop 未安装，正在安装"; adb install -r "${UNITY}/Builds/PicoTeleopUE.apk" >/dev/null 2>&1 \
         && ok "安装完成" || bad "安装失败"; }
# adb reverse 只转 TCP，头显里填 127.0.0.1 走 loopback，发送端会自动切 TCP。
# 头显重插或重启后这条会掉，所以每次都重建。
adb reverse --remove-all >/dev/null 2>&1 || true
adb reverse "tcp:${UDP_PORT}" "tcp:${UDP_PORT}" >/dev/null 2>&1 \
    && ok "adb 转发 ${UDP_PORT} 已建立" || { bad "建转发失败"; exit 1; }

# ---------- 5. 自检 ----------
step "5/5 机械臂自检（只读 + 一次模式切换）"
# gRPC 端口一出现并不代表驱动初始化完了：刚启动时握手可能要几十秒（SDK 会
# 内部超时并报 Failed to connect）。所以这里重试，而不是一次失败就判死。
selfcheck() {
    docker exec "${WORK_CONTAINER}" bash -lc 'python3 - <<PYEOF 2>/dev/null
import math, sys
from airbot_py.arm import AIRBOTPlay, RobotMode
r = AIRBOTPlay(url="localhost", port=50051)
if not r.connect():
    sys.exit(1)
print("  关节角(度): " + str([round(math.degrees(x),1) for x in r.get_joint_pos()]))
print("  末端位置: " + str([round(x,4) for x in r.get_end_pose()[0]]))
m = r.switch_mode(RobotMode.SERVO_CART_POSE)
print("  进入 SERVO_CART_POSE: " + str(m))
r.disconnect()
sys.exit(0 if m else 2)
PYEOF' 2>&1 | grep -v "^Client ID\|^ERROR:root"
    return "${PIPESTATUS[0]}"
}

checked=0
for attempt in 1 2 3 4 5 6; do
    if out="$(selfcheck)"; then
        printf '%s\n' "${out}"
        checked=1
        break
    fi
    [ "${attempt}" = 1 ] && printf '    驱动还在初始化，重试 '
    printf '.'
    sleep 5
done
[ "${attempt}" != 1 ] && printf '\n'
if [ "${checked}" != 1 ]; then
    bad "自检失败 —— 驱动内部状态可能错乱，试 $0 restart"
    printf '    也看一眼灯条：白色=驱动没接管，黄色=零点丢失需校准\n'
    exit 1
fi
ok "自检通过"

cat <<TXT

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
全部就绪。接下来两件事需要你手动做：

【1】看机械臂灯条 —— 必须是「绿色呼吸」
     绿色呼吸 = 驱动已接管，可以遥操作
     白色呼吸 = 驱动没接管，机械臂不会动（这时跑 app.py 也没用）
     黄色呼吸 = 零点丢失，要先做零点校准
     灯不绿就别往下走，先跑: ./teleop_up.sh restart

【2】头显里启动 PicoTeleop，确认面板显示 127.0.0.1:15000
     用 adb 拉起（省得在头显里翻菜单）：
       adb shell am start -n com.egoarx.picoteleopue/com.unity3d.player.UnityPlayerActivity
     头显挂脖子上或架桌上朝前，别戴脸上 —— 它只当追踪基站，画面是黑的正常。

然后启动遥操作（要占着终端看日志，所以不放进本脚本）：

  docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
    ${WORK_CONTAINER} python3 app.py --config config/airbot.yaml

  先空跑验证就把 airbot.yaml 换成 default.yaml（dummy 后端，机械臂不动）

日志里要看到：
  ROBOT ADAPTER: REAL (COMMAND_LAYER)      ← 真机（DUMMY 是空跑）
  [PICO] RX: 60.0 Hz  loss=0  state=READY  ← 包在进
  [TELEOP] Deadman armed after release     ← 松一次扳机才武装

操作：右食指扳机按住=跟随、松开=停住；右A=回零位（真实运动）；左X=暂停
退出务必 Ctrl-C（走 stop+disconnect 清理，直接杀进程会污染驱动接口状态）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TXT
