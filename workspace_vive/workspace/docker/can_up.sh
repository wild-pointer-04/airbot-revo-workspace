#!/bin/bash
# 容器内拉起 CAN。优先用 udev 绑定的 /dev/arxcanN，否则回退到 /dev/ttyACM*。
set -euo pipefail

start_one() {
    local dev="$1"
    local iface="$2"
    if [ ! -e "${dev}" ]; then
        echo "跳过 ${iface}: 找不到 ${dev}"
        return 0
    fi
    echo "启动 ${dev} -> ${iface}"
    sudo pkill -f "slcand .* ${iface}$" >/dev/null 2>&1 || true
    sudo ip link set "${iface}" down >/dev/null 2>&1 || true
    sudo slcand -o -f -s8 "${dev}" "${iface}"
    sudo ip link set "${iface}" up
    ip -details link show "${iface}"
}

start_one /dev/arxcan1 can1
start_one /dev/arxcan3 can3
start_one /dev/arxcan6 can6

if ! ip link show can1 >/dev/null 2>&1; then
    if [ -e /dev/ttyACM0 ]; then
        echo "未找到 /dev/arxcan1，使用 /dev/ttyACM0 作为 can1"
        start_one /dev/ttyACM0 can1
    fi
fi
