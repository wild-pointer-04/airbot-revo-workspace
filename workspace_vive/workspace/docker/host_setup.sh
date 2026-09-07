#!/bin/bash
# 宿主机一次性权限：docker 组、设备组、udev、compose 插件。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TARGET_USER="${SUDO_USER:-$USER}"

if [ "$(id -u)" -ne 0 ]; then
    echo "需要 root，改用: sudo $0"
    exit 1
fi

echo "==> 用户组: docker/dialout/plugdev/video/render/input"
usermod -aG docker,dialout,plugdev,video,render,input "${TARGET_USER}"

echo "==> 安装 udev 规则"
install -m 644 "${SCRIPT_DIR}/99-ego-arx.rules" /etc/udev/rules.d/99-ego-arx.rules
udevadm control --reload-rules
udevadm trigger

echo "==> 当前串口权限"
shopt -s nullglob
serial_devs=(/dev/ttyACM* /dev/ttyUSB*)
if [ ${#serial_devs[@]} -gt 0 ]; then
    chmod 666 "${serial_devs[@]}" || true
    ls -l "${serial_devs[@]}"
else
    echo "当前没有 ttyACM/ttyUSB"
fi
shopt -u nullglob

echo "==> 安装 docker compose 插件"
apt-get update -qq
apt-get install -y docker-compose-v2 >/dev/null

echo "完成。新组要重新登录后才对当前桌面会话生效。"
echo "验证: groups ${TARGET_USER}"
echo "      docker compose version"
