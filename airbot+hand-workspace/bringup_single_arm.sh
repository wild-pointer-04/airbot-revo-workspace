#!/bin/bash
# 单从臂调通：拉起 can1，启动 X5Controller（无夹爪）。
# 用法：
#   ./bringup_single_arm.sh           # 只启动控制器，不主动运动
#   ./bringup_single_arm.sh --build    # 先 colcon build 再启动
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
WS="${ROOT}/ARX/ARX_X5-main/ROS2/X5_ws"
CAN_DEV=""
CAN_IFACE="can1"

if [ -e /dev/arxcan1 ]; then
    CAN_DEV="/dev/arxcan1"
elif [ -e /dev/ttyACM0 ]; then
    CAN_DEV="/dev/ttyACM0"
else
    echo "找不到 USB2CAN（/dev/arxcan1 或 /dev/ttyACM0）。请检查接线。"
    exit 1
fi

echo "==> CAN: ${CAN_DEV} -> ${CAN_IFACE}"
sudo pkill -f "slcand .* ${CAN_IFACE}$" >/dev/null 2>&1 || true
sudo ip link set "${CAN_IFACE}" down >/dev/null 2>&1 || true
sudo slcand -o -f -s8 "${CAN_DEV}" "${CAN_IFACE}"
sudo ip link set "${CAN_IFACE}" up type can bitrate 1000000 2>/dev/null || sudo ip link set "${CAN_IFACE}" up
ip -details link show "${CAN_IFACE}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

if [ "${1:-}" = "--build" ] || [ ! -f "${WS}/install/setup.bash" ]; then
    echo "==> colcon build"
    cd "${WS}"
    colcon build --symlink-install
fi

set +u
source "${WS}/install/setup.bash"
set -u
cd "${WS}"

echo "==> 启动单从臂控制器（normal 模式，不发运动指令）"
echo "    看关节： ros2 topic echo /arm_status --once"
echo "    若 6 轴全是 0，检查机械臂供电，或把 single_arm.yaml 里 arm_end_type 改成 2 再试"
exec ros2 launch arx_x5_controller open_single_arm.launch.py
