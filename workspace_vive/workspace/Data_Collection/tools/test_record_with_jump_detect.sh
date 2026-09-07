#!/usr/bin/env bash
# test_record_with_jump_detect.sh — 带跳变检测的录制验证
#
# 启动顺序:
#   1. rosbridge + hal_state_publisher（已含跳变检测日志）
#   2. data_recorder_node（已含跳变检测日志）
#   3. teleop（需在另一个终端手动启动）
#
# 如果录制过程中出现跳变，日志中会打印 WARNING: *LEFT/RIGHT ARM JUMP*
#
# 用法:
#   终端1: ./test_record_with_jump_detect.sh rosbridge
#   终端2: ./test_record_with_jump_detect.sh recorder
#   终端3: 启动遥操作
#   终端4（可选）: ./test_record_with_jump_detect.sh diagnose

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
BASE_DIR="$(dirname "$WS_DIR")"

CMD="${1:-help}"

case "$CMD" in
  rosbridge)
    echo "=== 启动 rosbridge + hal_state_publisher (含跳变检测) ==="
    exec bash "${BASE_DIR}/start_rosbridge.sh"
    ;;
  recorder)
    echo "=== 启动 data_recorder_node (含跳变检测) ==="
    echo "  跳变阈值: 10°"
    echo "  日志关键词: LEFT ARM JUMP / RIGHT ARM JUMP"
    echo ""
    CONDA_BASE="${CONDA_BASE:-/home/dt/miniconda3}"
    source "$CONDA_BASE/etc/profile.d/conda.sh"
    conda activate record
    exec python3 "${SCRIPT_DIR}/data_recorder_node.py"
    ;;
  diagnose)
    echo "=== 启动 HAL 层诊断 (持续监听) ==="
    CONDA_BASE="${CONDA_BASE:-/home/dt/miniconda3}"
    source "$CONDA_BASE/etc/profile.d/conda.sh"
    conda activate record
    exec python3 "${SCRIPT_DIR}/diagnose_udp_state.py" --via-hal --duration 0
    ;;
  help|*)
    echo "用法: $0 {rosbridge|recorder|diagnose}"
    echo ""
    echo "  rosbridge  - 启动 rosbridge + hal_state_publisher"
    echo "  recorder   - 启动 data_recorder_node (含跳变检测)"
    echo "  diagnose   - 启动 HAL 层持续诊断"
    echo ""
    echo "验证步骤:"
    echo "  1. 终端1: $0 rosbridge"
    echo "  2. 终端2: $0 recorder"
    echo "  3. 终端3: 启动遥操作"
    echo "  4. 终端4(可选): $0 diagnose"
    echo "  5. 开始录制并操作机械臂"
    echo "  6. 观察终端2和终端1的日志:"
    echo "     - hal_state_publisher 报 JUMP → HAL→ROS2 层有问题"
    echo "     - data_recorder_node 报 JUMP → rosbridge 层有问题"
    echo "     - 两层都不报 JUMP → 问题在 SyncBuffer/写入层"
    echo "  7. 停止录制后检查 parquet 数据是否还有跳变"
    ;;
esac
