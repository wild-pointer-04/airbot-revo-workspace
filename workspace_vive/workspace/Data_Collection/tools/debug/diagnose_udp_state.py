#!/usr/bin/env python3
"""
diagnose_udp_state.py — 诊断 Apex UDP 状态推送

直接监听 Apex 控制器的 UDP 推送，打印每帧的 armId 和关节角，
用于排查数据跳变问题。

用法:
  1. 先关闭 HAL（因为 HAL 占用 UDP 9101 端口）
  2. python3 diagnose_udp_state.py
  3. 观察输出中同一 armId 的 joint[4] 是否稳定

  或者:
  1. 保持 HAL 运行
  2. python3 diagnose_udp_state.py --via-hal
  3. 通过 HAL client 的 get() 接口间接读取（不需要独占 UDP 端口）
"""

import sys
import time
import json
import socket
import argparse
import numpy as np
from pathlib import Path

_BASE = Path(__file__).resolve().parent.parent.parent
HAL_SRC = _BASE / "src" / "robot_hal" / "src"
APEX_SDK = _BASE / "src" / "robot_hal" / "third_party" / "apex_sdk" / "python"

sys.path.insert(0, str(HAL_SRC))
sys.path.insert(0, str(APEX_SDK))


def diagnose_via_hal(host, port, duration, hz):
    """通过 HAL client 读取状态（HAL 运行时使用）。"""
    from robot_hal.client import RobotHalClient

    client = RobotHalClient(host=host, port=port)
    client.connect()
    print(f"已连接 HAL {host}:{port}，监听中 @ {hz}Hz (Ctrl+C 停止)...\n")
    print(f"{'time':>6} {'arm':>4} {'j0':>8} {'j1':>8} {'j2':>8} {'j3':>8} {'j4':>8} {'j5':>8} {'j6':>8}  {'jump':>5}")

    prev_left = None
    prev_right = None
    interval = 1.0 / hz
    t0 = time.time()
    jump_count = 0

    try:
        sample_count = 0
        last_report = time.time()
        while True:
            if duration > 0 and time.time() - t0 >= duration:
                break

            state = client.get()
            elapsed = time.time() - t0
            sample_count += 1

            left = state.get("leftarm_state_joint_pos")
            right = state.get("rightarm_state_joint_pos")

            if left is not None:
                jump = ""
                if prev_left is not None:
                    max_d = np.abs(left - prev_left).max()
                    if max_d > 10:
                        jump = f"!{max_d:.0f}°"
                        jump_count += 1
                        vals = " ".join(f"{v:8.2f}" for v in left)
                        print(f"{elapsed:6.2f}   L  {vals}  {jump}")
                prev_left = left.copy()

            if right is not None:
                jump = ""
                if prev_right is not None:
                    max_d = np.abs(right - prev_right).max()
                    if max_d > 10:
                        jump = f"!{max_d:.0f}°"
                        jump_count += 1
                        vals = " ".join(f"{v:8.2f}" for v in right)
                        print(f"{elapsed:6.2f}   R  {vals}  {jump}")
                prev_right = right.copy()

            now = time.time()
            if now - last_report >= 5.0:
                print(f"  [{elapsed:.0f}s] 已采样 {sample_count} 帧, 跳变: {jump_count},"
                      f" L_j4={prev_left[4]:.2f} R_j4={prev_right[4]:.2f}")
                last_report = now

            time.sleep(interval)
    except KeyboardInterrupt:
        pass

    client.close()
    print(f"\n完成。共检测到 {jump_count} 次跳变 (>10°)")


def diagnose_udp_direct(duration, hz):
    """直接监听 UDP 9101（需要先关闭 HAL）。"""
    from apex_sdk.protocol import UDP_PORT

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.1)
    print(f"直接监听 UDP:{UDP_PORT}，持续 {duration}s...\n")
    print(f"{'time':>6} {'armId':>5} {'j0':>8} {'j1':>8} {'j2':>8} {'j3':>8} {'j4':>8} {'j5':>8} {'j6':>8}  {'jump':>5}")

    prev = {}  # armId -> prev_joints
    t0 = time.time()
    jump_count = 0

    while time.time() - t0 < duration:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue

        try:
            msg = json.loads(data.decode("utf-8").strip())
        except (json.JSONDecodeError, UnicodeDecodeError):
            continue

        if msg.get("type") != "status-push":
            continue

        arm_id = msg.get("armId", 0)
        js = msg.get("jointState", {})
        positions = js.get("positions", [])
        if not positions:
            continue

        joints = np.array(positions)
        elapsed = time.time() - t0
        arm_label = "L" if arm_id == 0 else "R"

        jump = ""
        if arm_id in prev:
            max_d = np.abs(joints - prev[arm_id]).max()
            if max_d > 10:
                jump = f"!{max_d:.0f}°"
                jump_count += 1
        prev[arm_id] = joints.copy()

        vals = " ".join(f"{v:8.2f}" for v in joints)
        print(f"{elapsed:6.2f}   {arm_label}  {vals}  {jump}")

    sock.close()
    print(f"\n完成。共检测到 {jump_count} 次跳变 (>10°)")


def main():
    parser = argparse.ArgumentParser(description="诊断 Apex UDP 状态推送")
    parser.add_argument("--via-hal", action="store_true",
                        help="通过 HAL client 读取（HAL 运行时使用）")
    parser.add_argument("--host", default="127.0.0.1", help="HAL host")
    parser.add_argument("--port", type=int, default=7860, help="HAL port")
    parser.add_argument("--duration", "-d", type=float, default=0,
                        help="监听时长(秒)，0=无限")
    parser.add_argument("--hz", type=float, default=30.0,
                        help="采样频率(仅 via-hal 模式)")
    args = parser.parse_args()

    if args.via_hal:
        diagnose_via_hal(args.host, args.port, args.duration, args.hz)
    else:
        diagnose_udp_direct(args.duration, args.hz)


if __name__ == "__main__":
    main()
