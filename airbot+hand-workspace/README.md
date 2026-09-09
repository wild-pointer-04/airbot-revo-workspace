# AIRBOT Play 臂（PICO / VIVE 遥操 + 数采）

合工作区里的臂侧：VR 追踪 → Python 中间层 → 官方 sidecar gRPC → CAN → AIRBOT Play 6 轴。  
**臂控制链路不走 ROS topic。** 末端夹爪在配置里是 `enable_eef: false`，握把命令会被丢掉；灵巧手走旁边的 Revo2 链路。

两套遥操输入：**PICO**（`EE_controlhal/middle_interface`，头显 UDP）和 **VIVE**（`VIVE_control_openvr` = 宿主机 SteamVR/OpenVR）。数采在 `Data_Collection/`，经 ROS topic `/arm/state` 拿臂状态 —— 这是唯一用到 ROS 的地方。

合容器总启动顺序见上一级 [README.md](../README.md)。

## 结构

```text
airbot+hand-workspace/
  README.md                      本文件
  teleop_up.sh                   臂一键：CAN → 合容器 → sidecar → adb → 只读自检
  docker/
    airbot_up.sh                 拉起/停止 sidecar airbot-play-runtime
    compose.yaml                 旧 Jazzy 工作容器（已不用）
  EE_controlhal/
    PICO_control/                Unity APK（PicoTeleop）
    middle_interface/            UDP :15000 → 遥操流水线 → 命令层
      app.py
      config/airbot.yaml         真机
      config/default.yaml        dummy 空跑
    VIVE_control_openvr/         SteamVR/OpenVR 双 tracker（宿主机）
      scripts/run_vive_teleop.py
      config/tracker.yaml / teleop.yaml
    Robot_control_interface/
      backends/airbot_sdk.py     airbot_py → localhost:50051
      config/airbot_sdk.yaml
      state_pub.py               发布 /arm/state 给数采
  Data_Collection/               图像 + 臂状态 → LeRobot（见其 README）
  Airbot_play/                   SDK wheel / 文档（C++ deb 不进 Humble 镜像）
  ros2_play/                     臂 URDF（可视化）
  revo2_description/             仅 URDF 占位，真机手不在这里
```

```text
PICO USB + adb reverse tcp:15000
  → middle_interface/app.py          扳机 deadman，grip 原为夹爪（现忽略）
  → Robot_control_interface/airbot_sdk
  → gRPC localhost:50051
  → 容器 airbot-play-runtime (fsm_node)
  → can0
  → AIRBOT Play
```

容器内工作目录：`/home/ubuntu/airbot+hand-workspace`  
Python：合容器 PATH 里的 `/opt/airbot-venv`（`airbot_py 5.1.6`）。

## 启动顺序

在合容器已起来的前提下（`../bringup.sh`）。

1. **USB2CAN 插上，确认 `can0` UP**
   ```bash
   ip -br link show can0
   ```
2. **头显 USB 调试 + `adb reverse`（USB 不通最常见就是漏这一步）**
   PicoTeleop 往头显自己的 `127.0.0.1:15000` 发 TCP，必须转到电脑：
   ```bash
   adb devices
   adb reverse tcp:15000 tcp:15000
   adb reverse --list    # 应看到 UsbFfs tcp:15000 tcp:15000
   ```
   `teleop_up.sh` 会做这件事；头显重插/重启后转发会掉，要再跑一次。
3. **一键依赖（占宿主机终端，会只读连一次臂）**
   ```bash
   /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh
   ```
   看机械臂灯条：**绿色呼吸**才能继续。白灯 = 驱动没接管；黄灯 = 要校零。不对就 `./teleop_up.sh restart`。
4. **头显里开 PicoTeleop**，面板 `127.0.0.1:15000`。可不戴，当追踪基站：
   ```bash
   adb shell am start -n com.egoarx.picoteleopue/com.unity3d.player.UnityPlayerActivity
   ```
5. **遥操进程（必须占着终端看日志）**  
   `docker exec` 必须写成：`选项 + 容器名 + 命令`（只写 `-w` 会报 `requires at least 2 arguments`）。
   ```bash
   docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
     airbot-revo-humble python3 app.py --config config/airbot.yaml
   ```
   空跑（臂不动）把 `airbot.yaml` 换成 `default.yaml`。

日志应对上：`ROBOT ADAPTER: REAL (COMMAND_LAYER)`、`[PICO] RX` 约 60 Hz、`Deadman armed after release`。

操作：右食指扳机按住跟随、松开停住；右 A 回零位；左 X 暂停。退出用 **Ctrl-C**。

```bash
./teleop_up.sh status
./teleop_up.sh stop          # 先停 app.py 再停 sidecar
./teleop_up.sh restart
```

## VIVE 遥操（替代 PICO 的输入）

不需要头显和 adb。SteamVR 必须在**宿主机**跑（不要装进容器）。前几步（CAN、`teleop_up.sh`）和 PICO 一样。

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/start_steamvr.sh
./scripts/close_room_setup.sh
python3 scripts/list_trackers.py
python3 scripts/run_vive_teleop.py              # 默认 dummy
# python3 scripts/run_vive_teleop.py --real-arm # 需 airbot_py + :50051
```

Null Driver 下的 Error 496（Failed to connect to headset display）可忽略。详见 [VIVE_control_openvr/README.md](EE_controlhal/VIVE_control_openvr/README.md)。

坐标系：`vr_space` — PICO=`unity`，OpenVR=`steamvr`。

## 数采

图像 + 臂关节角/末端 → LeRobot（parquet + MP4）。臂状态由遥操侧发 `/arm/state`，采集器经 rosbridge 订阅。

采集器跑在**宿主机自己的 Python 3.12 环境**里，不在合容器内 —— 因为 `vendor/lerobot-main` 要求 Python ≥ 3.12，而 Humble 只有 3.10。

```bash
# 1) 容器内起 rosbridge（常驻，一次即可）
docker exec -d -u 0 airbot-revo-humble \
  bash -lc 'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml'

# 2) 起遥操（上面 PICO 或 VIVE 任一），它会发布 /arm/state

# 3) 宿主机开录
cd Data_Collection && .venv-collect/bin/python collect/collect.py --task pick_cup
```

细节和字段说明见 [Data_Collection/README.md](Data_Collection/README.md)。
