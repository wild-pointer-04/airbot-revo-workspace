# AIRBOT 臂 + Revo2 手 合工作区

VIVE / PICO 遥操 **AIRBOT Play** 机械臂，MANUS 遥操 **BrainCo Revo2** 灵巧手；数采把 **图像 + 臂 + 手** 写入同一 LeRobot episode。

- **臂控制**：宿主机或容器内遥操进程 → gRPC `:50051` → sidecar → CAN（**不走 ROS topic 控臂**）
- **手控制**：容器内 ROS 2（MANUS → retarget → `revo2_driver`）
- **数采**：宿主机 Python 3.12（`.venv-collect`），经 rosbridge 取臂/手状态，本地读 USB 相机

离线镜像：`/home/ubuntu/airbot-revo-humble.tar.gz`  
加载：`docker load -i /home/ubuntu/airbot-revo-humble.tar.gz`

---

## 整体结构

```text
airbot-revo-workspace/
  README.md                          本文件（总览 + 从遥操到数采）
  bringup.sh                         拉起合容器 airbot-revo-humble
  airbot+hand-workspace/             臂遥操 + 数采
    teleop_up.sh                     臂：CAN 检查 → sidecar →（PICO 时）adb
    scripts/teleop_vive_hand_up.sh   推荐：rosbridge + 右手 + VIVE 一键
    EE_controlhal/
      VIVE_control_openvr/           SteamVR/OpenVR 双 tracker（宿主机）
      middle_interface/              遥操管线；PICO 入口 app.py（容器内）
      Robot_control_interface/       airbot_py → :50051；发 /arm/state
      PICO_control/                  PicoTeleop APK
    Data_Collection/
      collect/                       统一数采（图+臂+手）← 唯一入口
      collect/config/collect.yaml
      data/lerobot/                  落盘（gitignore）
      .venv-collect/                 Python 3.12 采集环境
      vendor/lerobot-main/
    docker/airbot_up.sh              臂 sidecar airbot-play-runtime
  manus-Revo2-Revo3/                 手：MANUS → Revo2
    bringup_combined.sh
    Revo-Retargeting-revo2_retargeting/
      …/manus_revo2_retarget/        retarget + tactile_to_vibration
      …/revo2_driver/                真机手驱动
```

| 宿主机路径 | 容器内路径 |
|------------|------------|
| `…/airbot+hand-workspace` | `/home/ubuntu/airbot+hand-workspace` |
| `…/manus-Revo2-Revo3` | `/home/dt/manus-Revo2-Revo3` |

| 容器 | 作用 | 在哪跑 |
|------|------|--------|
| `airbot-revo-humble` | Humble + 两套源码 + rosbridge / 手 pipeline | 手、rosbridge、PICO 遥操 |
| `airbot-play-runtime` | 臂官方驱动，gRPC `localhost:50051`，管 `can0` | 由 `teleop_up.sh` 拉起 |

---

## 端到端数据流

```text
宿主机 SteamVR
  → VIVE_control_openvr/run_vive_teleop.py
       ├─ gRPC :50051 → airbot-play-runtime → can0 → AIRBOT 臂
       └─ /arm/state + /teleop/hand_recording ──┐
                                                │
MANUS USB ──► 容器 manus_data_publisher          │
              → manus_revo2_retarget             │
              → revo2_driver → Revo2 右手        │
              → tactile_to_vibration             │
                 → /manus_glove_*/vibration_cmd  │
              joint_states / dynamic_joint_states┤
                                                ▼
                              rosbridge :9090
                                                │
宿主机 USB 全局相机（by-id → LRCP） ──────────┤
                                                ▼
              Data_Collection/collect/collect.py（.venv-collect）
                → data/lerobot/airbot_revo2_right/（parquet + mp4）
```

操作约定：

| 输入 | 作用 |
|------|------|
| 左踏板 | 臂 deadman（按住跟随） |
| 右踏板 | 数采门控 `/teleop/hand_recording`（按下写帧，松开保存） |
| MANUS 手套 | 驱动 Revo2；指尖力经振动回传到手套 |

---

## `docker exec` 写法（易错）

必须带齐：**选项 + 容器名 + 命令**。只写 `-w` 会报 `requires at least 2 arguments`。

```bash
# 进合容器 shell（手侧必须 -u 0）
docker exec -it -u 0 -w /home/dt/manus-Revo2-Revo3 \
  airbot-revo-humble bash

# 进合容器 shell（臂侧工作目录）
docker exec -it -u 0 -w /home/ubuntu/airbot+hand-workspace \
  airbot-revo-humble bash

# 容器内直接跑一条命令
docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
  airbot-revo-humble python3 app.py --config config/airbot.yaml
```

合容器是 `PidMode=host`：**在容器里乱 `pkill` 会杀到宿主机进程**，停进程用脚本记的 PID 或精确匹配。

---

## 从遥操到数采（推荐流程：VIVE + 右手）

下面按冷启动顺序。**标注「宿主机」/「容器内」**。

### 0. 公共：合容器 + 臂 sidecar（宿主机）

```bash
/home/ubuntu/airbot-revo-workspace/bringup.sh --no-build
/home/ubuntu/airbot-revo-workspace/bringup.sh status

/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh
```

看臂灯条：**绿色呼吸**再继续。检查：

```bash
ip -br link show can0
docker inspect -f '{{.State.Running}}' airbot-revo-humble airbot-play-runtime
```

### 1. 遥操一键（宿主机）

先起 SteamVR（宿主机，不要进 Docker）：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/start_steamvr.sh
./scripts/close_room_setup.sh          # 若弹出 Room Setup
python3 scripts/list_trackers.py       # body / right_wrist 均为 valid=true
```

再一键拉起 **rosbridge + 右手真机 pipeline（含触觉桥）+ VIVE 真机臂**：

```bash
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/scripts/teleop_vive_hand_up.sh
# 调试臂命令：…/teleop_vive_hand_up.sh --dummy-arm
```

该脚本在**容器内后台**会做的事等价于：

```bash
# rosbridge
docker exec -d -u 0 airbot-revo-humble \
  bash -lc 'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml'

# 右手真机 + MANUS retarget + 触觉→Manus 振动
docker exec -u 0 airbot-revo-humble bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/install/setup.bash
  exec ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
    hand_mode:=right \
    controller_backend:=ros2_control \
    launch_plot:=false \
    read_touch_status:=true \
    launch_haptic:=true
'
```

并在**宿主机前台**跑：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
python3 scripts/run_vive_teleop.py --real-arm
```

可选：进容器确认手话题（另开终端）：

```bash
docker exec -it -u 0 airbot-revo-humble bash
# --- 以下在容器内 ---
source /opt/ros/humble/setup.bash
source /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/install/setup.bash
ros2 topic hz /manus_glove_0
ros2 topic echo --once /revo2_right/revo2_joint_state/joint_states
ros2 topic echo --once /revo2_right/revo2_joint_state/dynamic_joint_states
```

### 2. 数采（宿主机，不要进 Docker）

采集器要 Python ≥ 3.12 + vendored LeRobot，必须用 `.venv-collect`：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/Data_Collection
.venv-collect/bin/python collect/collect.py --task airbot_revo2_right
```

| 操作 | 作用 |
|------|------|
| 右踏板 | 按下开始写帧，松开自动保存 episode |
| Space | 键盘开始/停止 |
| `s` / Enter | 保存 |
| `d` | 丢弃 |
| `q` | 退出 |

落盘目录：

```text
Data_Collection/data/lerobot/airbot_revo2_right/
  meta/info.json
  data/**/*.parquet
  videos/observation.images.cam_global/**/*.mp4
```

配置：[`collect/config/collect.yaml`](airbot+hand-workspace/Data_Collection/collect/config/collect.yaml)  
（默认 `cam_global` = LRCP by-id；USB 对照 [`docs/USB_DEVICES.md`](airbot+hand-workspace/docs/USB_DEVICES.md)；订 `/arm/state` + 右手 `joint_states` / `dynamic_joint_states`）

无硬件自检：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/Data_Collection
.venv-collect/bin/python collect/collect.py --mock --seconds 1 --task mock_smoke
```

---

## 备选：分步手动启动

### A. 仅手（容器内）

```bash
docker exec -it -u 0 -w /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting \
  airbot-revo-humble bash
```

容器内：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 真机右手（含触觉桥）
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  launch_plot:=false \
  read_touch_status:=true \
  launch_haptic:=true

# 只要手套→目标话题、不驱动真机时：
# ros2 launch manus_revo2_retarget pipeline_launch.py \
#   hand_mode:=right launch_manus_publisher:=true controller_backend:=ros2_control
```

### B. 仅臂 · PICO（容器内）

宿主机先：

```bash
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh
adb reverse tcp:15000 tcp:15000
```

再进容器跑遥操：

```bash
docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
  airbot-revo-humble python3 app.py --config config/airbot.yaml
```

### C. 仅臂 · VIVE（宿主机，不进容器）

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/start_steamvr.sh
python3 scripts/list_trackers.py
python3 scripts/run_vive_teleop.py --real-arm
```

此时若还要数采，需自己在容器内起 rosbridge（以及手 pipeline，若要采手）：

```bash
docker exec -d -u 0 airbot-revo-humble \
  bash -lc 'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
```

---

## 停止

```bash
# 数采：采集终端按 q

# VIVE 遥操：该终端 Ctrl-C，然后停脚本拉起的手链路 / rosbridge
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/scripts/teleop_vive_hand_up.sh stop

# 停臂 sidecar（会先尽量停 app.py）
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh stop

# 停合容器（sidecar 是否保留取决于 bringup 实现；默认先 stop 工作容器）
/home/ubuntu/airbot-revo-workspace/bringup.sh stop

# 停 SteamVR
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr/scripts/stop_steamvr.sh
```

手动停容器内手 launch：在**跑 launch 的那个** `docker exec` 终端 Ctrl-C。

---

## 关键配置

| 用途 | 文件 |
|------|------|
| VIVE / 踏板 / 门控话题 | [`airbot+hand-workspace/EE_controlhal/VIVE_control_openvr/config/teleop.yaml`](airbot+hand-workspace/EE_controlhal/VIVE_control_openvr/config/teleop.yaml) |
| 数采：相机、手话题、repo_id | [`airbot+hand-workspace/Data_Collection/collect/config/collect.yaml`](airbot+hand-workspace/Data_Collection/collect/config/collect.yaml) |
| USB 设备对照 | [`airbot+hand-workspace/docs/USB_DEVICES.md`](airbot+hand-workspace/docs/USB_DEVICES.md) |
| 臂 SDK | [`airbot+hand-workspace/EE_controlhal/Robot_control_interface/config/airbot_sdk.yaml`](airbot+hand-workspace/EE_controlhal/Robot_control_interface/config/airbot_sdk.yaml) |
| 手真机 launch | `manus-Revo2-Revo3/…/manus_revo2_retarget/launch/real_hand_pipeline_launch.py` |

更多细节：

- 臂 / VIVE：[airbot+hand-workspace/README.md](airbot+hand-workspace/README.md)、[VIVE_control_openvr/README.md](airbot+hand-workspace/EE_controlhal/VIVE_control_openvr/README.md)
- 手：[manus-Revo2-Revo3/README.md](manus-Revo2-Revo3/README.md)
- 数采字段：[Data_Collection/docs/DATA_FORMAT.md](airbot+hand-workspace/Data_Collection/docs/DATA_FORMAT.md)
