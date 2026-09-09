# AIRBOT 臂 + Revo2 手 合工作区

PICO / VIVE 遥操 AIRBOT Play 机械臂，与 MANUS 遥操 BrainCo Revo2 灵巧手，跑在**同一个 Humble 工作容器**里。两套控制链独立并行：臂走 CAN + gRPC，手走 USB/串口 + ROS 2。

臂侧另带一套数采（图像 + 臂状态 → LeRobot），跑在宿主机独立的 Python 3.12 环境里，经 rosbridge 取臂状态。**手的数据不在采集范围内。**

离线镜像包：`/home/ubuntu/airbot-revo-humble.tar.gz`  
加载：`docker load -i /home/ubuntu/airbot-revo-humble.tar.gz`

## 结构

```text
airbot-revo-workspace/
  README.md                          本文件：总览 + 总启动顺序
  bringup.sh                         拉起合容器（转调 manus 侧脚本）
  airbot+hand-workspace/             臂：PICO / VIVE → AIRBOT + 数采（见该目录 README）
  manus-Revo2-Revo3/                 手：MANUS → Revo2（见该目录 README）
    Dockerfile.combined              合容器镜像定义
```

容器内路径（不要在宿主机 `source` 手侧 `install/`）：

| 宿主机 | 容器内 |
|--------|--------|
| `airbot-revo-workspace/airbot+hand-workspace` | `/home/ubuntu/airbot+hand-workspace` |
| `airbot-revo-workspace/manus-Revo2-Revo3` | `/home/dt/manus-Revo2-Revo3` |

| 容器 | 作用 |
|------|------|
| `airbot-revo-humble` | 工作容器（ROS 2 Humble + airbot_py + 两套源码） |
| `airbot-play-runtime` | 官方臂驱动 sidecar，gRPC `localhost:50051`，管 `can0` |

```text
PICO VR :15000 ──┐
                 ├─► EE_controlhal/*/app.py ──► gRPC :50051 ──► airbot-play-runtime ──► can0 ──► AIRBOT 臂
VIVE (OpenVR) ─┘   │
                              └─► /arm/state ──► rosbridge :9090 ──► Data_Collection ──► LeRobot

MANUS USB 3325 ──► manus_data_publisher ──► manus_revo2_retarget ──► revo2_driver ──► 串口 ──► Revo2 手
```

## `docker exec` 写法（易错）

必须带齐：**选项 + 容器名 + 命令**。只写 `-w 路径` 会报 `requires at least 2 arguments`。

```bash
# 正确：进 shell（指定工作目录）
docker exec -it -u 0 -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
  airbot-revo-humble bash

# 正确：直接跑程序
docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
  airbot-revo-humble python3 app.py --config config/airbot.yaml

# 错误：缺容器名和命令
# docker exec -it -w /home/ubuntu/.../middle_interface
```

多行命令末尾的 `\` 后面不能有空格；也可写成一行。

## 启动顺序（先公共，再开两个终端）

两套链不要绑成一个 launch。先起容器，再分终端跑臂和手。

### 0. 公共：合容器 + 臂 sidecar

```bash
/home/ubuntu/airbot-revo-workspace/bringup.sh --no-build
/home/ubuntu/airbot-revo-workspace/bringup.sh status
```

需要 CAN 时 sidecar 会跟着起；没有 `can0` 也可以先只跑手。

### 1. 臂（另开终端）

按 [airbot+hand-workspace/README.md](airbot+hand-workspace/README.md)：

```bash
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh
```

灯条绿色呼吸后，二选一：

**PICO（容器内）：**

```bash
docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
  airbot-revo-humble python3 app.py --config config/airbot.yaml
```

**VIVE / SteamVR OpenVR（宿主机，不要进容器）：**

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/start_steamvr.sh
./scripts/close_room_setup.sh          # 若弹出 Room Setup
python3 scripts/list_trackers.py       # 两边 valid=true 再遥操
python3 scripts/run_vive_teleop.py     # 默认 dummy 臂；真机加 --real-arm
```

细节见该目录 [README.md](airbot+hand-workspace/EE_controlhal/VIVE_control_openvr/README.md)。

### 1b. 数采（可选，要先起遥操）

```bash
docker exec -d -u 0 airbot-revo-humble \
  bash -lc 'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/Data_Collection
.venv-collect/bin/python collect/collect.py --task pick_cup
```

### 2. 手（再开终端，必须 root）

按 [manus-Revo2-Revo3/README.md](manus-Revo2-Revo3/README.md)。

先进容器（默认目录是 `/home/dt/manus-Revo2-Revo3`，可用 `-w` 固定）：

```bash
docker exec -it -u 0 -w /home/dt/manus-Revo2-Revo3 \
  airbot-revo-humble bash
```

进入后在容器内执行：

```bash
source /opt/ros/humble/setup.bash
source /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/install/setup.bash
ros2 launch manus_revo2_retarget pipeline_launch.py \
  hand_mode:=both launch_manus_publisher:=true controller_backend:=ros2_control
```

无手套可先用仿真 publisher；真机手再换 `real_hand_pipeline_launch.py`。

### 停

- 臂遥操：`app.py` / `run_vive_teleop.py` 所在终端 **Ctrl-C**（不要直接杀进程）
- 采集器：`q` 退出；rosbridge 可以一直留着
- 手 launch：该终端 Ctrl-C，或在**宿主机**跑 `pkill -f 'pipeline_launch.py|manus_data_publisher|manus_revo2_retarget_node'`
- 停合容器（sidecar 保留）：`/home/ubuntu/airbot-revo-workspace/bringup.sh stop`
- 停臂驱动：`/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh stop`
- 停 SteamVR（若用了 OpenVR）：`…/VIVE_control_openvr/scripts/stop_steamvr.sh`

> 合容器是 `PidMode=host`，**在容器内**跑 `pkill` 会杀到宿主机进程。要在容器里停进程就用精确 PID。
