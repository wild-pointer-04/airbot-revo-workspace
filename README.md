# AIRBOT 臂 + Revo2 手 合工作区

PICO 遥操 AIRBOT Play 机械臂，与 MANUS 遥操 BrainCo Revo2 灵巧手，跑在**同一个 Humble 工作容器**里。两套控制链独立并行：臂走 CAN + gRPC，手走 USB/串口 + ROS 2。

离线镜像包：`/home/ubuntu/airbot-revo-humble.tar.gz`  
加载：`docker load -i /home/ubuntu/airbot-revo-humble.tar.gz`

## 结构

```text
airbot-revo-workspace/
  README.md                          本文件：总览 + 总启动顺序
  bringup.sh                         拉起合容器（转调 manus 侧脚本）
  airbot+hand-workspace/             臂：PICO → AIRBOT（见该目录 README）
  manus-Revo2-Revo3/                 手：MANUS → Revo2（见该目录 README）
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
PICO VR :15000 ──► EE_controlhal/app.py ──► gRPC :50051 ──► airbot-play-runtime ──► can0 ──► AIRBOT 臂

MANUS USB 3325 ──► manus_data_publisher ──► manus_revo2_retarget ──► revo2_driver ──► 串口 ──► Revo2 手
```

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
# 灯条绿色呼吸后：
docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
  airbot-revo-humble python3 app.py --config config/airbot.yaml
```

### 2. 手（再开终端，必须 root）

按 [manus-Revo2-Revo3/README.md](manus-Revo2-Revo3/README.md)：

```bash
docker exec -it -u 0 airbot-revo-humble bash
source /opt/ros/humble/setup.bash
source /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/install/setup.bash
ros2 launch manus_revo2_retarget pipeline_launch.py \
  hand_mode:=both launch_manus_publisher:=true controller_backend:=ros2_control
```

无手套可先用仿真 publisher；真机手再换 `real_hand_pipeline_launch.py`。

### 停

- 臂遥操：`app.py` 所在终端 **Ctrl-C**（不要直接杀进程）
- 手 launch：该终端 Ctrl-C，或 `pkill -f 'pipeline_launch.py|manus_data_publisher|manus_revo2_retarget_node'`
- 停合容器（sidecar 保留）：`/home/ubuntu/airbot-revo-workspace/bringup.sh stop`
- 停臂驱动：`…/airbot+hand-workspace/teleop_up.sh stop`
