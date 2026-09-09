# collect

采集主程序。从仓库根目录启动：

```bash
cd /home/ubuntu/airbot+hand-workspace/Data_Collection
python3 collect/collect.py --task pick_cup
```

真机默认订 ROS2 `/arm/state`（关节角 + 末端），由 `EE_controlhal/VIVE_control_openvr` 或 PICO `middle_interface` 发布。采集器只订阅后写入 LeRobot。

## 真机（ROS2）

先起 AIRBOT 和遥操（会发 `/arm/state`），再采：

```bash
# 终端 1：VIVE OpenVR 遥操（宿主机；body=头显，right_wrist=手柄）
cd ../EE_controlhal/VIVE_control_openvr
python3 scripts/run_vive_teleop.py --real-arm

# 终端 2：采集
cd ../../Data_Collection
.venv-collect/bin/python collect/collect.py --task pick_cup
```

或：

```bash
source /opt/ros/humble/setup.bash
ros2 launch collect/ros2/launch/collect.launch.py task:=pick_cup
```

话题：`/arm/state`（`std_msgs/String` JSON，字段与 LeRobot `observation.state.*` 一致）。

`--mock` 单测走本地假臂状态，不订话题。

## 常用命令

```bash
# 无硬件，录 1 秒后自动保存退出
python3 collect/collect.py --mock --seconds 1 --task mock_smoke

# 换配置 / 换输出根目录
python3 collect/collect.py --config collect/config/collect.yaml --data-root ./data
```

键盘：Space 开始/停止，`s` 保存，`d` 丢弃，`q` 退出。

## 这个目录里有什么

```text
collect/
  collect.py              启动脚本
  config/collect.yaml     相机、ROS2 话题、fps
  collector/          相机 / 订阅 ROS2 / 写盘
  ros2/launch/            起采集器
  tests/
```

## 改配置

编辑 `config/collect.yaml`：

| 项 | 说明 |
|----|------|
| `dataset.fps` | 落盘频率，默认 10 |
| `cameras.*` | 只写接入的相机；现在只有 `cam_right_wrist` |
| `state_source` | `ros2`（默认，订话题）或 `local`（`--mock` 假臂） |
| `ros2.arm_topic` | `/arm/state` |
| `rosbridge.enabled` | 默认关；打开后订 `/record_cmd`（start/stop/save/discard） |

命令行 `--mock`、`--task`、`--data-root` 会覆盖 yaml 里的对应项。

## 一帧里有什么

右腕图 `cam_right_wrist` + `host_monotonic_ns` + 机械臂关节 / 末端。

- **臂：** `arm_joint`（6）、`arm_ee_position`、`arm_ee_orientation`（`[x, y, z, w]`）、`arm_valid`

未安装 vendor LeRobot 的运行依赖时，帧写成 `data/lerobot/<task>/episodes/episode-*/frames.jsonl`。
容器里装 `collect/requirements-lerobot-record.txt` 后走 `vendor/lerobot-main`，写出 parquet + MP4。
