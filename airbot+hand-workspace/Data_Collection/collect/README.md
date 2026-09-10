# collect

统一采集主程序（图像 + 臂 + Revo2 手）。从仓库根目录启动：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/Data_Collection
.venv-collect/bin/python collect/collect.py --task airbot_revo2_right
```

真机默认 `state_source: rosbridge`：订 `/arm/state`、手 `joint_states` / `dynamic_joint_states`，相机本地 OpenCV 读全局机 by-id（见 `config/collect.yaml`）。

## 真机

```bash
# 终端 1：VIVE + 右手 + rosbridge（一键）
../scripts/teleop_vive_hand_up.sh

# 终端 2：采集
.venv-collect/bin/python collect/collect.py --task airbot_revo2_right
```

右踏板门控录制；键盘 Space/`s`/`d`/`q` 仍可用。`--mock` 走本地假臂/假手/假图。

## 常用命令

```bash
python3 collect/collect.py --mock --seconds 1 --task mock_smoke
python3 collect/collect.py --config collect/config/collect.yaml --data-root ./data
```

## 改配置

编辑 `config/collect.yaml`：

| 项 | 说明 |
|----|------|
| `dataset.repo_id` | 默认 `airbot_revo2_right` |
| `dataset.fps` | 落盘频率，默认 10 |
| `cameras.*.type` | `opencv`（默认）/ `realsense` / `mock` |
| `cameras.*.device` | UVC，用 `/dev/v4l/by-id/…`；默认全局机 LRCP |
| `hand.*` | 手话题与 tactile 话题 |
| `recording_gate.*` | 默认订 `/teleop/hand_recording` |
| `state_source` | `rosbridge`（默认）/ `ros2` / `local` |

## 一帧里有什么

全局图 `cam_global` + `host_monotonic_ns` + `arm_*` + `hand_right_joint/velocity/valid/tactile` + `action(=arm_joint)`。

详见 [../docs/DATA_FORMAT.md](../docs/DATA_FORMAT.md)。
