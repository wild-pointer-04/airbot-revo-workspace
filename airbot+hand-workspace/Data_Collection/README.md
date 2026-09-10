# Data_Collection

采集 **图像 + 机械臂关节角/末端 + Revo2 手反馈（含触觉）**，写入同一 LeRobot episode。  
只负责落盘，不做校验或可视化。

遥操：宿主机 VIVE（`EE_controlhal/VIVE_control_openvr`）控臂，并发布 `/arm/state` 与右踏板门控 `/teleop/hand_recording`；容器内 MANUS→Revo2 控手。一键入口：

```bash
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/scripts/teleop_vive_hand_up.sh
```

采集器跑在**自己的 Python 3.12 环境** `.venv-collect` 里，经 rosbridge websocket 取臂/手状态 —— 因为合容器是 ROS 2 Humble / Python 3.10，而 `vendor/lerobot-main` 要求 Python ≥ 3.12。

先保证遥操与 rosbridge 已起，再在宿主机开录：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/Data_Collection
.venv-collect/bin/python collect/collect.py --task airbot_revo2_right
```

没有相机 / 机械臂 / rosbridge 时，先用 Mock：

```bash
.venv-collect/bin/python collect/collect.py --mock --seconds 1 --task mock_smoke
```

---

## 采什么

| 模态 | 采集时做什么 | 写到哪里 |
|------|----------------|----------|
| 全局 USB 相机（LRCP V1080P） | OpenCV 取流，约 **10 Hz** 落一帧 | LeRobot 视频 |
| AIRBOT Play | rosbridge 订 `/arm/state`：6 关节 + 末端 | `arm_*` |
| Revo2 右手 | rosbridge 订 `joint_states` + `dynamic_joint_states` | `hand_right_*` + tactile |

默认相机：`cam_global` → `/dev/v4l/by-id/usb-LRCP_V1080P_LRCP_V1080P-video-index0`（见 `collect/config/collect.yaml`；USB 对照 [docs/USB_DEVICES.md](../docs/USB_DEVICES.md)）。

---

## 目录

```text
Data_Collection/
  collect/        统一采集（图+臂+手）——从这里启动
  vendor/         内置 lerobot-main（需要 Python >= 3.12）
  docs/           字段说明
  data/           录出来的数据（已 gitignore）
  .venv-collect/  采集专用 Python 3.12 环境（已 gitignore）
```

| 你想做的事 | 去哪 |
|------------|------|
| 开始录 | `collect/collect.py` |
| 改相机设备、手话题 | `collect/config/collect.yaml` |
| VIVE + 手一键遥操 | `../scripts/teleop_vive_hand_up.sh` |

---

## 采集怎么用

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/Data_Collection
.venv-collect/bin/python collect/collect.py --task airbot_revo2_right
```

启动后：

| 键 / 踏板 | 作用 |
|-----------|------|
| 右踏板（`/teleop/hand_recording`） | 按下开始写帧，松开自动保存 |
| Space | 开始 / 停止当前这一条 |
| `s` 或 Enter | 保存 |
| `d` | 丢掉，不保存 |
| `q` | 退出 |

依赖：`opencv-python-headless`（UVC 相机）、`roslibpy`、以及 `collect/requirements-lerobot-record.txt`（parquet/MP4）。

---

## 录完数据在哪

```text
data/lerobot/airbot_revo2_right/
  meta/info.json
  data/chunk-000/file-000.parquet
  videos/observation.images.cam_global/.../file-000.mp4
```

---

## 自测

```bash
.venv-collect/bin/python -m pytest collect/tests -q
```

字段含义见 [docs/DATA_FORMAT.md](docs/DATA_FORMAT.md)。
