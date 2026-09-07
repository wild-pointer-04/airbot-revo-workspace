# Data_Collection

采集 **图像 + 机械臂关节角 / 末端**，写入 LeRobot。只负责落盘，不做校验或可视化。

遥操在 `EE_controlhal/VIVE_control`（或 PICO `middle_interface`），通过 ROS2 `/arm/state` 把臂状态送给本目录。

```bash
cd /home/dt/manus-Revo2-Revo3/airbot+hand-workspace/Data_Collection
python3 collect/collect.py --task pick_cup
```

没有相机 / 机械臂时，先用 Mock 确认程序能跑：

```bash
python3 collect/collect.py --mock --seconds 1 --task mock_smoke
```

---

## 采什么

| 模态 | 采集时做什么 | 写到哪里 |
|------|----------------|----------|
| 右腕 RealSense D405 | 30 fps 取流，约 **10 Hz** 落一帧 | LeRobot（或 JSONL 回退） |
| AIRBOT Play | 订 `/arm/state`：6 关节 + 末端 xyz / `[x,y,z,w]` | 同一帧里的 `arm_*` |

---

## 目录

```text
Data_Collection/
  collect/   采集程序和配置（从这里启动）
  tools/         遗留诊断
  vendor/        内置 lerobot-main
  docs/          原始字段说明
  data/          录出来的数据（已 gitignore）
```

| 你想做的事 | 去哪 |
|------------|------|
| 开始录 | `collect/collect.py` |
| 改相机 SN、是否 Mock | `collect/config/collect.yaml` |
| VIVE 遥操 | `../EE_controlhal/VIVE_control` |

---

## 采集怎么用

所有命令都在本目录执行：

```bash
cd /home/dt/manus-Revo2-Revo3/airbot+hand-workspace/Data_Collection
python3 collect/collect.py --task pick_cup
```

指定输出目录：

```bash
python3 collect/collect.py --task pick_cup --data-root /path/to/data
```

启动后：

| 键 | 作用 |
|----|------|
| Space | 开始 / 停止当前这一条 |
| `s` 或 Enter | 保存 |
| `d` | 丢掉，不保存 |
| `q` | 退出 |

配置文件：`collect/config/collect.yaml`。

---

## 录完数据在哪

```text
data/lerobot/<任务名>/
  meta/info.json
  data/chunk-000/file-000.parquet
  videos/observation.images.cam_right_wrist/.../file-000.mp4
```

---

## 自测

```bash
python3 -m pytest collect/tests -q
```

字段含义见 [docs/DATA_FORMAT.md](docs/DATA_FORMAT.md)。
