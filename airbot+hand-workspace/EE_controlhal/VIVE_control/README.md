# VIVE_control

两台 VIVE Tracker 走和 PICO 同一套遥操作：一台绑身上当头显，一台当手柄，增量 clutch 去控 AIRBOT Play。

```text
body tracker  → HMD
hand tracker  → right controller
keyboard      → trigger / home / pause
  → PicoTeleopPacket
  → middle_interface TeleopPipeline
  → Robot_control_interface / airbot_sdk
  → /arm/state   # 关节角 + 末端，给数采订
```

Tracker 没有扳机，用键盘代替 PICO 按键：

| 键 | 对应 PICO | 作用 |
|----|-----------|------|
| Space | 右手食指扳机 | 按下开始跟手，再按松开保持 |
| `h` | 右手 A | 回 home |
| `p` | 左手 primary | 暂停 |
| `g` | 侧键 grip | 夹爪（当前 AIRBOT 未启用末端） |

## 运行

先起 AIRBOT 驱动，再：

```bash
cd /home/ubuntu/airbot+hand-workspace/EE_controlhal/VIVE_control
python3 app.py --config config/teleop.yaml
```

无硬件自测：

```bash
python3 app.py --mock --dummy-arm
```

`config/tracker.yaml` 里把 `backend` 改成 `libsurvive`，并把 `serial` 换成 `scripts/list_devices.py` 打出来的 `LHR-...`。

坐标系：`vr_space: steamvr`（+X 右，+Y 上，-Z 前）→ 与 PICO 相同的 `teleop_standard`。

数采订 `/arm/state`，见 `Data_Collection/collect`。

```bash
python3 -m pytest tests -q
```
