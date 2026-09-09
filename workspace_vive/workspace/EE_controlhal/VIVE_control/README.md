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
docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble python3 app.py --config config/teleop.yaml
```

无硬件自测：

```bash
python3 app.py --mock --dummy-arm
```

`config/tracker.yaml` 使用 `libsurvive`，并把 `serial` 换成 `scripts/list_devices.py` 打出来的稳定设备 ID。临时运行别名（例如本机的 `T20`、`T21`）会随环境变化，确认后应换为设备的 `LHR-...` ID。

## 坐标与标定

本机 libsurvive 原生位姿为 `+Z` 上，`vr_space: libsurvive` 保留位置和姿态，不做 OpenXR 的 `[-z,-x,y]` 变换。水平世界轴由基站校准决定，不能直接等同于人体前/左；`HeadBase` 结合腰部安装外参后才生成 `+X` 前、`+Y` 左、`+Z` 上的人体坐标。SteamVR/OpenXR 使用 `+Y` 上，必须与原生 libsurvive 区分。

本机 AIRBOT SDK 的笛卡尔命令轴实测为 `+X` 下、`+Y` 左、`+Z` 前，因此默认 `transform.axis_map` 为：

```yaml
x: ["z", -1.0] # teleop 上 -> SDK -X
y: ["y", 1.0]  # teleop 左 -> SDK +Y
z: ["x", 1.0]  # teleop 前 -> SDK +Z
```

腰部 tracker 给出操作者的水平朝向。腰部竖直安装、手腕 tracker 水平且绿灯面朝上是允许的，两者不必同向。腰部 tracker 必须绑紧，标定和遥操时躯干正向一致；手腕 tracker 只要在视野内并牢固固定即可。绿灯面不是配置坐标轴，不能据此手填外参。

先标定腰部安装。脚本只读取两个 Tracker，不启动遥操或机器人，也只会改 `teleop.head_tracker_to_body`。矩阵定义为 `body_vector = E @ tracker_vector`，其中 body 为 `+X` 前、`+Y` 左、`+Z` 上；它在运行时用于生成身体 yaw：

```bash
docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble \
  python3 scripts/calibrate_body_mount.py --config config/teleop.yaml --write
```

步骤是：站直且固定腰部 tracker，采一次手腕相对腰部的起始位置；随后每次回到起始位置，分别把手腕 tracker 沿身体正前、左方、上方纯平移至少 30 cm。每个姿势按回车后静止两秒。四步使用同一次跟踪进程，并在基站解算稳定后开始；基站参考切换时应重新采整轮。脚本拒绝过短、非正交、左手系、腰部转动、静止抖动或有效新帧不足的数据，并验证世界向上和实际 `HeadBase` 三轴重放，因此失败时不写配置。

手腕 tracker 的固定安装方向不改变当前的 XYZ 平移映射，也不要求重标定 `axis_map`。当前相对姿态增量里固定 tracker-to-hand 旋转会抵消；绝对工具姿态或手掌长度偏置才需要独立的手部外参。

两台 tracker 都必须有效才会发送命令。运行三轴离线标定并在结果通过后再写配置：

```bash
docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble \
  python3 scripts/calibrate_mapping.py --config config/teleop.yaml --write
```

`calibrate_mapping.py` 只标定 `transform.axis_map`，不会覆盖腰部安装矩阵。仅在基站世界系、`vr_space` 或 AIRBOT 命令坐标定义改变，或三轴复核失败时运行它；重新绑腰部或调整手腕佩戴只运行 `calibrate_body_mount.py`。

数采订 `/arm/state`，见 `Data_Collection/collect`。

```bash
python3 -m pytest tests -q
```
