# VIVE Tracker 遥操（SteamVR / OpenVR）

用两台 HTC VIVE Tracker（腰/身 + 右腕）在 **宿主机** 上通过 SteamVR 取位姿，编成与 PICO 相同的 `PicoTeleopPacket`，再走 `middle_interface` 的 `TeleopPipeline` 控 AIRBOT Play。

**不要把 SteamVR 装进 Docker。** Watchman 接收器同时只能被一套程序占用（不要残留 `survive-cli` / libsurvive）。

---

## 遥操链路（数据怎么走）

```text
Lighthouse 基站 ──光──► VIVE Tracker（body / right_wrist）
                              │
                     Watchman USB 接收器
                              │
                     SteamVR（宿主机）求解世界位姿
                              │
                     OpenVR API（本包 vive_openvr/）
                              │
              ┌───────────────┴───────────────┐
              │ 按 LHR- 序列号绑定角色          │
              │ body → 当作 HMD（腰原点）       │
              │ right_wrist → 当作右手柄       │
              │ 脚踏板/键盘 → trigger=离合 等  │
              └───────────────┬───────────────┘
                              │ PicoTeleopPacket
                              ▼
              middle_interface TeleopPipeline
                · vr_space=steamvr → teleop 标准系
                · head_base（相对腰，走动手不动）
                · 离合相对增量（按住才跟）
                · 滤波 / 行程限位 / 工作空间
                              │ EE 目标位姿
                              ▼
              Robot_control_interface（airbot_py）
                              │ gRPC :50051
                              ▼
              airbot-play-runtime ──can0──► AIRBOT Play
```

要点：

| 环节 | 说明 |
|------|------|
| 追踪 | SteamVR 负责光场求解；本包只读 OpenVR 位姿 |
| 绑定 | `config/tracker.yaml` 用永久序列号 `LHR-…`，不用设备序号 |
| 语义 | `body` 当头显（原点），`right_wrist` 当右手；与 PICO 共用同一套管线 |
| 坐标系 | `teleop.yaml` 里 `vr_space: steamvr`；本机已调到位置轴正确（`head_yaw_offset_deg: 0`） |
| 离合 | 左脚踏 **按住=跟随，松开=停**（`KEY_1`）；空格可作键盘备用切换 |
| 手部数采 | 右脚踏 **按住=写 Revo2 反馈，松开=保存 episode**（`KEY_2`）；VIVE 是 CM6K 唯一 HID 读取者 |
| 真机 | 宿主机跑遥操进程；臂驱动在 sidecar `:50051`（本机已装 `airbot_py`） |

---

## 目录

```text
VIVE_control_openvr/
  config/tracker.yaml       # LHR 绑定 + 外参（默认单位阵）
  config/teleop.yaml        # 管线 / 离合 / 工作空间 / 臂适配器
  vive_openvr/              # OpenVR → PacketSource
  scripts/                  # SteamVR 启停 + 遥操入口
  docs/steamvr_setup.md     # SteamVR / Null Driver 细节（英文）
  tests/                    # 无 SteamVR 的单测
```

| 角色 | 序列号（本机） |
|------|----------------|
| `body` | `LHR-CB0CFB64` |
| `right_wrist` | `LHR-15A9EB13` |

---

## 操作键

| 输入 | 作用 |
|------|------|
| **左脚踏**（SayoDevice CM6K，按住） | 离合 / deadman：跟随 |
| 松开左脚踏 | 停止跟随（保持当前位姿） |
| **右脚踏**（SayoDevice CM6K，按住） | 发布数据采集门控，手部采集器写帧 |
| 松开右脚踏 | 完成并保存当前手部 episode |
| Space | 离合切换（无踏板时备用） |
| `h` | 回零位 |
| `p` | 暂停 |
| `g` | 夹爪切换（当前 `enable_eef: false` 时实际忽略） |

踏板权限（只需做一次）：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/fix_pedal_udev.sh    # 需 sudo；不行就拔插踏板 USB
python3 scripts/probe_pedal.py  # 确认左踏 = KEY_1，右踏 = KEY_2（已写入 teleop.yaml）
```

---

## 从头到尾：真机遥操启动

下面按**冷启动**顺序。每一步占一个终端看日志；路径按本机仓库写死。

### 0. 硬件检查

- 两台 Lighthouse 上电，A/B 信道正确
- 两台 Tracker 开机，Watchman 接收器插好
- USB2CAN → `can0`；脚踏板 USB 已插
- 机械臂上电，周围无碰撞障碍

```bash
ip -br link show can0    # 应含 UP
```

### 1. 合容器 + 臂驱动（宿主机）

```bash
/home/ubuntu/airbot-revo-workspace/bringup.sh --no-build
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh
```

看臂灯条：**绿色呼吸** 再继续。白灯=驱动未接管；黄灯=需校零。异常可：

```bash
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh restart
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh status
```

### 2. 启动 SteamVR（宿主机，另开终端）

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/start_steamvr.sh
# 若弹出 Room Setup：
./scripts/close_room_setup.sh
```

确认 Tracker 在线（两边都要 `valid=true`）：

```bash
python3 scripts/list_trackers.py
```

期望类似：`body` / `right_wrist` 均为 `connected=true valid=true status=OK`。  
Null Driver 下的 Error 496（Failed to connect to headset display）可忽略。更多排查见 [docs/steamvr_setup.md](docs/steamvr_setup.md)。

### 3. 启动遥操（真机）

仍在该目录：

```bash
# Hand recording gate uses ROS 2 rclpy; source the same ROS environment as the Revo2 driver.
source /opt/ros/humble/setup.bash
python3 scripts/run_vive_teleop.py --real-arm
```

日志应对上：

- `ROBOT ADAPTER: COMMAND_LAYER`
- `Foot pedal listening ... deadman=KEY_1 recording=KEY_2 mode=hold`
- `Publishing /teleop/hand_recording at 20 Hz`
- `[OPENVR] tracking head=True hand=True`
- 周期性 `clutch=False`（未踩踏板）或踩下后跟随

### 4. 操作

1. 身体自然站姿，手持/腕戴 Tracker 在舒适位
2. **踩住左脚踏** → 臂跟随手的位置与姿态
3. **松开脚踏** → 臂停在当前目标
4. 需要回零：终端焦点在遥操窗口时按 `h`
5. 结束：该终端 **Ctrl-C**（不要强杀）

### 5. 停止

```bash
# 遥操终端：Ctrl-C
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/stop_steamvr.sh
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh stop
# 可选：停合容器（保留 sidecar 可只 stop bringup）
/home/ubuntu/airbot-revo-workspace/bringup.sh stop
```

---

## 一键抄命令（真机最短路径）

终端 A（臂）：

```bash
/home/ubuntu/airbot-revo-workspace/bringup.sh --no-build
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh
```

终端 B（SteamVR + 遥操）：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/start_steamvr.sh
./scripts/close_room_setup.sh   # 需要时
python3 scripts/list_trackers.py
source /opt/ros/humble/setup.bash
python3 scripts/run_vive_teleop.py --real-arm
```

---

## 安全空跑 / 干跑（不推真机）

不启 sidecar 也可先验证追踪与管线：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/start_steamvr.sh
python3 scripts/list_trackers.py
python3 scripts/run_vive_teleop.py              # dummy 臂（默认）
# 或只打印目标、不 send：
python3 scripts/run_vive_teleop.py --print-only
```

---

## 首次依赖（装过可跳过）

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
python3 -m pip install -r requirements.txt
# 真机还需宿主机能 import airbot_py（本机已装 5.1.6 wheel）
# 手部数采门控还需可 import rclpy（由 ROS 2 Humble 提供，先 source /opt/ros/humble/setup.bash）
```

无 SteamVR 的单测：

```bash
python3 -m pytest tests/ -q
```

---

## 分阶段排查（出问题再跑）

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
./scripts/start_steamvr.sh

# 1) 能否看见 Tracker
python3 scripts/list_trackers.py
python3 scripts/test_openvr_pose.py --role right_wrist

# 2) 质量 / 相对腰
python3 scripts/test_tracking_quality.py --role right_wrist --duration 30
python3 scripts/test_body_relative.py
python3 scripts/test_body_relative.py --axis-test

# 3) 组包 + 假臂管线
python3 scripts/test_packet_source.py
python3 scripts/test_packet_source.py --pipeline
```

相对腰自检：

- **一起走**：世界坐标变，身→手相对几乎不变
- **腰不动、只动手**：身→手相对变化
- **`--axis-test`**：量前后/左右/上下，用来改 `head_yaw_offset_deg` / 轴映射，不要凭感觉乱翻

---

## 基站挪了，世界系变了，手感会变吗？

**会，但通常只影响水平方向手感，不是整条链路坏掉。**

### 为什么大多还能用

本方案是 **相对遥操**：

1. **`head_base`**：手相对腰（`body`）计算；人带着两个 Tracker 一起走动时，臂不应跟着漂。
2. **离合相对增量**：踩踏板时冻结 VR / 臂基准，之后只跟 Δp、Δq。

因此基站重摆后，若只是 SteamVR **世界原点平移**（整屋坐标原点挪了），对上述两步基本无感，手感几乎不变。

### 真正会变手感的地方

基站重摆或重做 Room Setup 后，SteamVR **世界系绕竖直轴转了**（前后、左右相对房间 / 相对机械臂变了）：

- 配置使用 `head_base_yaw: world` + `head_yaw_offset_deg`
- 「手往房间前方伸」对应的 teleop 轴可能与上次调参不一致
- 常见表现：**上下仍对**（重力 Y-up 较稳），**前后 / 左右又拧了**（与当初把 `head_yaw_offset_deg` 从 180 改到 0 是同一类问题）

腕部安装外参、离合逻辑本身一般 **不必** 因「只挪基站」重做。

### 实际怎么处理

基站位置大改或重标定光场后，跑一次：

```bash
cd /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/EE_controlhal/VIVE_control_openvr
python3 scripts/test_body_relative.py --axis-test
```

必要时只改 `config/teleop.yaml` 里的 **`head_yaw_offset_deg`**（前后左右一起反 ≈ 再加 / 减 180°），把位置轴对上即可。

---

## 常见问题

| 现象 | 处理 |
|------|------|
| `list_trackers` 全空 | SteamVR 未起；dongle 被 libsurvive 占；拔插 Watchman；见 `docs/steamvr_setup.md` |
| 一只 `OutOfRange` | 遮挡 / 朝向基站；换站姿 |
| 踏板无离合 | `./scripts/fix_pedal_udev.sh`；`probe_pedal.py` 核对 `KEY_*` |
| 位置轴反了 | 改 `config/teleop.yaml` 的 `head_yaw_offset_deg`（前后左右一起反 ≈ ±180°） |
| 挪基站后水平轴又拧了 | 见上文「基站挪了…」；跑 `--axis-test`，调 `head_yaw_offset_deg` |
| 姿态乱拧 / IK 失败 | 可临时把 `rotation_scale` 设回 `0.0` 只跟位置；再标定腕部外参 |
| `--real-arm` 连不上 | `teleop_up.sh`、灯条绿、`:50051`、`python3 -c "import airbot_py"` |
