# VR 遥操作到机械臂控制：实现总结与复用指南

> 范围：`lizhen_ros2_ws/src` 中当前真正跑通的主链路。
> 目标：把 Quest VR 信号如何变成机械臂运动讲清楚，方便其他项目整层复用，而不是只抄配置。
> 对照设计稿：`vr_to_robot_command_design.md`、`design_notes.md`。本文以**代码实现**为准。

---

## 1. 结论先看

当前主链路是：

```text
Quest App (deeptouch-xr)
  → deeptouch_teleop（收包 + 坐标变换 + 增量控制）
  → robot_hal（TCP/protobuf）
  → Apex 在线跟踪（流式绝对笛卡尔目标）
```

不走 ROS2 topic。仓库里还留着一条旧 ROS2 随动路径（`apex_arm_controller/TrackingManager`），和这条**互斥**，复用时不要混用。

职责边界：

| 层 | 负责 | 不负责 |
|---|---|---|
| VR driver | 解析设备原始帧 | 机器人坐标系、IK |
| teleop 算法 | 归一化、head-base、增量、按键 | 机型列表、伺服 |
| HAL spec | `teleop_to_command`、限幅、当前 TCP | VR 协议 |
| 执行器 | 把绝对笛卡尔目标喂给控制器 | 遥操作语义 |

换 VR 只换设备层；换机械臂只换 HAL spec 和执行器。

---

## 2. 五层流水线

```text
Quest App (TCP Server :40051, 140B @ 72~120Hz)
    │  raw OpenXR grip pose + 按键
    ▼
① VR 设备层   device/vr/
    │  TeleopData
    ▼
② 坐标层      transform/
    │  OpenXR → teleop_standard → head-base 相对位姿
    ▼
③ 控制层      control/
    │  deadman / baseline / 增量 / 绝对笛卡尔指令
    ▼
④ HAL 抽象    RobotHalInterface
    │  TCP protobuf :7860
    ▼
⑤ 执行层      robot_hal → Apex
       ONLINE_TRACKING 流式喂目标位姿
```

对应代码：

| 层 | 关键文件 |
|---|---|
| ① | `device/vr/base.py`、`device/vr/quest/{types,receiver,device,filter,adb}.py` |
| ② | `transform/{vr_normalize,head_base,frame_adapter,pose}.py` |
| ③ | `control/{control_loop,teleop_controller,button_mapping,vr_button_handler}.py` |
| ④ | `hal/{interface,real_client,mock}.py` |
| ⑤ | `deeptouch_robot_hal/.../robot_apex.py` |

入口：`deeptouch_teleop/main.py`

- 只收 VR：`python main.py`
- 全链路 mock：`python main.py --teleop --hal mock -v`
- 真机：`python main.py --teleop --hal real`

---

## 3. VR 信号从哪来

Quest 上的 `com.deeptouch.xrteleop` 当 **TCP Server**，PC 当 Client。

| 模式 | 连接 | 说明 |
|---|---|---|
| `local` | USB + ADB `forward tcp:40051 tcp:40051`，连 `127.0.0.1:40051` | 自动拉起 App |
| `remote` | Wi-Fi 直连 Quest IP:40051 | App 需人手先开 |

配置见 `config/vr_teleop.yaml`。

### 3.1 帧协议

文件：`device/vr/quest/types.py`

- 定长 **140 字节**，无分隔符，按 `sizeof` 切片
- 末 4 字节 **CRC32**，错了整帧丢掉
- 坐标系：**OpenXR Stage**，`+X 右`、`+Y 上`、`-Z 前`
- 四元数：**`qw, qx, qy, qz`**（w 在前）
- **没有**旧 spiritxr 的 `Rgo_` 预旋转，发的是 raw controller grip pose

| 偏移 | 字段 |
|------|------|
| 0 | `flags`（bit0 = 有手柄） |
| 4 / 32 / 60 | head / left / right：`x,y,z,qw,qx,qy,qz` |
| 88 / 108 | 左右 Keys：`one,two,menu,thumb_btn` + `hand_trigger,index_trigger,thumb_x,thumb_y` |
| 128 | timestamp (`double`) |
| 136 | CRC32 |

按键语义：

| 字段 | 含义 |
|---|---|
| `one` | 左 X / 右 A |
| `two` | 左 Y / 右 B |
| `index_trigger` | 食指扳机，死人开关 `[0,1]` |
| `hand_trigger` | 侧握，夹爪 `[0,1]` |
| `thumb_btn` | 摇杆按下 |
| `thumb_x/y` | 摇杆 `[-1,1]`（底盘映射尚未接到主循环） |

### 3.2 接收架构（建议原样抄）

文件：`device/vr/quest/receiver.py`

1. **recv 线程**：纯 I/O，`TCP_NODELAY`，切 140B 入队
2. **process 线程**：CRC → 跟踪丢失检测 → 可选滤波 → 覆盖 latest snapshot
3. 控制环只读 **最新一帧**，不消费队列

跟踪丢失：手柄 pose 全 0。检测到后重置该侧滤波器，并设 `left/right_tracking_lost`，控制层该臂本帧不算增量。

滤波（当前 yaml 里 **`enable_filter: false`**）：

- 位置 OneEuro（`min_cutoff=5, beta=0.1, d_cutoff=20`）
- 姿态 Slerp EMA（`quat_cutoff=5`）
- 实际打在 **OpenXR 世界系** 上，不是设计文档说的 head-base 相对系之后

`config/vr_teleop.yaml` 里的 `hand_fixed_rot` / `head_fixed_rot` **代码未使用**，不要当成生效标定。

---

## 4. 坐标变换（算法本体）

内部约定：

- 空间：`teleop_standard`，`+X 前`、`+Y 左`、`+Z 上`
- 四元数内部：`[w, x, y, z]`
- 只在边界换序：VR 入 `wxyz`，HAL 出 `xyzw`

### 4.1 OpenXR → teleop_standard

文件：`transform/vr_normalize.py`

```text
x_t = -z_xr     # 前
y_t = -x_xr     # 左
z_t =  y_xr     # 上

qw_t =  qw_xr
qx_t = -qz_xr
qy_t = -qx_xr
qz_t =  qy_xr
```

这一步只跟 VR 设备约定有关。新设备如果已经是 Z-up，只换这个函数。

### 4.2 Head-base（直觉映射的关键）

文件：`transform/head_base.py`

从头显取出 **只含 yaw** 的参考系：低头/歪头不改变「前方」。

```text
forward = 头局部 +X 投到水平面
up      = [0, 0, 1]
left    = up × forward
R_head_base = [forward | left | up]
origin      = 头位置
```

然后：

```text
T_hand_rel = inv(T_head_base) × T_hand
```

人在房间里走动、转身，只要手相对身体不动，`T_hand_rel` 就基本不变。  
「往前推手 → 机器人往前」靠的就是这个，而不是手柄在房间里的绝对坐标。

### 4.3 增量（相对遥操作，不是绝对跟随）

文件：`control/teleop_controller.py`、`control/button_mapping.py`

死人开关：食指扳机迟滞 **按下 >0.5 / 松开 <0.3**。

按下上升沿：

1. 立刻记下 **机器人当前笛卡尔位姿** `robot_baseline`（`hal.get_state()`）
2. **不立刻**记 VR 位姿。随后 `set_motion_mode(ONLINE_TRACKING)` 会同步阻塞（等臂停稳 + enable tracking，可能数秒）。若此刻冻 VR，手还会动，第一帧会跳。
3. 模式切完后的后续帧，再用 **新 VR 相对位姿** 配上已存的 `robot_baseline` → `finish_capture`

持续按住：

```text
Δp_teleop = p_now - p_baseline               # head-base 系
Δq        = q_now * conj(q_baseline)         # 世界/head-base 系左乘
```

实现是 **世界系相对旋转**（`Δq * robot_baseline`），不是设计文档里的体轴相对（`q_base⁻¹ * q_now` 再右乘）。复用时按实现走：手腕在空间里怎么转，末端在基座系里怎么转。

安全限幅（来自 HAL spec，默认）：

- 线位移 **0.30 m**
- 角位移 **1.0 rad**（超了就 slerp 截断）

### 4.4 转到机器人命令系，叠成绝对目标

文件：`transform/frame_adapter.py`

```text
Δp_cmd = R_cmd_teleop @ Δp_teleop
Δq_cmd = q_R * Δq * q_R⁻¹

cmd_pos  = robot_baseline.pos + Δp_cmd
cmd_quat = Δq_cmd * robot_baseline.quat      # 左乘 = 基座系旋转
```

输出给 HAL（四元数变成 **xyzw**）：

```python
[x, y, z, qx, qy, qz, qw]
```

注意：`send_cmd()` 吃的是 **机器人命令坐标系下的绝对目标位姿**，不是原始 VR delta。

当前 Apex 的 `get_teleop_spec()` 里 `teleop_to_command` 是 **identity**。也就是假定：

> `teleop_standard` 的前/左/上 ≈ 机器人命令系（通常是 base）的 X/Y/Z。

换机型如果基座轴不一致，**不要改 teleop 算法**，改 HAL 的 `teleop_to_command`。`controller_to_ee` 目前也是单位阵，且相对姿态模式基本用不到；末端朝向不对时再在 HAL 里给 tool offset。

---

## 5. 控制环：信号变成机械臂动作

文件：`control/control_loop.py`

固定频率（CLI 默认 `--control-hz 100`）：

1. 连 HAL，`acquire_control(TELEOP)`
2. 新 `frame_id` 才处理；同帧就 sleep，避免空转
3. `controller.update(frame)` → 笛卡尔指令
4. 处理 A/X/B/夹爪等附加按键
5. 根据扳机切换运动模式
6. `hal.send_cmd(cmd)`

运动模式是机械臂真正动起来的开关：

| VR 状态 | HAL 模式 | 执行器行为 |
|---|---|---|
| 未按扳机 | `TRAJECTORY` | 不跟 VR；可 `move_j` 回零 |
| 按住扳机 | `ONLINE_TRACKING` + Cartesian | 每帧 `update_tracking_pose` |
| 按住 A/X 回零 | 强制回 `TRAJECTORY` | 发关节回零，并 latch，直到松开扳机 |

归位后必须 **松开食指再按**，才会重新抓 baseline。否则会用运动中的过渡位姿当基准，一恢复跟踪就跳。

### 5.1 按键一览

文件：`control/vr_button_handler.py`

| 输入 | 行为 |
|---|---|
| 左/右食指扳机 | 该臂遥操 deadman |
| 左/右侧握 | 夹爪比例开合（≥0.85 视为全闭） |
| 右 A | 右臂 + 右爪回 home |
| 左 X | 左臂 + 左爪回 home |
| 右 B | 开始/停止录数据 |
| 右摇杆按下 | 保存 episode |

### 5.2 夹爪两条互斥路径

| 启动方式 | 路径 | 单位 |
|---|---|---|
| 不带 `--grippers` | cmd 里带 `*gripper_cmd_pos`，走 HAL | 米，`(1 - closure) * max_pos` |
| 带 `--grippers` | 串口直控 4B4C，并从 cmd 删掉 HAL 夹爪字段 | 开口度 0–1000 |

串口实现：`control/gripper_controller.py`。端口绑定优先 udev symlink，其次 USB 物理口。

---

## 6. HAL 之后机械臂怎么跟

通信：TCP `127.0.0.1:7860`，4 字节长度头 + protobuf。

`send_cmd` 进 `robot_hal` CommandPipeline（固定 Hz，**只发最新一条**，中间积压丢掉）。

`ApexRobotActuator`（`robot_apex.py`）：

| 模式 | 笛卡尔指令 | 关节指令 |
|---|---|---|
| `ONLINE_TRACKING` | `[x,y,z,qx,qy,qz,qw]` → Euler XYZ 度 → `update_tracking_pose(..., reference_frame=1)` | `update_tracking_joints`（遥操默认不用） |
| `TRAJECTORY` | `move_l` | `move_j` |

切到跟踪时会：清故障 → `trajectory_stop` → **等臂停稳** → 一次性 enable tracking。这就是 baseline 必须延迟捕获的原因。跟踪失败会闩死，后续命令静默丢弃，直到复位。

控制权：`IDLE=0 / TELEOP=1 / POLICY=2`，TELEOP 与 POLICY 互斥。`send_cmd` 需要持有 token。

当前 `get_teleop_spec()` 默认值（`session.py`）：

```text
pose_format:     m, xyzw
teleop_to_command: identity
controller_to_ee:  identity（左右）
limits:          0.30 m / 1.0 rad / max_gripper_pos（有夹爪则取最大行程，否则 0.10 m）
```

---

## 7. 旧 ROS2 路径（不要和主链路混用）

`apex_arm_controller/TrackingManager` 订阅 `/teleoperation/{left,right}_cmd`，把 **相对 delta 旋到当时 TCP 上**：

```text
target_pos = base_tcp + R_tcp * Δp     # EE 系增量
target_rot = R_tcp * Δq
```

这和主链路的 **基座系绝对目标** 不是同一套数学。

`deeptouch_bringup` 已写明：`enable_arm` 的 ROS2 臂节点和 `robot_hal` 都会占控制器 TCP 9100 / UDP 9101，**互斥**。本 workspace 里 `deeptouch_vr` 包已不在，主路径就是 Python teleop + HAL。

---

## 8. 其他项目怎么复用

建议整包拷这些目录，而不是抽单文件：

```text
deeptouch_teleop/
  device/vr/          # 可换 quest → 你的设备
  transform/          # 算法，尽量原样
  control/            # 状态机，按硬件微调按键
  hal/interface.py    # 合同
```

### 8.1 换 VR 设备

实现 `VRDeviceBase`：`start / stop / get_frame / get_frame_id`。输出尽量对齐 `TeleopData`。

新设备坐标系不同，只改 `openxr_to_teleop_standard`：把设备轴映到 `+X前 +Y左 +Z上`。后面 head-base / delta 不用动。

帧协议可以完全不同，但控制环依赖：

- 最新帧 snapshot + 单调 `frame_id`
- 跟踪丢失标志
- 左右手 pose + `index_trigger` / `hand_trigger`

### 8.2 换机械臂

实现 `RobotHalInterface`，至少：

```text
connect / close
acquire_control(TELEOP) / release_control
get_state() → left/rightarm_state_cart_pos   [x,y,z,qx,qy,qz,qw]
send_cmd()  → left/rightarm_cmd_cart_pos     绝对笛卡尔，米 + xyzw
get_teleop_spec()
set_motion_mode(TRAJECTORY | ONLINE_TRACKING)
move_to_pose()                               # 回零可选
```

`get_teleop_spec()` 是机型适配点：

- `teleop_to_command`：标准空间相对基座转了多少
- `limits`：单帧最大线/角速度
- `max_gripper_pos`：夹爪行程

执行器必须能吃 **流式绝对笛卡尔目标**（类似伺服 CSP / 在线跟踪）。如果只有规划 `move_l`，100Hz 会卡、会跳，不能当遥操。

### 8.3 联调顺序

1. `python main.py`：只收 VR，确认 72~120Hz、CRC、pose 非零
2. `python main.py --teleop --hal mock -v`：按扳机看 `L_cmd` 是否随手移动，松开是否停
3. 真机只读 `get_state`，确认命令系轴方向
4. 小幅度改 `teleop_to_command`，校正「前/上」
5. 再开 `ONLINE_TRACKING`

---

## 9. 复用时必踩的坑

1. **输出是绝对位姿，不是 VR delta。** 执行器不要再叠一层 EE 增量。
2. **四元数边界要换序**：VR 入 `wxyz`，HAL 出 `xyzw`，Apex 再转 RPY 度。
3. **baseline 必须延迟捕获**，否则模式切换阻塞会造成第一帧大跳。
4. **归位和跟踪互斥**：回零走轨迹，遥操走跟踪；归位 latch 未清时禁止重新 enable tracking。
5. **CommandPipeline latest-wins**：上游别比执行器更快地狂发，中间帧会被扔掉。
6. **限幅是安全网**，不是工作空间限制；丢帧重连后第一帧靠它削跳变。
7. 设计文档里「滤波放在 head-base 之后」**尚未落地**；真要平滑，建议滤 `hand_rel` 而不是世界系 pose，否则转身会拖尾。
8. 当前 `teleop_to_command = I`，只在「标准空间 ≈ 机器人 base」时成立。
9. yaml 里的 `hand_fixed_rot` / `head_fixed_rot` 未接入代码。
10. 设计文档的姿态增量公式与实现不一致；以 `teleop_controller.py` 的世界系左乘为准。

---

## 10. 一句话算法

**食指按下瞬间冻结机器人 TCP；之后每一帧用「手柄相对操作者水平朝向」的位移/转角，经机型旋转后加到那个冻结 TCP 上，作为笛卡尔伺服目标。松开扳机停止更新，机械臂停在最后位置。**

这套和具体 Quest 协议、Apex SDK 解耦：新项目只要提供同等含义的手部位姿 + 绝对笛卡尔跟踪接口，就可以直接复用 `transform/` 和 `TeleopController`。

---

## 11. 相关文档

| 文件 | 内容 |
|---|---|
| `docs/vr_to_robot_command_design.md` | 原始设计（部分与实现有差异） |
| `docs/design_notes.md` | 架构决策、Quest 协议确认 |
| `docs/implementation_plan.md` | T1–T4 实施计划 |
| `docs/PROGRESS.md` | 进度记录 |
| `deeptouch_robot_hal/docs/` | HAL 协议、部署、故障排查 |
