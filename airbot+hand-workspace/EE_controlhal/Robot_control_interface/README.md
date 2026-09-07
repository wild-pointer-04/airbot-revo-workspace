# Robot_control_interface

机械臂命令层。只吃 **基座系绝对笛卡尔目标**，再转成 ARX 能执行的 `/arm_cmd`。

不做 VR、head-base、离合。那些在 `middle_interface`。

```text
middle_interface RobotCommand (m + xyzw)
        │
        ▼
Robot_control_interface
        │  xyzw → xyzrpy，latest-wins @ 100 Hz
        ▼
ROS 2  /arm_cmd   mode=4 END_CONTROL
        │
        ▼
X5Controller  →  can1
```

松开扳机后不要切 `PROTECT`：本机走 `/arm_cmd` 的 mode=2 仍会掉。和官方键盘一样，停在 `mode=4`，100 Hz 重复最后一个 TCP。不要发 `mode=0 SOFT`。
重力补偿 / 回零走 `/arx_joy`，不要带全 0 目标的 `/arm_cmd`。
`mode=4` 跟踪时 `joint_pos` 填 0（和官方键盘一致）。不要回填当前关节，否则末端和关节目标会打架，运动一卡一卡。
命令层按 `slew_linear` / `slew_angular` 在 100 Hz 匀速逼近最新目标，QoS 只留 1 帧。

本机夹爪已拆，`enable_gripper: false`。

## 后端

| `backend` | 作用 |
|---|---|
| `dummy` | 默认。只记命令，不碰 ROS / CAN |
| `arx_ros2` | 订 `/arm_status`，发 `/arm_cmd` |

Python CAN SDK（`SingleArm`）和 ROS 控制器互斥，本层不用 CAN SDK。

## 先起机械臂

```bash
/home/dt/ego_ARX/bringup_single_arm.sh
# 另开终端确认状态
source /opt/ros/humble/setup.bash
source /home/dt/ego_ARX/ARX/ARX_X5-main/ROS2/X5_ws/install/setup.bash
ros2 topic echo /arm_status --once
```

## 命令层自检

```bash
cd /home/dt/ego_ARX/EE_controlhal/Robot_control_interface
./run.sh
# 或：
./run.sh tools/send_cart.py --xyz 0.25 0.0 0.15
```

`app.py` 若没 source ROS，会自动再 exec 一次带上 ROS 2 / X5_ws。`X5Controller` 必须已经在跑。

`send_cart.py` 会动真机。先读 `/arm_status` 的 `end_pos`，只做小位移。

## 和 VR 串起来

`middle_interface/config/default.yaml` 仍是 dummy。真机用：

```bash
cd /home/dt/ego_ARX/EE_controlhal/middle_interface
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source /home/dt/ego_ARX/ARX/ARX_X5-main/ROS2/X5_ws/install/setup.bash
# 15000 只能有一个进程；先停 pose_table
python3 app.py --config config/arx.yaml
```

右食指扳机 = 开始跟踪；松开 = 停在最后 TCP（继续 mode=4）。右手 A = 回 `go_home_position`（当前六轴 0 位）。回零时会清掉旧的笛卡尔流式目标，回零完成后重新从实际 TCP 接管；改完后必须重启 `app.py`。
若仍觉得慢或顿，只调 `Robot_control_interface/config/arx_ros2.yaml` 的 `slew_linear`（m/s）、`slew_angular`（rad/s）。

若「前 / 上」反了，只改 `middle_interface` 的 `transform.axis_map`，不要改本层。

## 测试

```bash
cd /home/dt/ego_ARX/EE_controlhal/Robot_control_interface
python3 -m pytest -q
```
