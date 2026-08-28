# AIRBOT Play 臂（PICO 遥操）

合工作区里的臂侧：PICO 头显 → Python 中间层 → 官方 sidecar gRPC → CAN → AIRBOT Play 6 轴。  
**不走工作区 ROS topic。** 末端夹爪在配置里是 `enable_eef: false`，握把命令会被丢掉；灵巧手走旁边的 Revo2 链路。

合容器总启动顺序见上一级 [README.md](../README.md)。

## 结构

```text
airbot+hand-workspace/
  README.md                      本文件
  teleop_up.sh                   臂一键：CAN → 合容器 → sidecar → adb → 只读自检
  docker/
    airbot_up.sh                 拉起/停止 sidecar airbot-play-runtime
    compose.yaml                 旧 Jazzy 工作容器（已不用）
  EE_controlhal/
    PICO_control/                Unity APK（PicoTeleop）
    middle_interface/            UDP :15000 → 遥操流水线 → 命令层
      app.py
      config/airbot.yaml         真机
      config/default.yaml        dummy 空跑
    Robot_control_interface/
      backends/airbot_sdk.py     airbot_py → localhost:50051
      config/airbot_sdk.yaml
  Airbot_play/                   SDK wheel / 文档 / 数采（C++ deb 不进 Humble 镜像）
  ros2_play/                     臂 URDF（可视化）
  revo2_description/             仅 URDF 占位，真机手不在这里
```

```text
PICO USB + adb reverse tcp:15000
  → middle_interface/app.py          扳机 deadman，grip 原为夹爪（现忽略）
  → Robot_control_interface/airbot_sdk
  → gRPC localhost:50051
  → 容器 airbot-play-runtime (fsm_node)
  → can0
  → AIRBOT Play
```

容器内工作目录：`/home/ubuntu/airbot+hand-workspace`  
Python：合容器 PATH 里的 `/opt/airbot-venv`（`airbot_py 5.1.6`）。

## 启动顺序

在合容器已起来的前提下（`../bringup.sh`）。

1. **USB2CAN 插上，确认 `can0` UP**
   ```bash
   ip -br link show can0
   ```
2. **头显 USB 调试 + `adb reverse`（USB 不通最常见就是漏这一步）**
   PicoTeleop 往头显自己的 `127.0.0.1:15000` 发 TCP，必须转到电脑：
   ```bash
   adb devices
   adb reverse tcp:15000 tcp:15000
   adb reverse --list    # 应看到 UsbFfs tcp:15000 tcp:15000
   ```
   `teleop_up.sh` 会做这件事；头显重插/重启后转发会掉，要再跑一次。
3. **一键依赖（占宿主机终端，会只读连一次臂）**
   ```bash
   /home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh
   ```
   看机械臂灯条：**绿色呼吸**才能继续。白灯 = 驱动没接管；黄灯 = 要校零。不对就 `./teleop_up.sh restart`。
4. **头显里开 PicoTeleop**，面板 `127.0.0.1:15000`。可不戴，当追踪基站：
   ```bash
   adb shell am start -n com.egoarx.picoteleopue/com.unity3d.player.UnityPlayerActivity
   ```
5. **遥操进程（必须占着终端看日志）**
   ```bash
   docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
     airbot-revo-humble python3 app.py --config config/airbot.yaml
   ```
   空跑（臂不动）把 `airbot.yaml` 换成 `default.yaml`。

日志应对上：`ROBOT ADAPTER: REAL (COMMAND_LAYER)`、`[PICO] RX` 约 60 Hz、`Deadman armed after release`。

操作：右食指扳机按住跟随、松开停住；右 A 回零位；左 X 暂停。退出用 **Ctrl-C**。

```bash
./teleop_up.sh status
./teleop_up.sh stop          # 先停 app.py 再停 sidecar
./teleop_up.sh restart
```
