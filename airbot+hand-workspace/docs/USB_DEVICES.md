# USB / 设备节点对照（本机当前插法）

插拔后 `/dev/videoN`、`/dev/ttyUSBN` 会变号。配置里尽量用 **by-id / udev 别名**。

探测：

```bash
lsusb
ls -l /dev/v4l/by-id/ /dev/serial/by-id/ /dev/input/by-id/ /dev/revo2*
```

## 当前映射（2026-09-10 实测）

| 设备 | USB / 识别 | 稳定节点 | 用途 |
|------|------------|----------|------|
| LRCP V1080P | `05a3:9230` ARC Camera | `/dev/v4l/by-id/usb-LRCP_V1080P_LRCP_V1080P-video-index0`（现为 `video0`） | **全局相机** `cam_global` |
| 笔记本内置摄像头 | `0c45:64ab` Microdia | `/dev/v4l/by-id/usb-170428-_Integrated_Webcam_HD-video-index0`（现为 `video2`） | **不采** |
| Revo2 右手 | FTDI Dual RS232 | `/dev/revo2_hand_right` → `ttyUSB2` | 手 Modbus（`protocol_modbus_right.yaml`） |
| USB2CAN | DISCOVER Robotics | `/dev/ttyACM0` → 网络口 `can0` | AIRBOT 臂 |
| CM6K 踏板 | SayoDevice | `/dev/input/by-id/usb-SayoDevice_CM6K_*-event-kbd` | 左=臂离合，右=数采门控 |
| VIVE Watchman ×2 | `28de:2300` | SteamVR 占用 | tracker 接收 |
| CH340 ×2 | `1a86:7523` | `/dev/ttyUSB0`、`ttyUSB3` | 其它串口设备（非手/非臂） |

> `*-video-index1` 多为同摄像头的 metadata 节点，OpenCV 不要打开。

## 配置落点

| 设备 | 配置文件 |
|------|----------|
| 全局相机 | [`Data_Collection/collect/config/collect.yaml`](../Data_Collection/collect/config/collect.yaml) → `cameras.cam_global.device` |
| CM6K 踏板 | [`EE_controlhal/VIVE_control_openvr/config/teleop.yaml`](../EE_controlhal/VIVE_control_openvr/config/teleop.yaml) → `buttons.pedal.device` |
| Revo2 右手串口 | `manus-Revo2-Revo3/.../revo2_driver/config/protocol_modbus_right.yaml` → `port: /dev/revo2_hand_right` |

## 重插后怎么修

1. `ls -l /dev/v4l/by-id/` 确认 LRCP 的 `…-video-index0` 还在（by-id 名一般不变）。
2. 若换了另一台全局机，只改 `collect.yaml` 里 `cam_global.device`。
3. 手串口丢了：在容器内跑 `revo2_driver/setup/bootstrap_revo2.sh`，再 `check_revo2_setup.sh`。
4. 踏板无效：`VIVE_control_openvr/scripts/probe_pedal.py`，必要时重装 `docs/99-sayodevice-pedal.rules`。
