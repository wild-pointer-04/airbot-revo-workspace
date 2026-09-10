# 数据长什么样

采集器每个 tick（默认 10 Hz）写一帧：**图像 + 机械臂状态 + Revo2 手反馈（含触觉）**。  
时间对齐靠 `host_monotonic_ns`（`time.monotonic_ns()`），不要用 LeRobot 自动生成的 `timestamp = 帧号 / fps`。

本文件只描述**统一落盘**（`collect/collect.py`）：图像 + 臂 + Revo2 手。

## 一帧里有什么

- `observation.images.cam_global`：全局 RGB，480×640（LRCP V1080P，OpenCV；设备用 `/dev/v4l/by-id/…`）
- （可选）`observation.images.cam_right_wrist`：右腕相机，接入后在 `collect.yaml` 启用
- `observation.host_monotonic_ns`：长度 1 的 int64
- `observation.state.arm_joint`：6 个关节角（弧度）
- `observation.state.arm_ee_position`：末端 xyz（米，机器人基座系）
- `observation.state.arm_ee_orientation`：末端四元数 `[qx, qy, qz, qw]`
- `observation.state.arm_valid`：臂话题有效为 1
- `observation.state.hand_right_joint`：6 个 Revo2 关节角（按驱动名重排）
- `observation.state.hand_right_velocity`：6 个关节速度
- `observation.state.hand_right_valid`：手反馈有效为 1
- `observation.state.hand_right_tactile`：5 个指尖法向力（N；thumb/index/middle/ring/pinky）
- `action`：与 `arm_joint` 相同，便于 LeRobot 训练接口

真机下臂状态来自 `/arm/state`，手关节来自 `/revo2_right/revo2_joint_state/joint_states`，触觉来自同命名空间的 `dynamic_joint_states`，均经 rosbridge 进入采集器。相机由采集器本地 OpenCV 直读。

录制门控默认订阅 `/teleop/hand_recording`（VIVE 右踏板）；也可键盘 Space / `s` / `d` / `q`。

USB 端口对照见 [../../docs/USB_DEVICES.md](../../docs/USB_DEVICES.md)。

## 落盘位置

```text
data/lerobot/airbot_revo2_right/
  meta/info.json
  data/**/*.parquet
  videos/observation.images.cam_global/**/*.mp4
```
