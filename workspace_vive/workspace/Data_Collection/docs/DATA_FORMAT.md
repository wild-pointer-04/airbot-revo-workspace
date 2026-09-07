# 数据长什么样

采集器每个 tick（默认 10 Hz）写一帧图像和机械臂状态。时间对齐靠 `host_monotonic_ns`（`time.monotonic_ns()`），不要用 LeRobot 自动生成的 `timestamp = 帧号 / fps`。

本文件只描述**原始落盘**。

## 一帧里有什么

- `observation.images.cam_right_wrist`：右腕 RGB，480×640（当前只采这一路）
- `observation.host_monotonic_ns`：长度 1 的 int64
- `observation.state.arm_joint`：6 个关节角（弧度）
- `observation.state.arm_ee_position`：末端 xyz（米，机器人基座系）
- `observation.state.arm_ee_orientation`：末端四元数 `[qx, qy, qz, qw]`
- `observation.state.arm_valid`：话题有效为 1
- `action`：与 `arm_joint` 相同，便于 LeRobot 训练接口

真机下机械臂状态由 ROS2 话题 `/arm/state` 送进采集器（VIVE 或 PICO 遥操发布），再写成上面这些字段。相机仍由采集器直接读。

## 落盘位置

```text
data/lerobot/<task>/
  meta/info.json
  data/**/*.parquet
  videos/observation.images.cam_right_wrist/**/*.mp4
```
