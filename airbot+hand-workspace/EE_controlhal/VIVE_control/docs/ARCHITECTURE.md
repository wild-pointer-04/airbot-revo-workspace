# Architecture

本模块提供可靠的 VIVE Tracker 位姿。`VIVE_control/app.py` 通过 `TrackerManager` 取两台 tracker（身上=HMD，手持=手柄），编成 `PicoTeleopPacket` 后走 PICO 同一套遥操。上层不接触 libsurvive。

```text
TrackerManager
       ↓
TrackerBackend
       ↓
 ┌───────────────┐
 ↓               ↓
MockBackend   LibsurviveBackend
       ↓
 reader thread → bounded queue + latest cache
       ↓
 TrackerPose  (T_world_tracker, raw)
       ↓
 bench_frame.compute_bench_hand
       ↓
 T_bench_hand  →  LeRobot vive_*
```

## 数据

`TrackerPose` 是唯一向上暴露的样本类型：

- `serial`：设备永久 ID（`LHR-XXXXXXXX` 或 Mock serial）
- `logical_name`：安装位置，例如 `dexterous_hand`
- `position_m`：米
- `orientation`：四元数 **`[w, x, y, z]`**
- `sensor_timestamp`：Sensor Capture Time，没有则为 `None`
- `host_monotonic_ns`：Host Receive Time（`time.monotonic_ns()`）
- `tracking_valid`
- `sequence_id`
- `source`：`mock` / `libsurvive` / `interpolated`

丢失 tracking 时必须 `tracking_valid=False`。可以另外保存 `last_valid_pose`，但不能把上一帧当成新的有效观测。超过 `bench.stale_s` 的缓存样本也会被标成无效。

## 世界坐标 vs 工作台系

`TrackerManager` 输出的是 libsurvive / Mock 给出的原始跟踪位姿 `T_world_tracker`。独立 CSV（`record_pose.py`）仍写这个。

正式采集把两台桌面 landmark（`bench_origin`、`bench_x`）钉在工作台上，每一帧：

```text
原点 O = p_A
X     = normalize(p_B - p_A)
Z     = up_hint 去掉沿 X 的分量
Y     = Z × X
T_bench_hand = T_world_bench^{-1} * T_world_hand
```

LeRobot 的 `vive_position` / `vive_orientation` 是 **T_bench_hand**，不是 lighthouse 世界系。基站移动并重新校准后，只要 A、B 没动，工作台坐标不变。

本模块仍然 **不** 实现 `VIVE World → Robot Base`。工作台系是采集约定，不是机械臂基座标定。

训练策略时再从已存的 `T_bench_hand` 序列算 UMI 式相对动作：

```text
T_rel = T_hand(t)^{-1} * T_hand(t+k)
```

`relative_pose()` 已提供。用同一个全局变换 `G` 左乘两帧后，相对位姿不变。

## Tracker → Hand 外参

未来会有固定结构件外参 `T_tracker_hand`。配置里预留 `extrinsics: null`。当前不猜测、不写死数值。`pose_utils.apply_extrinsics()` 在外参为 `None` 时原样返回。

## 滤波与插值

- 采集默认保存 raw pose。`filter.enabled: false`。
- 以后若加滤波，必须同时保留 `raw_pose` 和 `filtered_pose`。
- `interpolate_pose()` 只是工具：位置线性插值，姿态 **SLERP**。Live 管线不会自动插值。
- 插值结果 `source="interpolated"`，且 `sensor_timestamp=None`（不伪造传感器时间）。

## 时间

本模块不做多模态同步。它只提供 raw pose + 两种时间戳。采集器 `SyncBuffer` 用 `host_monotonic_ns` 做过期，不用墙钟刷新同一 `sequence_id`。

## 线程

Backend 有独立 reader thread。队列 `max_size` 有上限，满时丢掉最旧样本并增加 `queue_drop_count`。打印、写 CSV、diagnostics 都在 reader 之外。

## ROS 2

核心库不依赖 ROS 2。遥操入口 `app.py` 把两台 tracker 编成 `PicoTeleopPacket`，走 `middle_interface` 控臂，并由 `ArmStatePublisher` 把关节角 / 末端发到 `/arm/state`。数采只订这个话题。
