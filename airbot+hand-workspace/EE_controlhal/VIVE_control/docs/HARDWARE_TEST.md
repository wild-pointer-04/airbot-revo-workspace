# Hardware tests (after Tracker arrives)

硬件未到货前不要把 Mock 结果当成真机结果。

## 第一天

```bash
cd /home/ubuntu/airbot+hand-workspace/EE_controlhal/VIVE_control
./scripts/install_libsurvive.sh
./scripts/install_udev_rules.sh
# 插入 Dongle；打开 Base Station 1.0 与 Tracker 3.0；关闭 SteamVR
./scripts/check_hardware.sh
./scripts/first_vive_test.sh
python3 scripts/list_devices.py
```

把两台真实 serial 写入 `config/tracker.yaml`（`body_hmd` / `hand_controller`），`backend: libsurvive`。

## Bench frame (three trackers)

A、B 锁在桌面上，手 tracker 固定相对桌子不动。重标定或等价于换世界系后：

- `scripts/record_pose.py` 的世界系 xyz 会跳
- 采集器 LeRobot 里的 `vive_position`（`T_bench_hand`）应仍在数毫米内

若世界系 +Z 不是桌面法向，改 `bench.up_hint`。

## Axis Test

手持 Tracker 分别：前 / 后 / 左 / 右 / 上 / 下。

```bash
python3 scripts/monitor_pose.py
```

记录 `x y z` 变化，确定真实 Lighthouse 坐标轴。不要事先猜测轴向。

## Rotation Test

分别绕 X / Y / Z 旋转，确认四元数连续（不应出现 `0.99 → -0.99` 的符号跳变）。

## Static Test

Tracker 固定 60 秒：

```bash
python3 scripts/run_diagnostics.py --duration 60
```

把 `diagnostics.check_static_stability` 设为 `true` 后再跑，关注：

- position std
- orientation jitter
- actual update rate
- dropouts

真实 Tracker 不一定是 100 Hz。配置里的 100 Hz 只是测试期望。

## Dynamic Test

慢速移动、快速移动、连续旋转：

```bash
python3 scripts/record_pose.py --output data/vive_dynamic.csv --duration 60
```

检查：

- timestamp 不逆序
- quaternion 无符号跳变
- tracking 状态正常
- CSV 正常写入
