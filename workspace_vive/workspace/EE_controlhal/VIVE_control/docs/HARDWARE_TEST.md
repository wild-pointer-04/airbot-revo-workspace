# Hardware tests (after Tracker arrives)

硬件未到货前不要把 Mock 结果当成真机结果。

## 第一天

```bash
cd /home/ubuntu/airbot-revo-workspace/workspace_vive/workspace/EE_controlhal/VIVE_control
./scripts/install_libsurvive.sh
./scripts/install_udev_rules.sh
# 插入 Dongle；打开 Base Station 1.0 与 Tracker 3.0；关闭 SteamVR
./scripts/check_hardware.sh
./scripts/first_vive_test.sh
python3 scripts/list_devices.py
```

把两台真实 serial 写入 `config/tracker.yaml`（`body_hmd` / `hand_controller`），`backend: libsurvive`。

## 腰部安装标定

`body_hmd` 就是绑在腰部的 tracker，不是额外的头显。它可以竖直安装；手腕 tracker 可以水平佩戴且绿灯面朝上，二者方向无需一致。把腰部 tracker 固定在腰带上，确保整个过程都能被基站看到。绿灯面不能用来推断 tracker 上报的局部坐标轴。

确认 Tracker ID 后，在实际运行的容器中执行以下只读标定。它不会启动遥操或发送机器人命令；传入 `--write` 后只替换 `teleop.head_tracker_to_body`：

```bash
docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble \
  python3 scripts/calibrate_body_mount.py --config config/teleop.yaml --write
```

采样顺序：先站直，采集手腕 tracker 相对腰部的起始位置；然后每次回到起始位置，保持躯干与腰部 tracker 姿势不动，分别将手腕 tracker 向身体正前、左方、上方平移至少 30 cm。每一步按回车后静止两秒。通过时得到 `body_vector = E @ tracker_vector` 的 `rotation_matrix`，其中 body 的坐标是前 / 左 / 上；运行时只使用它计算人体的水平 yaw。

重绑腰部或改变腰部 tracker 朝向时重跑本节。手腕 tracker 的固定朝向不改变当前 XYZ 平移映射，不需要重跑 `axis_map` 标定。只有基站布局、`vr_space` 或 AIRBOT 命令坐标定义变化时，才运行 `scripts/calibrate_mapping.py`；该脚本只更新 `transform.axis_map`，不会覆盖腰部安装标定。

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
