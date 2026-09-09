# VIVE 双 Tracker 快速校验

实际运行的工程在 `workspace_vive/workspace`，容器挂载到
`/home/ubuntu/workspace`。不要按旧文档中的 `airbot+hand-workspace` 路径启动。

当前平移坐标链路为：

```text
libsurvive/OpenXR -> teleop_standard(前/左/上) -> AIRBOT SDK(前/左/上)
                                      +X/+Y/+Z       +Z/+Y/-X
```

默认 `axis_map` 已落实为：

```yaml
x: ["z", -1.0]
y: ["y",  1.0]
z: ["x",  1.0]
```

先确认运行时识别到的两只 Tracker：

```bash
docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble python3 scripts/list_devices.py
```

把显示的两个实际 ID 写入
`workspace_vive/workspace/EE_controlhal/VIVE_control/config/tracker.yaml`。先移动或遮挡其中一只确认它是腰部 tracker 还是手腕 tracker；运行时别名如 `T20`、`T21` 不能自动判断角色，后续应替换为稳定的 `LHR-...` ID。

腰部 tracker 是配置中的 `vive.head`，用于人体水平朝向。它竖直绑在腰上时，先按实际安装方向设置 `teleop.head_tracker_to_body`，再运行三轴标定。手腕 tracker 水平、绿灯面朝上不会改变 XYZ 平移映射。

标定只读时不会改配置：

```bash
docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble python3 scripts/calibrate_mapping.py
```

三项验证都通过后，再加 `--write` 写入 `axis_map`。脚本不连接机械臂。

用真实 tracker 和虚拟臂验证：

```bash
docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble python3 app.py --config config/teleop.yaml --dummy-arm
```

确认 tracker 上/前/左分别对应机械臂的上/前/左，且松开 deadman、任一 tracker 丢失时均保持当前位置。真机前须先收紧位置、角度、速度和工作空间限制。
