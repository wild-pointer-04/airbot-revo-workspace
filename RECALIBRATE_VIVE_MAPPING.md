# VIVE 双 Tracker 坐标重新标定

本工程的有效源码和容器路径是：

```text
宿主机: /home/ubuntu/airbot-revo-workspace/workspace_vive/workspace
容器:   /home/ubuntu/workspace
```

旧文档中 `airbot+hand-workspace` 的命令与本次部署无关。

## 已修正的坐标链路

libsurvive、SteamVR 与 OpenXR 在此工程中共用 OpenXR Stage 基础：原始
`+X 右、+Y 上、-Z 前`。`vr_normalize.py` 先变为
`teleop_standard`：`+X 前、+Y 左、+Z 上`。

问题出在后一级：AIRBOT SDK 笛卡尔命令的实测基础是 `+X 下、+Y 左、+Z 前`，原配置却把 `teleop_standard` 直接当作 SDK 基础使用。因此出现“上到前、前到下、左右正确”。默认映射现为：

```yaml
transform:
  axis_map:
    x: ["z", -1.0]
    y: ["y",  1.0]
    z: ["x",  1.0]
```

`libsurvive` 没有旧文档所称的额外隐藏转换；改 `steamvr` 为 `libsurvive` 不会单独修复该问题。

## 先绑定两只 Tracker

```bash
docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble python3 scripts/list_devices.py
```

当前设备可能以 `T20`、`T21` 这类临时别名出现。移动一只 Tracker 确认角色后，将它们分别填进 `config/tracker.yaml` 的 `body_hmd` 和 `hand_controller`。不要猜测两个 ID 的角色。稳定的 `LHR-...` ID 可用时应替换临时别名。

## 腰部 Tracker 安装

`body_hmd` 就是腰部 tracker，不是额外的头显。它用于消除操作者转身时的水平 yaw。配置 `teleop.head_tracker_to_body` 的含义是将腰部 tracker 的本地轴变成身体的 `前/左/上`。腰部 tracker 竖直安装时，默认 identity 通常不正确；确认 tracker 正面与本地轴后再修改这个右手坐标系旋转。

手腕 tracker 的安装姿态可以与腰部不同，例如水平、绿灯面朝上。它不改变 XYZ 平移的方向；相对姿态控制中的固定 tracker-to-hand 旋转外参也会抵消。只有绝对工具姿态或工具长度偏置需要把该外参接入数据链路。

## 三轴标定

先运行只读模式：

```bash
docker exec -it -w /home/ubuntu/workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble python3 scripts/calibrate_mapping.py
```

腰部 tracker 保持不动，按提示让手腕 tracker 分别上移、向机器人前移、向左移至少 30 cm。脚本采样主程序实际使用的 `vr_normalize -> HeadBase -> axis_map` 前的输出，并拒绝反射矩阵、重复轴和非单轴动作。

结果三项均通过后，重复命令并添加 `--write`。这只会改 `config/teleop.yaml` 的 `transform.axis_map`，不会连接机械臂。

## 验证顺序

1. 用 `--dummy-arm` 与真实 tracker 做三轴校验。
2. 验证 deadman 松开和任一 tracker 丢失都会停止流式目标，恢复后必须松开再按 deadman。
3. 按实际场地设置工作空间、速度、最大位置和最大角度后，再进行小位移真机测试。
