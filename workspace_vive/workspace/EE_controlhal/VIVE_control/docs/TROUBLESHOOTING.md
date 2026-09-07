# Troubleshooting

## Mock 正常，libsurvive 起不来

```text
Libsurvive backend requested but libsurvive
is not available.

Please run:

./scripts/install_libsurvive.sh

Then rerun the program.
```

下一步：安装脚本 → udev → 重新插拔 Dongle。

## `libatlas-base-dev` 无法安装

Ubuntu 24.04 上 `liblapacke` 与旧版 ATLAS 冲突。本仓库安装脚本不使用 ATLAS，改用 `libopenblas-dev`。

## `No VIVE Tracker detected.`

这是正常退出，不是 traceback。检查：Dongle、Tracker 开机、Base Station 开机、SteamVR 已关闭、`backend` 与 serial 配置。

`check_hardware.sh` 看到的 USB VID/PID 对不上，**不能**据此断言设备不存在。不同 Dongle 描述字符串差异很大。

## 空间里同时有 1.0 和 2.0 基站

本桌是两台 1.0：`--lighthouse-gen 1`、`--ootx-ignore-sync-error`，禁用 LH 2–15。`Using LH 0 (8d9a5499)` 在只有桌上 1.0 运转时就是本桌基站 ID，不是串台。不要加载 SteamVR `lighthousedb.json`。校准是否可用看 `vive_valid` 和 Global solve 误差是否降下来，不要看这个 hex。

## SteamVR 与 libsurvive 抢设备

关闭 SteamVR 后再跑 `survive-cli` / Python。

## 位姿乱跳或突然全 invalid

Base Station 可能被挪过，需要重新校准。先备份 `~/.config/survive/`，不要让程序删 config。

## Python 3.12 / pysurvive

pysurvive 需要从 libsurvive 源码 `pip install .` 编译。系统包或旧 wheel 可能对 3.12 不兼容。本模块在 pysurvive 不可用时会尝试 `ctypes` 加载 `libsurvive.so`。若两种都失败，不会静默崩溃，而是给出安装提示。

## 四元数看起来跳到负号

`q` 与 `-q` 是同一旋转。本模块会做符号连续化。若 CSV 仍跳变，开 issue 时附带前后两帧 `qw qx qy qz`。
