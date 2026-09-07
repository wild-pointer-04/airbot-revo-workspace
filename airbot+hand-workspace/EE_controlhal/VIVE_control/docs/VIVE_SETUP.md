# VIVE Setup (Linux + libsurvive)

正式跟踪路径：

```text
VIVE Tracker 3.0 + Base Station 1.0（本桌）+ Dongle
        ↓
libsurvive
        ↓
Python (pysurvive 或 ctypes)
        ↓
TrackerPose
```

不要把 SteamVR 当作日常采集依赖。SteamVR 只用于 Tracker-Dongle 配对或固件，用完关掉。

这张桌子用两台 **Base Station 1.0**（channel 0/1）。`tracker.yaml` 为 `--lighthouse-gen 1`、`--ootx-ignore-sync-error`，并禁用 LH 2–15。日志里的 `Using LH 0 (xxxxxxxx)` 是桌上 1.0 的 OOTX ID，8 位十六进制属正常。仓库里的 SteamVR `lighthousedb.json` 是历史 2.0 记录，不要加载。

## 安装

```bash
cd vive_tracking_linux
./scripts/install_libsurvive.sh
./scripts/install_udev_rules.sh
```

默认源码目录：`~/third_party/libsurvive`。脚本不会删除已有目录。

Ubuntu 24.04 不要安装 `libatlas-base-dev`。

## 校准文件

第一次跑 libsurvive 时通常要做 Lighthouse 校准。配置一般在用户目录下，常见位置：

```text
~/.config/survive/
~/libsurvive/
```

以及源码树里的 config JSON。具体文件名随 libsurvive 版本变化，以 `survive-cli` 首次运行打印为准。

要求：

- 校准必须持久化
- **不要自动删除** config
- 本仓库的 Python 代码不会清理这些文件

手动备份示例：

```bash
mkdir -p ~/vive_backups/$(date +%F)
cp -a ~/.config/survive ~/vive_backups/$(date +%F)/ 2>/dev/null || true
find ~/third_party/libsurvive -name '*config*' -o -name '*.json' | head
```

如果 Base Station 被物理移动、重新摆放或更换，需要重新校准。旧校准会让位姿跳变或 `tracking_valid` 变差。重新校准前先备份旧文件，不要直接删。

## 配对

若 Tracker 无法被 Dongle 看到，才临时开 SteamVR 做配对，然后退出 SteamVR，再回到 libsurvive。
