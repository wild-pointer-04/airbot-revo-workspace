# VR 遥操臂 + 手套遥操灵巧手：整套系统搭建教程

本文是 `airbot-revo-workspace` 的**学习型文档**：从宿主机基础环境讲到驱动控制、镜像配合、算法细节和排障，按「实际搭链路的顺序」组织。

和其它文档的分工：

| 文档 | 定位 |
|---|---|
| [README.md](README.md) | 速查：总启动顺序、路径映射 |
| **TUTORIAL.md**（本文） | 教程：每一层为什么这样设计、怎么搭、怎么排障 |
| [airbot+hand-workspace/README.md](airbot+hand-workspace/README.md) | 臂侧操作手册 |
| [manus-Revo2-Revo3/README.md](manus-Revo2-Revo3/README.md) | 手侧操作手册 |
| [manus-Revo2-Revo3/控制链路.md](manus-Revo2-Revo3/控制链路.md) | 手侧运维/验证记录 |

---

## 目录

- [第 0 章 全局地图](#第-0-章-全局地图这个工程到底是什么)
- [第 1 章 宿主机基础环境](#第-1-章-宿主机基础环境)
- [第 2 章 镜像与容器](#第-2-章-镜像与容器)
- [第 3 章 臂链路：PICO → AIRBOT Play](#第-3-章-臂链路pico--airbot-play)
- [第 4 章 手链路：MANUS → Revo2](#第-4-章-手链路manus--revo2)
- [第 5 章 合起来：完整开机到运动](#第-5-章-合起来完整开机到运动)
- [第 6 章 排障对照表](#第-6-章-排障对照表)
- [第 7 章 调参速查](#第-7-章-调参速查)
- [第 8 章 这份代码值得学的设计](#第-8-章-这份代码值得学的设计)

---

# 第 0 章 全局地图：这个工程到底是什么

## 0.1 两条互不相干的控制链

这个工作区最重要的一个设计决定：**臂和手是两条完全独立的链，只是恰好跑在同一台机器、同一个容器里**。

```text
链 A（臂）  PICO 头显 ── TCP:15000 ──► Python 中间层 ── gRPC:50051 ──► 官方驱动 ──► can0 ──► AIRBOT Play 6轴
链 B（手）  MANUS 手套 ── USB ──► ROS2 节点 ── ROS topic ──► ros2_control ──► Modbus 串口 ──► Revo2 灵巧手
```

它们没有任何共享的消息、时钟或状态机。链 A 完全不走 ROS（虽然它跑在 ROS 容器里，但只是借用 Python 环境）；链 B 完全在 ROS 2 Humble 里。

这不是偷懒，是有意的。[根 README](README.md) 里写得很直接：「两套控制链不要绑成一个 launch」。原因是耦合会带来两种事故：一是手侧节点崩了会拖着臂的 launch 一起退出，而臂在运动中被杀进程会污染驱动的接口状态；二是两侧调试节奏完全不同（臂要看灯条和 gRPC 自检，手要切 controller），绑在一起就没法单独重启一半。

## 0.2 容器拓扑

```text
宿主机 Ubuntu
├── airbot-revo-humble        ← 工作容器。你的代码都在这里跑
│     ROS 2 Humble + /opt/airbot-venv(airbot_py 5.1.6) + 两套源码挂载进来
├── airbot-play-runtime       ← sidecar。官方臂驱动，只做一件事
│     ros2 run fsm fsm_node -i can0 -p 50051
└── 硬件：can0 / /dev/revo2_hand_right / MANUS USB / PICO USB(adb)
```

> **名字别读错**：`airbot-revo` 是并列命名（AIRBOT 臂 **和** Revo 手，两套源码都在里面），**不是**"从 airbot 到 revo 的链路"——那条链不存在。详见 [2.1](#21-先搞清楚镜像名它记录的是合并历史不是数据流向)。

为什么臂驱动要单独一个容器？因为官方驱动镜像是 `airbot-runtime:5.1.6`，基于 Noble，里面的 `airbot_cpp` deb 装不进 Humble 镜像（[Dockerfile.combined:2-3](manus-Revo2-Revo3/Dockerfile.combined#L2-L3) 明确写了「Does not install the Noble airbot_cpp deb」）。所以官方驱动原样跑在自己的容器里，工作容器只装 Python 客户端 `airbot_py`，两者通过 **gRPC localhost:50051** 说话。因为两个容器都是 `--network host`，"localhost" 对它们是同一个网络栈。

这也是为什么 sidecar 叫 sidecar 而不是"第二套业务容器"——它没有你的代码，只是硬件的一层壳。

## 0.3 目录 ↔ 容器路径映射

这块最容易搞错。**不要在宿主机上 `source` 手侧的 `install/`**，因为 colcon 是 `--symlink-install`，install 里的路径都是容器内路径。

| 宿主机 | 容器内 |
|---|---|
| `airbot-revo-workspace/airbot+hand-workspace` | `/home/ubuntu/airbot+hand-workspace` |
| `airbot-revo-workspace/manus-Revo2-Revo3` | `/home/dt/manus-Revo2-Revo3` |

两个不同的 home 是历史原因（臂侧原来在 `ubuntu` 用户下开发，手侧在 `dt`），合并时保留了各自路径以免改一堆脚本。

---

# 第 1 章 宿主机基础环境

这一层是所有东西的地基，且**不在容器里**——容器起来之后再补这些会很痛苦。

## 1.1 需要装什么

```bash
# Docker + compose 插件
docker --version && docker compose version

# adb —— 给 PICO 头显用，必须在宿主机（容器内的 adb 看不到 USB 授权）
sudo apt-get install -y adb

# CAN 工具（宿主机侧排查用；容器里 Dockerfile.combined 也装了 can-utils）
sudo apt-get install -y can-utils iproute2
```

## 1.2 CAN 总线：臂的物理通道

AIRBOT Play 走 CAN，靠一个 USB2CAN 适配器（本机是 `0483:0000 DISCOVER Robotics USB to CAN adapter`）。

```bash
ip -br link show can0
# can0  UP  <NOARP,UP,LOWER_UP>   ← 必须是 UP
```

如果 `can0` 不存在或 DOWN：

- `lsusb | grep -i "USB to CAN"` 看适配器在不在
- 在的话是 udev 没把它拉成 `can0`，重插适配器
- 官方驱动 `fsm_node` 自己不会去 `ip link set up`，所以接口必须**先**是 UP 状态

[teleop_up.sh:107-115](airbot+hand-workspace/teleop_up.sh#L107-L115) 把这个检查放在整个启动流程的第 1 步，检查失败就直接 exit——因为后面每一步都依赖它。

## 1.3 Revo2 串口别名：手的物理通道

Revo2 走 Modbus RTU over USB 串口。**问题是 `/dev/ttyUSB*` 的编号会随插拔顺序变**，所以必须做 udev 别名。

```bash
# 一次性配置（脚本在驱动包里）
cd .../revo2_driver/setup
bash bootstrap_revo2.sh          # 自动探测 slave 126=左 / 127=右，写 udev 规则
bash check_revo2_setup.sh        # 健康检查
ls -l /dev/revo2_hand_left /dev/revo2_hand_right
```

约定死了的两个数字：

| 手 | slave_id | udev 别名 | 波特率 |
|---|---|---|---|
| 左 | 126 | `/dev/revo2_hand_left` | 460800 |
| 右 | 127 | `/dev/revo2_hand_right` | 460800 |

**这里有个坑值得单独说**：配置里必须 `auto_detect: false`。开了自动探测的话，双手启动时两个驱动进程会同时扫所有串口，互相抢占——一个抢到了另一个就连不上。

另外自动探测本身在某些机器上不可靠：`detect_revo2_ports.py` 在本机会因为 ttyACM 上 `termios.tcflush` 报 `Inappropriate ioctl for device` 而崩掉，而 CH340 串口（`1a86:7523`）对 Modbus 探测会回 ASCII 文本 `md err\r\n`，表现为日志里一串 `Invalid function code: 0x6D/0x64...`。看到这个报错就是探错了端口，不是手坏了。

## 1.4 MANUS USB 权限：为什么一切都要 root

MANUS 接收器是 USB vendor `3325`。它的 SDK 通过 HIDAPI/libusb **写** USB 设备节点，普通用户没有写权限。

症状很有迷惑性：普通用户跑 `manus_data_publisher` 会看到 `Manus Core connected`、甚至 `Dongle license data`，**但一条手套数据都没有**。看起来像是手套没配对，实际是权限。

工程里用了三重保险：

1. Dockerfile 写 udev 规则（[Dockerfile:72-77](manus-Revo2-Revo3/Dockerfile#L72-L77)）：
   ```
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"
   KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"
   ```
2. compose 里 `user: "0:0"` + `privileged: true` + 映射 `/dev` 和 `/run/udev`
3. 所有手侧命令都用 `docker exec -it -u 0`

第 3 条是你每天要记住的：**进容器跑手侧的东西，必须 `-u 0`**。忘了这个就会浪费半小时怀疑手套坏了。

实测记录（[控制链路.md](manus-Revo2-Revo3/控制链路.md)）：Dongle `0x7E465340`，右手 `0xC6142F4C`，左手 `0xDF470310`，设备节点 `/dev/bus/usb/001/023`。

## 1.5 PICO 头显：adb 与 USB 调试

这是整条臂链路**最常出问题的一环**，而且症状是"什么都不动，也没报错"。

原理：Unity APK 往**头显自己的** `127.0.0.1:15000` 发数据。头显和电脑之间没有网络，所以要靠 `adb reverse` 把头显的 loopback 端口转发到电脑上。

```bash
adb devices                          # 应看到一台 device
adb reverse tcp:15000 tcp:15000
adb reverse --list                   # 应看到 UsbFfs tcp:15000 tcp:15000
```

两个必须知道的点：

**（1）`adb reverse` 只转 TCP，不转 UDP。** 这直接决定了协议设计。看 [UdpTeleopSender.cs:34-38](airbot+hand-workspace/EE_controlhal/PICO_control/pico_teleop_unity/Assets/PicoTeleop/Scripts/Network/UdpTeleopSender.cs#L34-L38)：

```csharp
_useTcp = IPAddress.IsLoopback(_endpoint.Address);
```

发送端自己判断——目标是 loopback 就走 TCP（换行分隔的 JSON 流），是别的 IP（WiFi 模式）就走 UDP（一包一个 JSON datagram）。接收端 [udp_receiver.py](airbot+hand-workspace/EE_controlhal/middle_interface/receiver/udp_receiver.py) 干脆在同一个 15000 端口上**同时** bind UDP 和 TCP，两边都收，所以两种模式都不用改配置。这个类名叫 `UdpReceiver` 但其实是 UDP+TCP，是历史遗留。

**（2）转发会掉。** 头显重插、重启、息屏唤醒之后 `adb reverse` 就失效了，而且不报错——app 照样在发，只是发到了头显自己的空端口。所以 [teleop_up.sh:184-186](airbot+hand-workspace/teleop_up.sh#L184-L186) 每次启动都先 `--remove-all` 再重建。

如果 `adb devices` 看不到头显，去头显里：设置 → 开发者 → 打开 USB 调试，并把 USB 连接方式设成「调试」（不是传输文件/仅充电），重插线后在头显里点「允许 USB 调试」。这个设置**会自己退回默认**，所以是常客。

---

# 第 2 章 镜像与容器

## 2.1 先搞清楚镜像名：它记录的是合并历史，不是数据流向

三个镜像名很容易读错，**先把这件事讲清楚，否则后面全是误解**：

| 镜像 | 构词法 | 真正含义 |
|---|---|---|
| `manus-revo:humble` | 链路式（输入 → 输出） | MANUS 手套 **遥操** Revo2 手 |
| `airbot-revo:humble` | **并列式（A + B）** | AIRBOT 臂 **和** Revo2 手，两套源码都在里面 |
| `airbot-runtime:5.1.6` | 官方镜像 | AIRBOT 官方臂驱动，厂商提供 |

两个名字的构词法**不一致**，这就是它读起来别扭的原因。

`airbot-revo` 里的 "revo" **不是**说"从 airbot 到 revo 有一条链"——这条链根本不存在。臂和手是两条完全独立的控制链（见 [0.1](#01-两条互不相干的控制链)）。这个名字的意思是"AIRBOT 那套 + Revo 那套，合在一个镜像里"。

按数据流向，实际存在的两条链是：

```text
PICO  →  AIRBOT 臂        （臂链，走 gRPC + CAN，不走 ROS）
MANUS →  Revo2 手         （手链，走 ROS 2 + Modbus）
```

所以更贴切的名字应该是 `airbot-manus-revo` 或者 `teleop-workspace`。**这个镜像名起得不好，它记录的是"往 revo 镜像上叠了 airbot"这个合并动作，而不是任何数据流。** 知道这一点就不会再被它误导。

顺带说明：不存在 `airbot-pico` 这样的镜像。PICO 侧的代码（Unity APK）跑在头显上，电脑侧只是一个 Python 进程，不需要单独镜像。

## 2.2 为什么会长成这样：合并前是两个容器

这套东西原来是**两个完全独立的容器，连 ROS 版本都不一样**：

```text
臂侧  airbot-hand-workspace:jazzy    ROS 2 Jazzy    容器名 airbot-hand-workspace
手侧  manus-revo:humble              ROS 2 Humble   容器名 manus-revo-humble
```

臂侧旧镜像名里的 "hand" 是当年"臂+手一起做"的规划，但实际上它只跑臂。痕迹至今还在：`airbot+hand-workspace/revo2_description/` 这个目录留着，README 里注明「仅 URDF 占位，真机手不在这里」。

**合并时基座选了 Humble，不是 Jazzy。** 原因是手侧的 `revo2_driver`（ros2_control + Stark SDK）已经编译验证过、真机跑通过，迁到 Jazzy 要重新验证一整轮；而臂侧其实**不需要 ROS**——它走 gRPC，只用 Python 和 `airbot_py`。[Dockerfile.combined:2](manus-Revo2-Revo3/Dockerfile.combined#L2) 的注释直接说明了这个选择：

> Extends the **verified** manus-revo:humble image.

所以合并方向是「臂侧叠加到手侧镜像上」，而不是反过来。新镜像名 `airbot-revo` = 在原来的 `revo` 上加了 `airbot`。

**旧容器的残留还在仓库里**，看到别困惑：

| 残留 | 是什么 |
|---|---|
| `airbot+hand-workspace/docker/Dockerfile` | 旧 Jazzy 臂侧镜像定义，`FROM osrf/ros:jazzy-desktop` |
| `airbot+hand-workspace/docker/compose.yaml` | 旧 Jazzy 臂容器，README 标注「已不用」 |
| `manus-Revo2-Revo3/docker-compose.yml` | 旧的纯手侧启动（指向 `manus-revo:humble`），现在不用 |
| `bringup_combined.sh` 里的 `OLD_ARM` / `OLD_HAND` | 清理逻辑：发现旧容器在跑就警告「会抢 /dev、CAN、gRPC、UDP」 |

保留清理逻辑是必要的——两套容器同时跑会抢同一批物理设备，而症状很难定位。

## 2.3 两层 Dockerfile 的分工

镜像是叠出来的，不是一个 Dockerfile 搞定：

```text
osrf/ros:humble-desktop
    │  Dockerfile  ← 手侧先建（历史上手侧曾独立使用这一层）
    │    ROS 2 控制栈、libusb/libudev、MANUS udev 规则、
    │    创建 dt 用户、COPY 源码、colcon build 四个手侧包
    ▼
manus-revo:humble          手侧链路镜像：MANUS → Revo2
    │  Dockerfile.combined ← 臂侧叠加上来
    │    can-utils、docker.io CLI、
    │    /opt/airbot-venv 装 airbot_py 5.1.6 wheel
    ▼
airbot-revo:humble         合镜像：AIRBOT 臂 + Revo2 手，两套源码都在
```

`manus-revo:humble` 现在**只是构建中间层**，不直接拿来跑容器。实际运行的一直是 `airbot-revo:humble`。

三个镜像的实际分工和体积：

| 镜像 | 大小 | 装了什么 | 谁用 |
|---|---|---|---|
| `manus-revo:humble` | 9.5 GB | ROS 2 Humble + 手侧四包编译产物 | 仅作底层，不直接跑 |
| `airbot-revo:humble` | 17.4 GB | 上面 + can-utils + airbot_py venv | **实际运行的工作容器** |
| `airbot-runtime:5.1.6` | 1.0 GB | 官方臂驱动（Noble 基座） | sidecar，只管 can0 |

构建顺序有依赖，[bringup_combined.sh:113-117](manus-Revo2-Revo3/bringup_combined.sh#L113-L117) 会检查底镜像存在，缺了就明确报错：

```bash
cd manus-Revo2-Revo3
docker build -t manus-revo:humble -f Dockerfile .            # 第一层
docker build -t airbot-revo:humble -f Dockerfile.combined .  # 第二层
```

**为什么 airbot_py 要装在 venv 里而不是系统 Python？** 因为 `airbot_py` 拉的依赖（pydantic、omegaconf、hydra-zen）和 ROS 2 Humble 的系统 Python 包会互相打架。放进 `/opt/airbot-venv` 并把它加到 PATH 最前面（[Dockerfile.combined:10-11](manus-Revo2-Revo3/Dockerfile.combined#L10-L11)），臂侧 `python3` 就用 venv，手侧的 ROS 节点走 `ros2 run` 用的是自己的解释器，互不干扰。

还有个细节：镜像里 `Dockerfile:91` 用 `COPY . /home/dt/manus-Revo2-Revo3` 把源码打进去**并且编译过一遍**，但运行时 compose 又把宿主机同一个目录挂载到同一个位置覆盖掉。这看起来多余，实际有用——镜像自带一份编译好的 `install/`，导出成 tar 到别的机器上就能直接跑，不用重新 colcon build。离线包就是这么用的：

```bash
docker load -i /home/ubuntu/airbot-revo-humble.tar.gz
```

## 2.4 compose 里那些危险 flag 各自为了什么

[docker-compose.combined.yml](manus-Revo2-Revo3/docker-compose.combined.yml) 开了一堆平时不该开的东西。逐个说清楚它们是为谁开的，你才知道哪些能收紧：

| 配置 | 为了谁 | 能不能去掉 |
|---|---|---|
| `user: "0:0"` | MANUS HIDAPI 写 USB | 不能，见 1.4 |
| `network_mode: host` | gRPC 50051 到 sidecar；UDP/TCP 15000 收 PICO；ROS 2 DDS 发现 | 不能 |
| `privileged: true` | MANUS USB + CAN + 串口 | 理论上可以换成精确的 device 映射，但工程选了简单路线 |
| `ipc: host` | ROS 2 共享内存传输 | 去掉只会慢 |
| `pid: host` | `teleop_up.sh` 从宿主机管容器内的 `app.py` 进程 | 去掉会让 stop/restart 失效 |
| `/var/run/docker.sock` | 容器内能起 sidecar | 只在容器里执行 `airbot_up.sh` 时需要 |
| `/dev` `/run/udev` `/sys` | 串口别名、USB 枚举、CAN | 不能 |
| `ROS_DOMAIN_ID: 100` | 和机器上别的 ROS 系统隔离 | 建议保留 |

`pid: host` 有个反直觉的副作用，代码里专门处理了。[teleop_up.sh:36-42](airbot+hand-workspace/teleop_up.sh#L36-L42) 的注释：

> 容器是 PidMode=host，容器内 pgrep 能看到宿主机全部进程 —— 包括 `docker exec` 自己那条含 "app.py" 的命令行。所以用 pgrep 会咬到自己，必须按可执行文件名匹配（`-x python3`）再筛真正的脚本参数。

所以它用了 `ps -eo pid=,comm=,args=` 加 awk 精确匹配 `comm` 是 `python3` 且参数含 `app.py --config` 的进程。这种小细节是这个工程踩过坑的痕迹。

## 2.5 启动容器

```bash
/home/ubuntu/airbot-revo-workspace/bringup.sh --no-build
/home/ubuntu/airbot-revo-workspace/bringup.sh status
```

顶层 [bringup.sh](bringup.sh) 只有一行，转调手侧的 `bringup_combined.sh`。后者做四件事：

1. 检查并警告旧容器（`airbot-hand-workspace` / `manus-revo-humble`）——它们会抢 `/dev`、CAN、gRPC 端口
2. 构建镜像（`--no-build` 跳过）
3. 起工作容器（有 compose 用 compose，没有就用等价的 `docker run`）
4. 如果 `can0` UP 就顺手起臂 sidecar；没 CAN 就跳过并提示，**这样没有臂也能只跑手**

第 4 步这个设计很实用：手和臂可以分别独立调试，缺一个硬件不阻塞另一个。

停容器用 `bringup.sh stop`（sidecar 保留）。注意 `exit` 只退出 shell 不停容器，`docker compose down` 会**删**容器——日常用 `stop`。

## 2.6 多容器如何联合工作

这套系统同时跑两个容器，各有分工：

```text
┌─ airbot-revo-humble ────────────┐     ┌─ airbot-play-runtime ──────┐
│  你的代码                        │     │  官方臂驱动（不含你的代码）  │
│  ROS 2 Humble                   │     │  Noble + airbot_cpp        │
│  /opt/airbot-venv (airbot_py)   │     │  ros2 run fsm fsm_node     │
│                                 │     │    -i can0 -p 50051        │
│  app.py ──── gRPC :50051 ───────┼────►│                            │
│  手侧 ROS 节点 ──► /dev/revo2_* │     │         └──► can0 ──► 臂    │
└─────────────────────────────────┘     └────────────────────────────┘
        both: --network host, --privileged, -v /dev:/dev
```

**为什么必须拆开？** 官方驱动镜像基于 Ubuntu Noble，`airbot_cpp` deb 装不进 Humble 镜像。所以官方驱动原样跑在自己的容器里，工作容器只装 Python 客户端 `airbot_py`。这是"操作系统版本冲突"的标准解法——不要试图把两个发行版的包塞进一个镜像。

**协同靠三个机制**，缺一不可：

| 机制 | 作用 |
|---|---|
| `--network host`（两个容器都开） | 共享宿主机网络栈，`localhost:50051` 对它们是同一个地址。gRPC 能通的前提，也让 ROS 2 DDS 发现正常 |
| `-v /dev:/dev` + `privileged` | 两个容器看到同一套物理设备。工作容器摸串口和 MANUS USB，驱动容器摸 `can0`，分工不重叠 |
| `-v /var/run/docker.sock` + `docker.io` CLI | 工作容器内可以启动 sidecar。所以 `airbot_up.sh` 既能在宿主机跑也能在容器里跑 |

**冲突怎么避免？** sidecar 启动前三项互斥检查（[airbot_up.sh:92-99](airbot+hand-workspace/docker/airbot_up.sh#L92-L99)）：

```bash
pgrep -af 'X5Controller|arx_x5|bringup_single_arm'   # 旧 ARX 控制器在跑？会抢 CAN
ss -ltn "sport = :50051"                             # gRPC 端口被占？
docker inspect airbot-play-runtime                   # 已经有一个了？
```

任何一项命中就**拒绝启动**，而不是硬上去造成两个进程抢同一条 CAN 总线。同理 `bringup_combined.sh` 会检查旧的 `airbot-hand-workspace` / `manus-revo-humble` 容器，发现在跑就警告"会和合容器抢 /dev、CAN、gRPC、UDP"。

**生命周期是不对称的**，这点容易搞混：

| 操作 | 工作容器 | sidecar |
|---|---|---|
| `bringup.sh` | 起 | can0 UP 时顺带起 |
| `bringup.sh stop` | 停 | **保留** |
| `teleop_up.sh stop` | 不动 | 停 |
| `teleop_up.sh restart` | 不动 | 先杀 app.py，再重启 |

sidecar 是 `--rm` 的，停了就删，每次 `start` 都是全新进程。这是故意的——驱动的内部状态（controller 接口、CAN 握手）不适合复用，重启比清理干净。工作容器是持久的，停了再起不丢东西（编译产物和日志都在挂载的宿主机目录里）。

**必须知道的顺序约束**：重启 sidecar 前必须先停 `app.py`。SDK 客户端连着时重启驱动会留下悬挂的 gRPC 会话，新驱动起来后 controller_manager 的接口状态就是脏的，切模式必然失败。

## 2.7 日常怎么进容器干活

容器起来后是 `sleep infinity` 挂着的，所有工作通过 `exec` 进去做。**两侧的进入方式不一样**：

```bash
# 手侧 —— 必须 -u 0，MANUS SDK 要写 USB 设备节点
docker exec -it -u 0 airbot-revo-humble bash
source /opt/ros/humble/setup.bash
source /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/install/setup.bash

# 臂侧 —— 不需要 root，直接指定工作目录跑
docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
  airbot-revo-humble python3 app.py --config config/airbot.yaml
```

三个容易踩的点：

- `exit` 只退出 shell，**不停容器**。停用 `bringup.sh stop`；`docker compose down` 会**删**容器。
- 别在宿主机 `source` 手侧的 `install/`——colcon 是 `--symlink-install`，里面全是容器内路径。
- `/etc/profile.d/airbot-revo.sh` 会自动 source ROS 和手侧 overlay，但 `bash -c` 非交互模式不走 profile，脚本里要显式 source。

改了手侧源码要重新编译：

```bash
docker exec -it -u 0 airbot-revo-humble bash
cd /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting
source /opt/ros/humble/setup.bash
python3 -m colcon build --symlink-install --packages-select \
  manus_ros2_msgs manus_ros2 manus_revo2_retarget revo2_description revo2_driver
source install/setup.bash
```

臂侧是纯 Python，改完重启 `app.py` 即可。

---

# 第 3 章 臂链路：PICO → AIRBOT Play

现在从最底层往上搭。

## 3.1 底层：官方驱动 sidecar

```bash
docker run -d --rm --name airbot-play-runtime --network host --privileged \
  -v /dev:/dev -v /run/udev:/run/udev:ro -v /sys:/sys:ro \
  registry.cn-shanghai.aliyuncs.com/discover-robotics/airbot-runtime:5.1.6 \
  bash -lc "source /root/ros2_ws/install/setup.bash && exec ros2 run fsm fsm_node -i can0 -p 50051"
```

这是 [docker/airbot_up.sh:103-111](airbot+hand-workspace/docker/airbot_up.sh#L103-L111) 干的事。`fsm_node` 是官方的有限状态机节点，内部跑 MoveIt Servo，对外只开一个 gRPC 端口。

**灯条是最快的状态读数**，比看日志快：

| 灯条 | 含义 | 怎么办 |
|---|---|---|
| 绿色呼吸 | 驱动已接管，可以遥操 | 继续 |
| 白色呼吸 | 驱动没接管，臂不会动 | `teleop_up.sh restart` |
| 黄色呼吸 | 零点丢失 | 先做零点校准，别遥操 |

## 3.2 头显端：PicoTeleop Unity APK

APK 在 `EE_controlhal/PICO_control/pico_teleop_unity/Builds/PicoTeleopUE.apk`，源码在 `Assets/PicoTeleop/Scripts/`。四个模块：

```text
Input/XRDeviceReader.cs      读 Unity XR InputDevices（HMD + 双手柄）
Protocol/PicoTeleopPacket.cs 组 JSON
Network/UdpTeleopSender.cs   TCP 或 UDP 发出去
Core/PicoTeleopManager.cs    以固定 60Hz 驱动上面三个
```

**这个 app 不认识机器人。** 它只做一件事：把 XR 追踪空间的原始位姿和按键塞进 JSON 发走。所有坐标变换、所有机器人语义都在电脑侧。换机械臂不用重新出 APK，换 VR 设备只需要改电脑侧的一个 `vr_space` 配置。

读设备用的是标准 Unity XR API，不是 PICO 私有接口：

```csharp
device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position);
device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation);
```

每个值都过 `IsFinite` 检查，NaN 就把 `Valid` 置 false 而不是发出去污染下游。

发送频率和渲染帧率解耦（用累加器自己攒时间），所以画面卡了不影响控制频率。

**一个很实用的细节**：`ConfigureEnterpriseKeepAwake()` 调用 PICO 企业版 API 关掉接近传感器、把息屏延时设为 NEVER。这样头显可以**挂在脖子上或架在桌上当追踪基站**，不用戴脸上。屏幕是黑的，追踪照常工作。遥操作时你需要看真实的机械臂，不是看头显里的画面。

用 adb 直接拉起，省得在头显里翻菜单：

```bash
adb shell am start -n com.egoarx.picoteleopue/com.unity3d.player.UnityPlayerActivity
```

## 3.3 协议：pico_teleop_v1

一帧 JSON。[pico_packet.py](airbot+hand-workspace/EE_controlhal/middle_interface/protocol/pico_packet.py) 是权威定义：

```json
{
  "protocol": "pico_teleop", "version": 1,
  "seq": 12345, "device_time_ns": 0, "wall_time_unix_ms": 0,
  "hmd": {"valid": true, "position": [0,0,0], "rotation": [0,0,0,1]},
  "left_controller": {
    "valid": true, "position": [0,0,0], "rotation": [0,0,0,1],
    "linear_velocity": [0,0,0], "angular_velocity": [0,0,0],
    "trigger": 0.0, "grip": 0.0,
    "primary_button": false, "secondary_button": false,
    "joystick": [0,0]
  },
  "right_controller": {}
}
```

三条约定要记住：

- **四元数一律 `[x, y, z, w]`**，全链路统一。ROS 是 xyzw，Unity 的 `Quaternion` 也是 xyzw，所以没有转换点，但你自己加代码时别写成 wxyz。
- **位置单位是米**
- **解析严格**：`from_dict` 对任何字段缺失、长度不对、NaN/Inf 都抛 `ValueError`；`from_json_bytes` 捕获后打 warning 返回 None。坏包被丢掉而不是变成 NaN 传下去。

接收侧还统计 seq 断号：

```text
[PICO] RX: 60.0 Hz seq=12345 loss=0 src=127.0.0.1:xxxxx state=TELEOP_ACTIVE
```

`loss` 持续增长就是链路有问题（USB 线、CPU 抢占），`Hz` 明显低于 60 也一样。

一个容易忽略的点：主循环用的是 `poll_latest()` 而不是 `poll()`。它会把缓冲区里所有积压的包全读掉，**只保留最新的一帧**。遥操作里陈旧的目标位姿毫无价值——宁可丢帧也不要延迟累积。

## 3.4 中间层：五级流水线

这是整个工程算法密度最高的部分。入口 [app.py](airbot+hand-workspace/EE_controlhal/middle_interface/app.py)，核心 [processing/pipeline.py](airbot+hand-workspace/EE_controlhal/middle_interface/processing/pipeline.py)。

```text
原始包
 ①  vr_normalize    设备追踪系 → teleop_standard
 ②  head_base       转成"相对操作者朝向"
 ③  filter          EMA 平滑
 ④  clutch          扳机离合 + 增量合成       ← 核心
 ⑤  limits          速度 / 工作空间
绝对笛卡尔目标 RobotCommand
```

### ① 坐标归一化

先定义一个统一的中间坐标系 `teleop_standard`：**+X 前、+Y 左、+Z 上**（右手系，和 ROS 习惯一致）。

Unity XR 追踪空间是 **+X 右、+Y 上、+Z 前**。转换就是 [vr_normalize.py:52-54](airbot+hand-workspace/EE_controlhal/middle_interface/processing/vr_normalize.py#L52-L54)：

```python
def unity_to_teleop_position(position):
    x, y, z = as_vec3(position)
    return np.array([z, -x, y])      # 前=Unity的Z，左=Unity的-X，上=Unity的Y
```

旋转是相似变换 `R · M · Rᵀ`（不是简单换轴，四元数换轴必须走这个形式，否则手性和旋转顺序会错）。

代码里还留了 OpenXR Stage 的版本（`+X 右, +Y 上, -Z 前`，映射成 `[-z, -x, y]`），靠 `transform.vr_space` 配置选。PICO 这个 app 用 `unity`。**换 VR 设备只需要在这里加一个矩阵**，下游完全不动。

### ② head-base：为什么必须做

这是遥操作体验的关键一步，也是最容易被跳过的。

问题：VR 位姿是在房间固定坐标系里的。如果你握着手柄站着不动然后**原地转身 90 度**，手柄的世界坐标位置变了——机械臂就会跟着动。这显然不对：你的手相对于你自己没动。

解法（[head_base.py](airbot+hand-workspace/EE_controlhal/middle_interface/processing/head_base.py)）：把手柄位姿表达在**头显的偏航坐标系**里。

```python
p_rel = R_yawᵀ · (p_hand - p_hmd)
R_rel = R_yawᵀ · R_hand
```

关键是 `R_yaw` **只取偏航，丢掉俯仰和滚转**：

```python
forward = matrix_from_quat(hmd.rotation) @ [1, 0, 0]
forward[2] = 0.0          # 把 Z 分量清零 → 投影到水平面
forward /= norm(forward)
left = cross([0,0,1], forward)
return column_stack((forward, left, up))
```

为什么只要偏航？因为头显是**挂在脖子上**的（见 3.2），它的俯仰滚转是随机噪声，不代表操作者意图。只保留"我面朝哪"这一个自由度，语义就变成了：**朝你面对的方向推手柄 → 机器人 +X 前进**。你转身，映射跟着转，手不动机器人就不动。

HMD 追踪短暂丢失时会沿用上一帧的参考系（`_has_ref`），而不是突然跳回单位矩阵——那会让机器人猛地一动。

### ③ EMA 滤波

[filter.py](airbot+hand-workspace/EE_controlhal/middle_interface/processing/filter.py) 是一阶指数平滑，`alpha=0.25`，作用在 **head-base 之后**的 `hand_rel` 上。

顺序很重要。[default.yaml](airbot+hand-workspace/EE_controlhal/middle_interface/config/default.yaml) 的注释说明了原因：「world-frame filtering smears turns」——在世界系滤波的话，你转身时整个位姿都在变，滤波会把这个变化拖成一个缓慢的漂移，手感会"糊"。在 head-base 之后滤，转身本身已经被抵消掉了，滤的就只是真实的手抖。

### ④ 离合（clutch）：核心机制

[clutch.py](airbot+hand-workspace/EE_controlhal/middle_interface/processing/clutch.py) 实现的是**增量式相对遥操作**，不是绝对映射。这是理解整个臂链路的关键。

绝对映射（手柄位置直接当机器人位置）不可用，原因有三：人手工作空间和机械臂工作空间不重合、启动瞬间机器人会猛地跳到手柄位置、手臂累了没法"松手休息"。

增量式的做法，握住右手食指扳机（deadman）时：

```text
按下瞬间：
    robot_ref  = 当前机器人 TCP 位姿    （立刻锁存）
    vr_ref     = 手柄位姿              （延后一帧锁存）

握住期间每一帧：
    Δp = scale · (p_now − p_vr_ref)              → 截断到 max_delta_position
    Δq = slerp(I, q_now · conj(q_vr_ref), rot_scale) → 截断到 max_delta_angle
    cmd_pos = robot_ref.pos + R·Δp
    cmd_rot = (R·Δq·R⁻¹) · robot_ref.rot

松开：清空所有 ref，机器人停在最后一个目标
```

几个设计点：

**为什么 VR 基线要延后一帧？** `delay_vr_baseline: true`。按下扳机那一帧，手指的动作会带着手柄轻微抖动。如果这一帧就锁存 VR 基线，第一个增量里就带着这个抖动，机器人会"抽"一下。延后一帧（`_pending_vr` 逻辑），第一帧直接返回 `robot_ref` 本身（Δ=0），下一帧才建立基线。

**Δ 截断在做什么？** 单次握持内的位移上限 0.35 m、角度上限 1.50 rad（约 86°）。这不是速度限制，是**行程限制**——防止追踪跳变（遮挡、反光）造成一个巨大的 Δ 把机器人甩出去。想走更远就松开扳机再握一次（"重新抓握"），和用鼠标时提起来重新放下一样。

角度上限 86° 的选择理由写在 [airbot.yaml](airbot+hand-workspace/EE_controlhal/middle_interface/config/airbot.yaml)：「Angle cap ~86 deg so the wrist is visible」——超过这个角度腕部会转到看不见的姿态。

**旋转合成为什么写成 `R·Δq·R⁻¹`？** 因为 Δq 是在 teleop_standard 系里算出来的，要作用到机器人基座系的姿态上，必须做基变换。直接 `Δq · robot_rot` 会得到错误的旋转轴。

**`_reset_motion()` 与"重新武装"。** `require_deadman_release_on_start: true` 配合 `_deadman_armed` 标志：程序启动、暂停、超时、回零之后，**必须先松一次扳机**才能重新开始跟随。如果不这样，你启动程序时正好扳机是按着的，机器人立刻开始跟随——而你还没准备好。日志里那句 `[TELEOP] Deadman armed after release` 就是武装完成。

### ⑤ 限位

两层：

- **速度限制** `VelocityLimit`：真机配置里 `enabled: false`，因为下游命令层已经有 slew 限速了，两层限速会互相拖后腿。
- **工作空间裁剪** `WorkspaceLimit`：轴对齐的盒子，`stop_on_exceed: false` 意味着超界只**裁剪**不停止——遥操作继续有效，你只是推不出盒子。这比"超界就急停"友好得多。

盒子的值 `x: 0.06~0.50, y: ±0.22, z: 0.10~0.50` 是围绕标定过的 home TCP（约 0.13, 0, 0.21）的可达范围。**换臂或换安装位置必须重新量这个盒子。**

### 安全状态机

[safety.py](airbot+hand-workspace/EE_controlhal/middle_interface/processing/safety.py) 五个状态：

```text
DISCONNECTED ──收到包──► READY ──按扳机──► TELEOP_ACTIVE
                          ▲                    │
                          │                 松扳机
                          │                    ▼
                       (重新武装) ◄────────  PAUSED

超时/工作空间trip ──► ERROR ──收到包──► READY
```

类注释一句话说清了原则：**"Never auto-starts motion. Clutch is the only enable path from READY/PAUSED."** 任何路径都不会自动开始运动，唯一的使能入口是扳机。

### 按键映射

| 输入 | 作用 |
|---|---|
| 右食指扳机 | deadman，按住跟随、松开停住（阈值 press 0.5 / release 0.3，带回差防抖） |
| 右 A（`right_primary`） | 回零位，**会真实运动** |
| 左 X（`left_primary`） | 暂停 |
| 右侧握把（`grip`） | 原本是夹爪，**当前被丢弃**（`enable_eef: false`） |

握把被丢弃是因为这台臂的末端装的是 Revo2 灵巧手，不是夹爪。手由链 B 独立控制。命令层收到非零 gripper 会打一条 info 日志然后忽略。

回零用的是"边沿触发"（`_home_edge`），按住不会连续触发。而且回零前会 `_reset_motion()` 并进入 PAUSED，回零完成后要重新武装扳机——避免回零途中你还握着扳机导致刚回到位就被拽走。

## 3.5 命令层：Robot_control_interface

middle_interface 的输出是抽象的 `RobotCommand`（绝对笛卡尔目标）。往下是两级适配：

```text
RobotCommand              ← 遥操作语义（middle_interface）
  │ robot/adapters/command_layer.py
  ▼
ArmCommand + MotionMode   ← 机器人抽象（interface.py / arm_types.py）
  │ backends/airbot_sdk.py
  ▼
airbot_py servo_cart_pose ← 具体 SDK
```

分两层是为了换机器人只改一个 backend。`factory.py` 支持 `dummy` / `airbot_sdk` / `arx_ros2`（ARX X5 走 ROS topic，是这套代码的前一代目标机器人）。

### 连接序列

[airbot_sdk.py:73-113](airbot+hand-workspace/EE_controlhal/Robot_control_interface/backends/airbot_sdk.py#L73-L113) 顺序是有讲究的：

```python
robot.connect()                                   # 1. gRPC 连上
self._refresh_state()                             # 2. 读一次状态，验证关节数
robot.set_speed_profile(SpeedProfile.DEFAULT)     # 3. 速度档
robot.set_params({                                # 4. 覆盖 MoveIt Servo 缩放
    "servo_node.moveit_servo.scale.linear": 1.5,
    "servo_node.moveit_servo.scale.rotational": 3.0,
    "servo_node.moveit_servo.scale.joint": 0.6,
})
robot.switch_mode(RobotMode.SERVO_CART_POSE)      # 5. 进伺服模式
threading.Thread(target=self._command_loop)       # 6. 起 100Hz 发布线程
```

第 4 步的三个数字是调出来的，注释解释了为什么（[airbot_sdk.yaml](airbot+hand-workspace/EE_controlhal/Robot_control_interface/config/airbot_sdk.yaml)）：

> DEFAULT: linear/rotational 0.2, joint 0.1. FAST is 10/10/1 (too aggressive).
> Position tracking is acceptable at 1.5. Wrist tracking needs a higher rotational scale and a joint scale above 0.1 or J4/J6 cannot keep up.

官方 DEFAULT 档太慢（0.2 大约 4~5 cm/s），FAST 档（10/10/1）太猛。手调到 1.5/3.0/0.6：位置跟踪 1.5 够了，但**腕部旋转需要更高的 rotational，而且 joint scale 必须超过 0.1，否则 J4/J6 跟不上手腕的转动**。这是典型的"官方给的两档都不合适，需要自己在中间找"。

### 命令循环：latest-wins + slew

```python
while self._running:
    latest = self._latest          # 最新目标（上层随时覆盖）
    current = self._stream_pose    # 上一次实际发出去的目标
    position, rotation = slew_pose(current, latest, dt=0.01,
                                   max_linear=1.0, max_angular=4.0)
    self._stream_pose = (position, rotation)
    self._robot.servo_cart_pose([position, rotation])
    self._refresh_state()
    time.sleep(0.01)
```

两个机制：

**latest-wins**：上层写 `self._latest`，循环只读最新值。上层 60Hz 写、循环 100Hz 读，不排队不积压。老目标直接被覆盖丢弃——遥操作里这是对的。

**slew 限速**（[convert.py:83-128](airbot+hand-workspace/EE_controlhal/Robot_control_interface/convert.py#L83-L128)）：不直接发目标，而是从当前流式位姿朝目标匀速逼近，每周期最多走 `1.0 m/s × 0.01 s = 1 cm`，旋转最多 `4.0 rad/s × 0.01 = 0.04 rad`。位置用线性插值，旋转用 slerp。这样即使上层给出一个跳变目标，机械臂也是平滑过去的。

注意这个循环**即使没有新目标也在跑**（`_refresh_state`），保证状态是新鲜的——上层 `get_ee_pose()` 拿到的不是几百毫秒前的旧值。

### stop() 为什么不能进 SOFT

抽象方法注释写得很直接：

> Hold the current TCP. **Must not go SOFT (arm would drop).**

`stop()` 的实现是：清空 `_latest`、把 `_stream_pose` 设成当前实际位姿、**重新进入 SERVO_CART_POSE**。也就是"停在这里保持"，而不是"卸力"。

ARX 那一代的 README 里记录了同类教训：「松开扳机后不要切 PROTECT，本机走 `/arm_cmd` 的 mode=2 仍会掉。和官方键盘一样，停在 mode=4，100 Hz 重复最后一个 TCP。不要发 mode=0 SOFT。」

松开扳机 ≠ 关掉电机。松开扳机是"我暂时不动它了"，机器人必须**保持姿态**，尤其是它可能正举着东西。

### go_home 的完整序列

回零不是发一个位姿就行（[airbot_sdk.py:170-204](airbot+hand-workspace/EE_controlhal/Robot_control_interface/backends/airbot_sdk.py#L170-L204)）：

```text
1. 清空 _latest（停止伺服流）
2. 校验当前关节在官方规划限位内   ← 不在的话规划器会直接拒绝
3. 校验目标 home_joints 也在限位内
4. switch_mode(PLANNING_POS)      ← 从伺服模式切到规划模式
5. move_to_joint_pos(home_joints) ← 阻塞式规划运动
6. 读回关节，验证误差 < 0.03 rad  ← 不验证的话可能"报告完成但没到位"
7. 用到位后的实际位姿重置 _stream_pose  ← 否则伺服会从旧位姿跳
8. switch_mode(SERVO_CART_POSE)
9. sleep(0.3)                     ← 关键
10. 恢复 END_CONTROL
```

第 9 步那句注释：「Servo interfaces need a beat after planning before cart commands stick.」——从规划模式切回伺服模式后，controller_manager 的接口需要一点时间稳定，立刻发笛卡尔命令会被吞掉。这是实测出来的。

第 2 步的限位校验也是踩过坑的：如果机械臂当前姿态已经超出官方规划限位（可能是手动推的），规划器会拒绝，而错误信息很不明确。这里提前检查并抛出人能读懂的消息：

```text
AIRBOT current state is outside official planning limits: joint3=3.200 not in [-0.094, 3.161]
```

`app.py` 对 go_home 失败的处理是"记日志但留在遥操作里"，不是崩掉——你按了 A 但臂在限位外，那就提示你先把臂推回来，遥操作继续可用。

## 3.6 臂侧完整启动

```bash
# 前提：can0 UP，合容器已起
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh
```

[teleop_up.sh](airbot+hand-workspace/teleop_up.sh) 五步：

```text
1/5 CAN 总线      ip -br link show can0 | grep UP
2/5 工作容器      running？没有就 docker start，不存在就提示先跑 bringup
3/5 AIRBOT 驱动   先 kill_app（重要，见下）→ 起 sidecar → 等端口 → 再等 15 秒初始化
4/5 PICO 头显     adb devices → 装 APK（如需）→ 重建 adb reverse
5/5 机械臂自检    连一次臂，读关节角和末端位姿，切一次 SERVO_CART_POSE
```

第 3 步为什么要先杀 `app.py`？脚本注释：

> app.py 必须先停：SDK 客户端连着时重启驱动会留下悬挂的 gRPC 会话，新驱动起来后 controller_manager 的接口状态就是脏的，切模式必然失败。

这是那种"不知道就要 debug 两小时"的知识。

第 3 步里有两个等待，也值得注意：gRPC 端口出现 ≠ 驱动可用。端口 listen 之后，驱动还要初始化控制器、和 CAN 握手，这段时间连接会超时。所以脚本先等端口（最多 30 秒），再无条件等 15 秒。

第 5 步自检不是一次性判死，而是重试 6 次 × 5 秒，因为刚启动时握手可能要几十秒，SDK 会内部超时并报 `Failed to connect`。

自检通过后，**手动**启动遥操作进程：

```bash
docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
  airbot-revo-humble python3 app.py --config config/airbot.yaml
```

为什么不放进脚本？因为它要占着终端看日志。日志是这条链唯一的实时可观测窗口。

**日志三行对照表**（看到这三行才算真的通了）：

```text
ROBOT ADAPTER: REAL (COMMAND_LAYER)      ← 真机（DUMMY 是空跑）
[PICO] RX: 60.0 Hz  loss=0  state=READY  ← 包在进
[TELEOP] Deadman armed after release     ← 松一次扳机才武装
```

空跑验证把配置换成 `default.yaml`（dummy 后端，臂不动，全流水线照跑）。**第一次搭建强烈建议先空跑**，确认包进得来、状态机转得对，再上真机。

退出**务必 Ctrl-C**。`app.py` 的 `finally` 会走 `adapter.stop()` → `disconnect()` → 关 socket → 关记录文件。直接 kill 会留下脏的 gRPC 会话，下次启动就要 restart 驱动。

## 3.7 数据记录

`airbot.yaml` 开了 `recording.enabled: true`。每收到一个包写一行 JSON 到 `logs/vr_arm_*.jsonl`，内容包括：手柄/HMD 位姿、**命令下发前的机器人实际 TCP**、命令目标 TCP、遥操作状态、动作类型。

注意 `app.py` 是在 `pipeline.process()` **之前**读实际位姿的——这样记录下来的是"这个命令是基于哪个实际状态算出来的"，做离线分析（比如算跟踪误差、训模仿学习）时因果关系是对的。

---

# 第 4 章 手链路：MANUS → Revo2

现在换一条完全不同风格的链——这条是标准 ROS 2 + ros2_control。

## 4.1 全链路

```text
MANUS 手套 + 接收器 (USB 3325)
  │ manus_ros2 / manus_data_publisher      C++，链 MANUS Integrated SDK，~60Hz
  ▼
/manus_glove_0  /manus_glove_1             manus_ros2_msgs/ManusGlove
  │ manus_revo2_retarget_node              Python，algorithm: joint_thumb，无 IK
  ▼
/revo2_<side>/revo2_pid_controller/target_joint_states    sensor_msgs/JointState, rad, ~100Hz
  │ revo2_pid_controller                   C++ ros2_control controller，PD 速度闭环
  ▼
<joint>/velocity command interface         rad/s
  │ brainco_hand_hardware                  SystemInterface，Modbus RTU
  ▼
/dev/revo2_hand_right slave 127 @460800    Revo2 灵巧手
```

## 4.2 manus_ros2：SDK 桥

[src/ManusDataPublisher.cpp](manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_capabilities/manus_ros2/src/ManusDataPublisher.cpp)（986 行 C++）。三个关键点：

**（1）用 Integrated SDK，不是 Core SDK。** [CMakeLists.txt:16-23](manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_capabilities/manus_ros2/CMakeLists.txt#L16-L23)：

```cmake
option(MANUS_USE_CORE_SDK "Link against libManusSDK.so for MANUS Core connection mode" OFF)
# 默认 OFF → 链 libManusSDK_Integrated.so
```

Integrated 版本把 MANUS Core 编译进了进程，不需要单独跑 MANUS Core 服务，也不需要装 gRPC/Protobuf 依赖。少一个进程少一层故障点。

`.so` 必须放在 `ManusSDK/lib/` 下（源码不含，从官方 SDK 拷）：

```text
src/brainco_capabilities/manus_ros2/ManusSDK/lib/libManusSDK.so
src/brainco_capabilities/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
```

本工程已经软链好了（源来自 `ManusSDK_v3.1.1/`）。

**（2）Topic 是惰性创建的。** 检测到第几只手套就建 `manus_glove_0`、`manus_glove_1`。所以 **topic 编号 ≠ 左右手**！编号只是发现顺序。左右手靠消息里的 `side` 字段区分，retarget 节点按 `side` 分流。

这点在实测记录里对上了：右手在 `/manus_glove_0`，左手在 `/manus_glove_1`——但这是这次的发现顺序，换个上电顺序可能就反了。**不要在代码里假设编号对应哪只手。**

**（3）消息内容。** [ManusGlove.msg](manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_capabilities/manus_ros2_msgs/msg/ManusGlove.msg)：

```text
int32 glove_id
string side                          # "Left" / "Right"
ManusRawNode[] raw_nodes             # 骨架节点，带 Pose（做 IK/可视化用）
ManusErgonomics[] ergonomics         # 关节角，name+value（度）← retarget 主要用这个
geometry_msgs/Quaternion raw_sensor_orientation
geometry_msgs/Pose[] raw_sensor
```

`ergonomics` 是 MANUS SDK 算好的人手关节角，名字形如 `IndexMCPStretch`、`ThumbMCPSpread`，单位是**度**。这是 retarget 的主要输入——直接用现成的关节角，比自己从骨架点做 IK 简单得多也稳得多。

## 4.3 手套标定（.mcal）

MANUS 官方标定校正**手套本身**（不同人手型不同），产出 `.mcal` 文件。

```bash
# 标定前先停掉 retarget launch 和 manus_data_publisher（多个 SDK client 会抢手套）
ros2 run manus_ros2 manus_calibration_tool --list                    # 看在线手套
ros2 run manus_ros2 manus_calibration_tool --side right --overwrite  # 右手
ros2 run manus_ros2 manus_calibration_tool --side left  --overwrite  # 左手
```

工具会打印每一步动作提示（大致是：手放平手指并拢拇指外展 → 抬手握拳 → 指尖反复触碰掌根），准备好按回车。具体以运行时 SDK 打印为准。

产出路径：

```text
src/brainco_capabilities/manus_ros2/calibrations/<glove_family>/Calibration_<side>.mcal
```

什么时候要重做：换使用者、重新佩戴后指尖明显漂移、拇指/无名指对指关系整体偏掉（不是单个关节小 offset 能救的）、`.mcal` 丢了。

验证加载成功——启动 publisher 看日志：

```text
Successfully loaded calibration for metaglovepro right glove ... from .../Calibration_right.mcal
```

**注意**：本工程当前的 `calibrations/metagloveprohaptics/Calibration_right.mcal.disabled` 是被**禁用**的（改了后缀）。[控制链路.md](manus-Revo2-Revo3/控制链路.md) 说明：「日志中的 `Calibration_left/right.mcal` 缺失是校准文件警告，目前不阻断手套消息和目标话题输出」。也就是说没标定也能跑，只是映射精度差些。要精细操作就该做标定。

## 4.4 retarget：joint_thumb 算法

[retargeters/joint_thumb_retargeter.py](manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_capabilities/manus_revo2_retarget/manus_revo2_retarget/retargeters/joint_thumb_retargeter.py)。这是当前默认算法，核心思想：**不做 IK，直接按关节语义映射**。

Revo2 有 6 个主动关节（[revo2_joints.py](manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_capabilities/manus_revo2_retarget/manus_revo2_retarget/revo2_joints.py)），顺序固定，全链路都按这个顺序：

| # | 关节 | 限位 (rad) |
|---|---|---|
| 0 | `thumb_proximal_joint` | 0 ~ 1.0472 (60°) |
| 1 | `thumb_metacarpal_joint` | 0 ~ 1.5184 (87°) |
| 2 | `index_proximal_joint` | 0 ~ 1.4661 (84°) |
| 3 | `middle_proximal_joint` | 0 ~ 1.4661 |
| 4 | `ring_proximal_joint` | 0 ~ 1.4661 |
| 5 | `pinky_proximal_joint` | 0 ~ 1.4661 |

**四指映射。** 人手每指有 MCP/PIP/DIP 三个弯曲自由度，Revo2 每指只有一个。所以做加权求和：

```python
FOUR_FINGER_STRETCH_WEIGHTS = {"MCP": 0.50, "PIP": 0.35, "DIP": 0.15}

weighted = 0.50*IndexMCPStretch + 0.35*IndexPIPStretch + 0.15*IndexDIPStretch
normalized = clip(weighted / 84.0, 0, 1)      # 84 度 = Revo2 上限
target = normalized * 1.4661                   # rad
```

MCP 权重最大，因为它主导整体弯曲的视觉/功能效果。负值（某些配置下的过伸）用 `max(0.0, ...)` 夹到张开端。

**拇指映射。** 这里是这个算法名字的来源：

```text
侧摆：一个 MANUS 量 → 一个 Revo2 关节
  ThumbMCPSpread → thumb_metacarpal_joint

弯曲：三个 MANUS 量加权 → 一个 Revo2 关节
  0.45*ThumbMCPStretch + 0.35*ThumbPIPStretch + 0.20*ThumbDIPStretch → thumb_proximal_joint
```

**为什么放弃拇指 IK？** 类文档字符串说得很明确：

> It intentionally avoids free thumb IK so thumb_meta expresses side swing and thumb_proximal expresses thumb flexion.

拇指只有 2 个主动自由度，但目标是 3D 指尖位置——**IK 是欠定的，多解**。实际后果是：你弯四指的时候，四指参考中心变了，IK 会把 `thumb_meta` 也拉着乱跑。README 里的原话：「避免四指弯曲或四指参考中心变化时牵着 `thumb_meta` 乱跑」。

关节语义映射牺牲了指尖位置精度，换来了**可预测性**——你侧摆拇指，就只有侧摆关节动。对遥操作来说这个交换非常值得：操作者能建立稳定的肌肉记忆。

原来的 `revo3_thumb`（IK 版，941 行）保留着，但只用于诊断和 MuJoCo 可视化，不是控制路径。

**调参入口**在 [retarget.yaml](manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_capabilities/manus_revo2_retarget/config/retarget.yaml) 的 `revo3_thumb.right` 下：

```yaml
thumb_meta_range_deg: 30.0      # 越小，MANUS 小幅侧摆就越容易打满 Revo2 侧摆
thumb_meta_zero_deg: 0.0        # 中立位偏内摆时，小幅增加这个
thumb_prox_zero_deg: 10.0       # 张开时的死区，让被动 DIP 看起来更直
thumb_prox_range_deg: 90.0
thumb_prox_mcp_weight: 0.45     # 三个权重会自动归一化
thumb_prox_pip_weight: 0.35
thumb_prox_dip_weight: 0.20
```

## 4.5 revo2_pid_controller：闭环在哪

这是当前推荐的控制后端，也是理解手链路的关键。

retarget 只输出**目标位置**（JointState, rad），不做闭环。闭环在 [revo2_pid_controller.cpp](manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_drivers/revo2_driver/src/revo2_pid_controller.cpp) 里——它是一个 **ros2_control controller**，不是普通 ROS 节点。

这个区别很重要：作为 controller，它跑在 controller_manager 的实时循环里，**直接读硬件 state interface、直接写 velocity command interface**，不经过 ROS topic。中间少了两次序列化和一次调度延迟。

`update()` 的完整逻辑：

```text
1. 没收到 target？          → 速度全零 + 限流警告
2. target 超时 (>0.3s)？    → 速度全零   ← 手套断了手不会失控
3. 读实际位置                actual = state_interface * scale + offset
4. 目标 EMA 滤波            大变化用 fast_alpha=1.0，小变化用 alpha=0.75
5. error = filtered_target - actual
6. PD                       v = kp*error + kd*d(error)/dt
7. 分关节 kp 缩放           拇指两关节 x2.4，无名指 x1.5
8. 死区                     |error| <= deadband → v = 0
9. 拇指刹车区               接近目标时按 error/brake_zone 线性压 v 上限
10. 全局限幅 + 变化率限制    velocity_max = 2.25 rad/s
11. 写 command interface     command_interfaces_[i].set_value(v)
```

几个设计值得学：

**第 2 步的超时保护**是安全底线。手套掉线、retarget 崩了、USB 拔了——任何原因导致目标停止更新，0.3 秒后速度归零，手停住。不做这个的话，最后一个速度命令会一直执行下去。

**第 4 步的双 alpha 滤波**（`alpha: 0.75, fast_alpha: 1.0, fast_threshold: 0.04`）：小变化（<0.04 rad）用 0.75 平滑掉手套噪声，大变化直接用 1.0 完全跟随不引入延迟。这样既滤抖又不拖慢快速动作。

**第 7、9 步的拇指特殊处理**：拇指 kp 放大 2.4 倍（因为拇指负载大、行程要求高），但同时加"刹车区"——距目标小于 `brake_zone=0.209 rad` 时按比例压低速度上限。README 描述了不加刹车的症状：拇指会像「刹车点头」一样来回晃。这是大 kp 必然带来的过冲，用接近目标时的速度衰减来治，比降 kp 更好（保留了远距离的快速性）。

参数在 [revo2_controllers.yaml](manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_drivers/revo2_driver/config/revo2_controllers.yaml) 的 `pd_velocity` 下。

## 4.6 硬件接口：单位换算链

`brainco_hand_hardware.cpp` 是 ros2_control 的 `SystemInterface` 实现，桥接 ROS 单位（rad, rad/s）和 BrainCo Stark SDK 的设备单位。

右手当前配置（[protocol_modbus_right.yaml](manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_drivers/revo2_driver/config/protocol_modbus_right.yaml)）：

```yaml
finger_unit_mode: physical                    # 物理单位模式（另一个是 normalized）
physical_velocity_command_scale: 57.29577951  # rad/s → deg/s
physical_velocity_state_scale: 0.01745329     # deg/s → rad/s
physical_velocity_device_min: -130.0          # 手的实际角速度上限 deg/s
physical_velocity_device_max: 130.0
velocity_percentage: 100.0
```

所以一条速度命令的完整旅程：

```text
1.0 rad/s (controller 输出)
  x 57.2958                 → 57.3 deg/s
  clamp 到 [-130, 130]      → 57.3 deg/s
  x velocity_percentage/100 → 57.3 deg/s
  → SDK speed API → Modbus RTU → 手
```

两种模式的区别：

- `normalized`：设备单位是 0~1000 的无量纲值，scale 是 `572.9577951308232`，clamp 到 `[-1000, 1000]`
- `physical`：设备单位是真实的度/度每秒，更直观，但需要按真机行程标定

配置里位置的 physical 覆盖参数是**注释掉的**，注释说明原因：「Position overrides stay commented until the physical angle range is calibrated on the real hand」——位置还没在真机上标定物理行程，所以先用基础 scale。速度已经标了（±130 deg/s）。这是个诚实的中间状态，值得学习这种"标了什么写什么"的态度。

启动时还有重试机制：

```yaml
connection_retry_attempts: 20
connection_retry_interval_ms: 500
```

因为「启动时串口/udev 可能比 ros2_control 稍晚就绪」——避免因为几百毫秒的时序差就要手动重启驱动。

## 4.7 Controller 切换：必须理解的一步

驱动**默认激活的不是 PID controller**：

```yaml
load_on_configure:                    # 加载四个
  - revo2_joint_state
  - joint_forward_pos_controller
  - joint_forward_vel_controller
  - revo2_pid_controller

activate_on_configure:                # 只激活两个
  - revo2_joint_state
  - joint_forward_pos_controller       # ← 默认是位置前向控制器
```

这是**有意的安全默认值**。硬件 bring-up 阶段你想直接发位置命令测手，不依赖手套和 retarget：

```bash
ros2 topic pub --once /revo2_right/joint_forward_pos_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.30, 0.10, 0.60, 0.60, 0.60, 0.60]}"
```

但 MANUS 遥操作要用 `revo2_pid_controller`，所以必须切。ros2_control 里同一组 command interface 只能被一个 controller 独占，所以要**先 deactivate 位置控制器，再 activate PID 控制器**：

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control switch_controllers \
  -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --activate revo2_pid_controller \
  --activate-asap --strict
```

**遥操作时的理想状态**：

```text
revo2_joint_state            active
joint_forward_pos_controller inactive
revo2_pid_controller         active
joint_forward_vel_controller inactive
```

**两个坑**：

1. **不要在 `--strict` 下 deactivate 已经 inactive 的 controller**。比如 `joint_forward_vel_controller` 本来就是 inactive，写进 `--deactivate` 会让整个切换失败。`real_hand_pipeline_launch.py` 的自动切换脚本就是先用 `controller_active` 逐个检查、只把真正 active 的加进 `stop_args`。

2. **`ROS2CLI_DISABLE_DAEMON=1`**。`ros2 control` 有时会报 `!rclpy.ok()` 或 `xmlrpc.client.Fault`，那是 ROS 2 CLI daemon 状态异常。禁掉 daemon 直连即可（或者 `ros2 daemon stop && ros2 daemon start`）。

## 4.8 手侧启动：三种模式

### 模式 A：只跑前半段（无灵巧手，已验证）

这是最安全的起点——手套数据进 ROS，retarget 算目标，但**不向真机发命令**。

```bash
docker exec -it -u 0 airbot-revo-humble bash
cd /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch manus_revo2_retarget pipeline_launch.py \
  hand_mode:=right \
  launch_manus_publisher:=true \
  controller_backend:=ros2_control
```

另开终端验证：

```bash
ros2 topic hz /manus_glove_0                                                   # 应约 60 Hz
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once    # 应约 100 Hz
```

**目标话题 `Subscription count: 0` 在这个模式下是正常的**——没人订阅，因为驱动没起。第一次看到这个别以为是坏了。

已验证的数据（[控制链路.md](manus-Revo2-Revo3/控制链路.md)）：两只 MetagloveProHaptics 手套，`/manus_glove_0` 和 `/manus_glove_1` 约 60Hz，左右手目标话题各约 100Hz。

完全没有手套时用 `sim_manus_glove_publisher`（包里自带），**不要**开 publisher 去连不存在的接收器。

### 模式 B：真机一键（会运动）

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  launch_plot:=false
```

这个 launch 按时序做四件事：

```text
t=0     启动 revo2_driver（update_rate 20Hz）
t=16s   尝试自动切到 revo2_pid_controller（重试 30 次）
t=18s   启动 MANUS publisher + target-only retarget
t=19s   启动 plot 监视器（launch_plot:=false 可关）
```

延时不是随便定的：`switch_delay:=16.0` 是给慢速 Modbus 启动留的余量。如果你的端口固定、启动快，可以调小。

**`update_rate` 这里默认 20Hz，而 `revo2_driver` 单独启动默认 500Hz。** README 解释了：20Hz「比 500Hz 更适合当前 Modbus 右手链路」。Modbus RTU 在 460800 波特率下，一次读写往返有实实在在的时间开销，500Hz 会撑不住导致丢帧或超时。验证串口足够稳之后再往上调。

这也是理解手链路性能上限的关键：**瓶颈在 Modbus 串口，不在算法**。手套 60Hz、retarget 100Hz、controller 20Hz——最慢的那一环决定实际响应。

### 模式 C：手动拆开逐环排查

每一环单独起，用来定位问题：

```bash
# 1. 只起 MANUS publisher
ros2 run manus_ros2 manus_data_publisher
# 验证：ros2 topic echo /manus_glove_0 --once  → 看 side 字段

# 2. 只起驱动
ros2 launch revo2_driver revo2_system.launch.py hand_side:=right
# 手动切 controller（见 4.7）
# 验证：ros2 topic echo /revo2_right/revo2_joint_state/joint_states --once

# 3. 只起 target-only retarget
ros2 launch manus_revo2_retarget pipeline_launch.py \
  hand_mode:=right launch_manus_publisher:=false controller_backend:=ros2_control
```

`hand_mode:=both` 会起**两个独立的 retarget 进程**（`manus_revo2_retarget_left` / `_right`），各自只算一只手。不是一个进程算两只——这样一只手的算法出问题不会拖另一只。

## 4.9 可视化诊断工具

两个只订阅不控制的工具，用来判断问题出在哪一环：

```bash
# 实时曲线：target vs actual vs error vs velocity
python3 src/brainco_capabilities/manus_revo2_retarget/tools/revo2_retarget_plot.py \
  --side right --joints thumb_prox,thumb_meta,index --window 8

# MuJoCo 三合一：Revo2 模型 + MANUS 原始关键点 + retarget 内部拇指目标点
ros2 run manus_revo2_retarget mujoco_manus_overlay_viewer --ros-args -p hand_mode:=right
```

第一个回答"跟踪好不好"（看 error 曲线），第二个回答"问题在 MANUS 数据、目标生成，还是关节跟踪"。调参时基本靠这两个。

## 4.10 Revo3 是另一套，不要混

`Revo-Retargeting-revo3_retargeting/` 和 `brainco_revo3_ros2-*/` 是 Revo3 的东西。**Revo3 走 MIT 命令，不是这条 `joint_thumb` 链路。**

关键约束：不要把 Revo2 和 Revo3 的 workspace 叠在同一个 overlay 里 source。它们有同名包，叠加后 `ros2 run` 会拿到不确定的那一个，症状极其诡异。

---

# 第 5 章 合起来：完整开机到运动

## 5.1 顺序（必须按这个顺序）

```bash
# ── 0. 公共：容器 + 臂 sidecar ──────────────────────────
/home/ubuntu/airbot-revo-workspace/bringup.sh --no-build
/home/ubuntu/airbot-revo-workspace/bringup.sh status


# ── 1. 臂（终端 A）──────────────────────────────────────
/home/ubuntu/airbot-revo-workspace/airbot+hand-workspace/teleop_up.sh
#   ↑ 五步自检全绿后：
#   ① 看灯条 —— 必须绿色呼吸
#   ② 头显里开 PicoTeleop，面板显示 127.0.0.1:15000
#      adb shell am start -n com.egoarx.picoteleopue/com.unity3d.player.UnityPlayerActivity

docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/middle_interface \
  airbot-revo-humble python3 app.py --config config/airbot.yaml
#   ↑ 这个终端要一直占着看日志


# ── 2. 手（终端 B，必须 -u 0）───────────────────────────
docker exec -it -u 0 airbot-revo-humble bash
source /opt/ros/humble/setup.bash
source /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/install/setup.bash

# 首次或插拔过串口：
cd .../revo2_driver/setup && bash bootstrap_revo2.sh && bash check_revo2_setup.sh

# 真机：
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right controller_backend:=ros2_control launch_plot:=false
```

## 5.2 停止（也有顺序）

```bash
# 臂遥操：app.py 终端 Ctrl-C          ← 不要 kill！会污染驱动接口状态
# 手 launch：该终端 Ctrl-C，或
pkill -f 'pipeline_launch.py|manus_data_publisher|manus_revo2_retarget_node'

# 停臂驱动
.../airbot+hand-workspace/teleop_up.sh stop      # 会先停 app.py 再停 sidecar

# 停合容器（sidecar 保留）
/home/ubuntu/airbot-revo-workspace/bringup.sh stop
```

## 5.3 第一次搭建的推荐路径

不要一上来就全真机。按这个梯度走，每一步失败都能定位到一环：

```text
1. 容器起来           bringup.sh status 全绿
2. 臂 dummy 空跑       app.py --config config/default.yaml
                       → 确认 [PICO] RX 60Hz、状态机会转
3. 臂真机              app.py --config config/airbot.yaml
                       → 先只轻轻动，确认方向对
4. 手前半段            pipeline_launch.py（模式 A）
                       → 确认 manus_glove 和 target 话题都有数
5. 手真机              real_hand_pipeline_launch.py（模式 B）
                       → 确认 controller 状态正确
6. 两条一起            两个终端各跑各的
```

第 3 步「确认方向对」的具体做法：握扳机，**向你面对的方向轻推手柄**，机器人应该 +X 前进。反了就改 `middle_interface` 的 `transform.axis_map`，**不要改命令层**（`Robot_control_interface/README.md` 明确说了这条）。

---

# 第 6 章 排障对照表

## 臂侧

| 症状 | 原因 | 处理 |
|---|---|---|
| 灯条白色呼吸 | 驱动没接管 | `teleop_up.sh restart` |
| 灯条黄色呼吸 | 零点丢失 | 先做零点校准，别遥操 |
| `can0 不可用` | udev 没起 / 适配器没插 | `lsusb \| grep -i "USB to CAN"`，重插 |
| gRPC 50051 起不来 | 端口被占 / 驱动崩了 | `docker logs airbot-play-runtime` |
| `[PICO] waiting for packets` | adb reverse 掉了 | `adb reverse --list`，没有就重建 |
| adb 看不到头显 | USB 模式退回默认 | 头显里重开 USB 调试并选「调试」模式 |
| 有 RX 但臂不动 | 没武装 / 状态不是 TELEOP_ACTIVE | 松一次扳机，看日志 `Deadman armed` |
| 切模式失败 | 上次 app.py 被 kill 留了脏会话 | `teleop_up.sh restart` |
| 按 A 回零报错 | 关节在官方限位外 | 按错误信息里的关节号，手动推回限位内 |
| 运动一卡一卡 | slew 太慢 | 调 `airbot_sdk.yaml` 的 `slew_linear` / `slew_angular` |
| 腕部跟不上 | servo joint scale 太小 | `servo_scale_joint` 要 > 0.1（当前 0.6） |
| 「前/上」方向反了 | 坐标映射 | 只改 `middle_interface` 的 `transform.axis_map` |

## 手侧

| 症状 | 原因 | 处理 |
|---|---|---|
| 只有 `Manus Core connected`，无手套数据 | **USB 写权限**（最常见） | 用 `-u 0` 进容器 |
| `No compatible license found` | 同上，权限 | 同上 |
| `lsusb` 看到 3325 但 SDK 连不上 | 设备节点权限 | `chmod 666 /dev/bus/usb/00X/00Y` |
| target 话题 Subscription 0 | 驱动没起 | target-only 模式下**正常** |
| `Waiting for right target JointState` | MANUS 没发 `side: Right` / retarget 没 publisher | `ros2 topic echo /manus_glove_0 --once` 看 side |
| `Waiting for right Revo2 feedback` | joint_states 没 publisher | 检查驱动是否起来 |
| `revo2_pid_controller` 不存在 | 驱动没重新 build/source | `ros2 control load_controller -c ... --set-state inactive revo2_pid_controller` |
| `revo2_pid_controller` inactive | 位置/速度控制器占着 command interface | 见 4.7 切换 |
| 启动了但手不动 | controller 状态不对 | `ros2 control list_controllers -c /revo2_right/controller_manager` |
| 双手抢串口 | `auto_detect: true` | 改成 `false`，用 udev 别名 |
| Modbus 探测无响应 | 探错了端口 | 见 1.3，CH340 会回 `md err` |
| `Invalid function code: 0x6D...` | 同上，端口错 | 确认是 FTDI 而非 CH340 |
| 拇指「刹车点头」晃动 | kp 太大 | 降 `thumb_velocity_kp_scale`，或 `velocity_kd` 试 0.01→0.015 |
| 某关节稳定零点偏差 | 反馈标定 | `feedback.position_offsets` |
| `!rclpy.ok()` / `xmlrpc.client.Fault` | CLI daemon 异常 | 前缀 `ROS2CLI_DISABLE_DAEMON=1` |
| 串口 500Hz 不稳 | Modbus 带宽 | 用 `update_rate:=20` |

## 通用

| 症状 | 原因 |
|---|---|
| 设备被抢、行为诡异 | 旧容器还在跑 → `bringup.sh status` 会警告 |
| 在宿主机 source 手侧 install 后各种找不到 | symlink-install 是容器内路径，只能在容器里 source |
| Revo2/Revo3 包混淆 | 不要叠在同一个 overlay |

---

# 第 7 章 调参速查

| 想改什么 | 改哪 |
|---|---|
| 臂：前/上方向 | `middle_interface/config/airbot.yaml` → `transform.axis_map` |
| 臂：单次握持行程上限 | 同上 → `teleop.max_delta_position` / `max_delta_angle` |
| 臂：手抖大 | 同上 → `filter.position_alpha` / `rotation_alpha`（调小更平滑） |
| 臂：可达盒子 | 同上 → `workspace.{x,y,z}` |
| 臂：运动快慢/顿感 | `Robot_control_interface/config/airbot_sdk.yaml` → `slew_linear` / `slew_angular` |
| 臂：跟踪跟不上 | 同上 → `servo_scale_linear` / `_rotational` / `_joint` |
| 臂：home 位姿 | 同上 → `home_joints` |
| 手：拇指侧摆灵敏度 | `manus_revo2_retarget/config/retarget.yaml` → `revo3_thumb.<side>.thumb_meta_range_deg` |
| 手：拇指中立位偏内 | 同上 → `thumb_meta_zero_deg` |
| 手：拇指弯曲映射 | 同上 → `thumb_prox_zero_deg` / `thumb_prox_range_deg` |
| 手：PD 增益 | `revo2_driver/config/revo2_controllers.yaml` → `pd_velocity.*` |
| 手：目标平滑 | 同上 → `target_filter.alpha` / `fast_alpha` / `fast_threshold` |
| 手：关节零点偏差 | 同上 → `feedback.position_offsets` |
| 手：串口/slave/波特率 | `revo2_driver/config/protocol_modbus_<side>.yaml` |
| 手：速度单位换算 | 同上 → `physical_velocity_*_scale` / `_device_min/max` |

---

# 第 8 章 这份代码值得学的设计

看完整个工程，有几个模式是可以直接搬到别的项目里的：

**1. 设备层不认识机器人。** PICO 的 APK 只发原始 XR 位姿，MANUS 的 publisher 只发原始人手关节角。所有机器人语义都在下游。换设备只改一个归一化矩阵，换机器人只改一个 backend。

**2. 统一的中间坐标系。** `teleop_standard`（+X 前、+Y 左、+Z 上）作为所有设备的汇聚点。N 个设备 × M 个机器人从 N×M 个转换降成 N+M 个。

**3. 增量式而非绝对式遥操作。** 离合（deadman）+ 双基线（robot_ref 立即锁存 / vr_ref 延后一帧）+ 行程截断。这套组合解决了工作空间不匹配、启动跳变、操作者疲劳三个问题。

**4. "停"不等于"卸力"。** 松开扳机是保持姿态，不是关电机。这在臂侧（`stop()` 重进 SERVO_CART_POSE 而非 SOFT）和手侧（target 超时归零速度而非切断）都体现了。

**5. 分层限速，各管一段。** 上层做行程截断（防跳变），下层做速度 slew（防冲击）。真机配置里中间那层 velocity_limit 是关掉的——三层限速会互相拖后腿。

**6. 安全默认值。** `middle_interface` 默认 `adapter: dummy`，`revo2_driver` 默认激活位置控制器。都要显式改配置或显式切换才会动真机。

**7. 把踩过的坑写成代码注释。** 这份工程里最有价值的部分是注释：为什么要 `sleep(0.3)`、为什么不能用 `pgrep`、为什么 `--strict` 下不能 deactivate 已 inactive 的 controller、为什么 servo scale 要 1.5/3.0/0.6。这些都是花时间 debug 出来的，写在代码里下一个人就不用再花一次。

## 读码顺序建议

如果要深入，按这个顺序读收益最高：

```text
1. README.md                                           总览
2. .../middle_interface/processing/clutch.py            核心算法，181 行
3. .../middle_interface/processing/pipeline.py          流水线编排，314 行
4. .../middle_interface/processing/head_base.py         最容易忽略但很关键，64 行
5. .../Robot_control_interface/backends/airbot_sdk.py   SDK 对接的全部坑，260 行
6. .../manus_revo2_retarget/retargeters/joint_thumb_retargeter.py  映射数学，243 行
7. .../revo2_driver/src/revo2_pid_controller.cpp        ros2_control controller 范例
8. airbot+hand-workspace/teleop_up.sh                  运维脚本里的经验密度最高
```

第 8 个别跳过——脚本里的注释解释了一堆"为什么必须这个顺序"，那些是文档里没有的。
