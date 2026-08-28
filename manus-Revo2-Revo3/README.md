# MANUS → Revo2 手

合工作区里的手侧：MANUS 手套 → ROS 2 Humble retarget →（可选）`revo2_driver` → Revo2。  
Revo3 是另一套 workspace，不要和 Revo2 叠在同一个 overlay 里。

更细的已验证步骤见 [控制链路.md](控制链路.md)。合容器总顺序见上一级 [README.md](../README.md)。

## 结构

```text
manus-Revo2-Revo3/
  README.md                          本文件
  控制链路.md                        运维/验证记录
  bringup_combined.sh                合容器启动（根目录 bringup.sh 转调这里）
  Dockerfile.combined                Humble 合镜像
  docker-compose.combined.yml
  vendor/airbot/                     打进镜像的 airbot_py wheel
  ManusSDK_v3.1.1/                   MANUS Core SDK
  Revo-Retargeting-revo2_retargeting/   当前主用 ROS workspace
    src/brainco_capabilities/
      manus_ros2                     手套 → /manus_glove_0(右) /manus_glove_1(左)
      manus_ros2_msgs
      manus_revo2_retarget           joint_thumb 映射，发 PID 目标
    src/brainco_drivers/
      revo2_driver                   ros2_control + Stark SDK，Modbus 串口
      hex_glove_driver               可选 Hex 手套 UDP
    src/brainco_description/revo2_description
  Revo-Retargeting-revo3_retargeting/   Revo3（MIT），后加
  brainco_revo3_ros2-*/              Revo3 上游驱动副本
```

```text
MANUS 接收器 USB vendor 3325（必须 root 访问）
  → manus_data_publisher ~60 Hz
  → /manus_glove_0  /manus_glove_1
  → manus_revo2_retarget (joint_thumb，无 IK)
  → /revo2_<left|right>/revo2_pid_controller/target_joint_states
  → revo2_driver（真机才需要）
  → /dev/revo2_hand_left  slave 126
    /dev/revo2_hand_right slave 127
```

容器内：`/home/dt/manus-Revo2-Revo3`  
`ROS_DOMAIN_ID=100`。colcon 是 symlink-install，只在容器里 `source install/setup.bash`。

## 启动顺序

合容器已起来（`../bringup.sh`）。**进容器必须 `-u 0`。**

### A. 只有手套、不驱动真机（已验证）

1. 插 MANUS 接收器，`lsusb` 能看到 `3325`
2. 进容器并 source
   ```bash
   docker exec -it -u 0 airbot-revo-humble bash
   cd /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```
3. 启动前半段
   ```bash
   ros2 launch manus_revo2_retarget pipeline_launch.py \
     hand_mode:=right \
     launch_manus_publisher:=true \
     controller_backend:=ros2_control
   ```
   单手把 `both` 改成 `right` 或 `left`。无手套时用包里的 `sim_manus_glove_publisher`，不要开 publisher 连真接收器。
4. 另开终端看话题
   ```bash
   docker exec -it -u 0 airbot-revo-humble bash
   source /opt/ros/humble/setup.bash
   source /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/install/setup.bash
   ros2 topic hz /manus_glove_0
   ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
   ```
   目标话题 Subscription 为 0 在 target-only 下正常。

### B. 真机 Revo2（会运动）

1. 宿主机/容器内配串口别名（插拔后要能保住）
   ```bash
   docker exec -it -u 0 airbot-revo-humble bash
   cd /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting/src/brainco_drivers/revo2_driver/setup
   bash bootstrap_revo2.sh
   bash check_revo2_setup.sh
   ls -l /dev/revo2_hand_left /dev/revo2_hand_right
   ```
   YAML 里双手必须 `auto_detect: false`，否则抢口。
2. `revo2_driver` 已在合镜像里编过；改过源码再：
   ```bash
   cd /home/dt/manus-Revo2-Revo3/Revo-Retargeting-revo2_retargeting
   source /opt/ros/humble/setup.bash
   python3 -m colcon build --symlink-install --packages-select revo2_description revo2_driver
   source install/setup.bash
   ```
3. 真机一键（driver + 切 PID + retarget）
   ```bash
   ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
     hand_mode:=right \
     controller_backend:=ros2_control \
     launch_plot:=false
   ```
   理想：`revo2_pid_controller` active，`joint_forward_pos_controller` inactive。

停 launch：该终端 Ctrl-C，或

```bash
pkill -f 'pipeline_launch.py|manus_data_publisher|manus_revo2_retarget_node'
```
