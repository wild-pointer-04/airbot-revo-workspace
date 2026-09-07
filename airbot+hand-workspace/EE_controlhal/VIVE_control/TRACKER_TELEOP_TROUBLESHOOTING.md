# VIVE Tracker 遥操坐标映射问题诊断报告

**日期**: 2026-09-04  
**问题**: Tracker 移动方向与机械臂末端移动方向不一致

---

## 问题描述

### 实际观察到的错误行为
- ❌ tracker **上** → 臂**前** （期望：臂上）
- ❌ tracker **下** → 臂**后** （期望：臂下）
- ❌ tracker **前** → 臂**下** （期望：臂前）
- ❌ tracker **后** → 臂**上** （期望：臂后）
- ✅ tracker **左** → 臂**左** （正确）
- ✅ tracker **右** → 臂**右** （正确）

### 期望的正确行为
直觉映射：tracker 和机械臂末端方向完全一致
- tracker 上 → 臂上
- tracker 前 → 臂前
- tracker 左 → 臂左

---

## 系统架构

### 坐标转换链路
```
SteamVR 原始坐标 (+X右, +Y上, -Z前)
    ↓
vr_normalize.py: openxr_to_teleop_position()
    ↓
teleop_standard 世界坐标 (+X前, +Y左, +Z上)
    ↓
head_base.py: relative() + yaw_offset (180°)
    ↓
teleop_standard 身体相对坐标 (+X前, +Y左, +Z上，相对于腰部 tracker)
    ↓
coordinate_transform.py: axis_map 映射
    ↓
机械臂命令坐标 (+X前, +Y左, +Z上)
```

---

## 诊断过程

### 1. 初步分析（错误方向）

**假设**: 问题出在 `axis_map` 配置

**尝试的修改**:
```yaml
# 错误尝试 1: 交换轴
axis_map:
  x: ["z", -1.0]  # robot前 ← -teleop上
  y: ["y", 1.0]   # robot左 ← teleop左
  z: ["x", -1.0]  # robot上 ← -teleop前
```

**结果**: 行为没有变化 ❌

**原因**: 
1. 修改的是 `workspace_vive/workspace/` 目录的配置文件
2. 实际运行的代码在 `airbot+hand-workspace/` 目录
3. **配置文件路径不匹配！**

---

### 2. 发现关键问题

#### 问题 1: 工作目录混淆
- 存在**两份代码库**：
  - `workspace_vive/workspace/EE_controlhal/` （未使用）
  - `airbot+hand-workspace/EE_controlhal/` （实际运行）

- Docker 容器内路径：`/home/ubuntu/airbot+hand-workspace/`

#### 问题 2: `head_base_yaw` 配置

在 `airbot+hand-workspace/EE_controlhal/VIVE_control/config/teleop.yaml` 中发现：

```yaml
head_base: true
head_base_yaw: world
head_yaw_offset_deg: 180.0
```

**关键发现**: 
- `head_base_yaw: world` 表示使用世界坐标系作为 yaw 参考（而非 tracker 朝向）
- `head_yaw_offset_deg: 180.0` 施加了 **180° 旋转**
- 注释说明：操作员面朝 SteamVR +Z（后方），需要 180° 翻转让"手朝前"映射到"机械臂前"

**180° 旋转矩阵效果**:
```
[[-1,  0, 0],     teleop +X (前) → -X (后)
 [ 0, -1, 0],     teleop +Y (左) → -Y (右)
 [ 0,  0, 1]]     teleop +Z (上) → +Z (上) ✓
```

这解释了为什么左右是对的，但前后上下出现了额外的旋转变换。

---

### 3. 理论推导

根据完整链路计算（包含 180° offset）：

**当前配置（恒等 axis_map）的实际效果**:
```python
# head_base 应用 180° 旋转
rotation_180 = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]

# 示例：手部向上移动
steamvr = [0, 1, 0]  # +Y 上
teleop = [0, 0, 1]   # vr_normalize → +Z 上
after_head_base = rotation_180 @ teleop = [0, 0, 1]  # 上不受 180° 影响
robot = [0, 0, 1]    # 恒等 axis_map → 臂上 ✓

# 示例：手部向前移动
steamvr = [0, 0, -1]  # -Z 前
teleop = [1, 0, 0]    # vr_normalize → +X 前
after_head_base = rotation_180 @ teleop = [-1, 0, 0]  # 变成后！
robot = [-1, 0, 0]    # 恒等 axis_map → 臂后 ❌
```

**问题根源**: 
- 180° offset 是为了让操作员"背对机械臂"时，手朝前 = 机械臂前
- 但这导致了 X/Y 轴反转
- 当前的恒等 `axis_map` **可能已经是正确的**，只是操作员的站位/朝向与预期不符

---

## 添加的调试代码

### 1. 在 `pipeline.py` 初始化时打印配置

```python
# 第 62-67 行
self.command_transform = CoordinateTransform.from_config(transform_cfg)

# DEBUG: 打印坐标转换矩阵
logger.warning(f"[PIPELINE INIT] axis_map: {transform_cfg.get('axis_map')}")
logger.warning(f"[PIPELINE INIT] transform matrix R:\n{self.command_transform.R}")

self.head_base = HeadBase(...)
```

### 2. 在 `clutch.py` 中打印运行时增量

```python
# 第 171-181 行
# DEBUG: 记录转换前后的增量
import logging
logger = logging.getLogger(__name__)
if np.linalg.norm(delta_pos) > 0.01:  # 只记录大于1cm的移动
    logger.warning(f"[CLUTCH] delta BEFORE transform: {delta_pos}")

if self.command_transform is not None:
    delta_pos = self.command_transform.transform_position(delta_pos)
    delta_rot = self.command_transform.transform_rotation(delta_rot)
    if np.linalg.norm(delta_pos) > 0.01:
        logger.warning(f"[CLUTCH] delta AFTER transform:  {delta_pos}")
        logger.warning(f"[CLUTCH] transform R:\n{self.command_transform.R}")
```

---

## 当前阻塞问题

### 机械臂校准未完成

**现象**:
```
[WARN] [identify_product]: Please calibrate the robot position
```

**原因**: 
- `airbot-play-runtime` 容器的 FSM 节点要求机械臂校准
- gRPC 端口 50051 未开放，无法建立连接

**尝试的解决方案**:
1. ✅ 驱动容器已启动（`airbot-play-runtime`）
2. ✅ FSM 节点正在运行
3. ❌ ROS2 服务不可用（可能是 Jazzy 环境配置问题）
4. ❌ gRPC 端口 50051 未监听

**需要**: 完成物理校准（按下基座按钮或通过其他界面）

---

## 后续步骤

### 立即测试方案（推荐）

**使用虚拟机械臂验证坐标映射逻辑**:

```bash
docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble python3 app.py --config config/teleop.yaml --dummy-arm
```

这样可以：
1. 查看 `[PIPELINE INIT]` 日志，确认 `axis_map` 加载正确
2. 移动 tracker 时查看 `[CLUTCH]` 日志，观察坐标转换前后的值
3. 验证转换逻辑是否正确，无需等待真实机械臂校准

### 完成机械臂校准后

1. **物理校准机械臂**（方法待确认）

2. **运行真实遥操**:
```bash
docker exec -it -w /home/ubuntu/airbot+hand-workspace/EE_controlhal/VIVE_control \
  airbot-revo-humble python3 app.py --config config/teleop.yaml
```

3. **观察日志**，确认：
   - `[PIPELINE INIT]` 显示的 `axis_map` 和转换矩阵
   - `[CLUTCH]` 显示的增量转换前后值
   - 实际机械臂运动方向

4. **根据日志调整 `axis_map`**（如果需要）

---

## 可能的解决方案

### 方案 1: 确认当前配置是否正确

当前配置（恒等映射）可能已经是正确的，如果：
- 操作员按照注释所说，**背对机械臂**站立
- 180° offset 已经正确补偿了朝向

### 方案 2: 移除或调整 180° offset

如果操作员**面向机械臂**站立，需要：

```yaml
head_yaw_offset_deg: 0.0  # 或者 180.0 改成 0.0
```

### 方案 3: 修改 `axis_map` 补偿 180° 的副作用

如果必须保持 180° offset，但希望直觉映射，可能需要：

```yaml
axis_map:
  x: ["x", -1.0]  # 补偿 180° 对 X 的反转
  y: ["y", -1.0]  # 补偿 180° 对 Y 的反转
  z: ["z", 1.0]   # Z 不受 180° 影响
```

**但需要验证实际效果！** 必须通过日志确认 `head_base` 输出的坐标值。

---

## 关键文件位置

### 配置文件
- **实际使用**: `airbot+hand-workspace/EE_controlhal/VIVE_control/config/teleop.yaml`
- **未使用**: `workspace_vive/workspace/EE_controlhal/VIVE_control/config/teleop.yaml`

### 代码文件
- **Pipeline**: `airbot+hand-workspace/EE_controlhal/middle_interface/processing/pipeline.py`
- **Clutch**: `airbot+hand-workspace/EE_controlhal/middle_interface/processing/clutch.py`
- **HeadBase**: `airbot+hand-workspace/EE_controlhal/middle_interface/processing/head_base.py`
- **CoordinateTransform**: `airbot+hand-workspace/EE_controlhal/middle_interface/processing/coordinate_transform.py`
- **VR Normalize**: `airbot+hand-workspace/EE_controlhal/middle_interface/processing/vr_normalize.py`

---

## 调试清单

- [x] 确认代码路径正确（`airbot+hand-workspace/`）
- [x] 添加 `[PIPELINE INIT]` 日志
- [x] 添加 `[CLUTCH]` 日志
- [x] 清理 Python 缓存（`.pyc` 文件）
- [ ] 使用 `--dummy-arm` 运行，查看日志
- [ ] 完成机械臂校准
- [ ] 使用真实机械臂测试
- [ ] 根据实际日志调整 `axis_map`

---

## 参考资料

### 坐标系定义

**SteamVR / libsurvive (OpenXR Stage)**:
- +X: 右 (right)
- +Y: 上 (up)
- -Z: 前 (forward)

**teleop_standard**:
- +X: 前 (forward)
- +Y: 左 (left)
- +Z: 上 (up)

**机械臂命令坐标系** (与 teleop_standard 相同):
- +X: 前 (forward)
- +Y: 左 (left)
- +Z: 上 (up)

### vr_normalize 转换公式

```python
def openxr_to_teleop_position(position: np.ndarray) -> np.ndarray:
    x, y, z = as_vec3(position)
    return np.array([-z, -x, y], dtype=np.float64)
    # teleop.X = -steamvr.Z
    # teleop.Y = -steamvr.X
    # teleop.Z = steamvr.Y
```

### 测试验证表

| Tracker 动作 | SteamVR 坐标 | teleop 坐标 | 180° 后 | axis_map 后 | 期望臂动作 | 实际臂动作 |
|-------------|-------------|------------|---------|------------|----------|----------|
| 上 (+Y)     | (0, 1, 0)   | (0, 0, 1)  | (0,0,1) | (0, 0, 1)  | 臂上     | 臂前 ❌   |
| 下 (-Y)     | (0,-1, 0)   | (0, 0,-1)  | (0,0,-1)| (0, 0,-1)  | 臂下     | 臂后 ❌   |
| 前 (-Z)     | (0, 0,-1)   | (1, 0, 0)  | (-1,0,0)| (-1, 0, 0) | 臂前     | 臂下 ❌   |
| 后 (+Z)     | (0, 0, 1)   | (-1, 0, 0) | (1,0,0) | (1, 0, 0)  | 臂后     | 臂上 ❌   |
| 左 (-X)     | (-1, 0, 0)  | (0, 1, 0)  | (0,-1,0)| (0,-1, 0)  | 臂左     | 臂左 ✓   |
| 右 (+X)     | (1, 0, 0)   | (0,-1, 0)  | (0,1,0) | (0, 1, 0)  | 臂右     | 臂右 ✓   |

**关键观察**: 
- 左右正确，说明 Y 轴映射没问题
- 上下和前后都错了，说明 X/Z 轴有问题
- 180° offset 对 X/Y 都做了反转，但只有 Y 是正确的

这可能意味着：
1. 实际的 `head_base` 输出不是我们计算的值
2. 或者还有其他隐藏的转换环节

**必须通过日志确认实际的数值流动！**
