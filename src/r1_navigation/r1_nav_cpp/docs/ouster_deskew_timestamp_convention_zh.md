# Ouster LiDAR Deskew — 时间戳约定与修复记录

## 背景

本文记录了一个 deskew（点云去畸变）bug 的排查与修复过程。该 bug 导致机器人旋转时点云无法正确匹配到地图，直线行驶时表现正常。

---

## Ouster 时间戳约定（已通过实测验证）

### `t` 字段

PointCloud2 消息中每个点都携带 `t` 字段（uint32，单位 ns），表示该点相对于**本帧扫描起始时刻**的时间偏移：

```
t = 0           → 扫描开始时采集（= header.stamp 对应时刻）
t = 99,888,878  → 扫描开始后约 99.9 ms 采集（本帧最后一个点）
```

实测数据（1024 列，10 Hz）：

| 列号 | t (ms) |
|------|--------|
| 扫描起始列（本次约为 col 4） | 0.000 |
| +128 列 | ~12.1 |
| +256 列 | ~24.6 |
| +512 列 | ~49.6 |
| +1023 列 | ~99.5 |

**相邻列时间间隔**：均值 ≈ **97.7 µs**，标准差极小，无"扫完等待"现象。

### 列的排列顺序

ROS 消息中的列**按方位角（0°→360°）排列，不按采集时间排列**。每帧的扫描起点（t=0 的列）由电机当时的位置决定，每帧可能不同。

本次实测示例：

```
col 0–3    → t ≈ 99.6–99.9 ms（本帧最后采集的列）
col 4      → t = 0 ms         （本帧扫描起点）
col 5–1023 → t 线性递增至 ~99.5 ms
```

t 从 ~100ms 跳回 0 的"折返点"每帧位置不同，但 deskew 代码无需关心，直接用每个点自己的 `t` 值即可。

### `header.stamp`

`header.stamp` 等于**扫描起始时刻**（即 t=0 的点对应的绝对 ROS 时间）。验证依据：

1. `t_min = 0`，说明最早的点偏移为零。
2. `header.stamp` 比系统时间早约 184 ms，与"触发后经过一个完整扫描周期"的预期一致。

---

## Bug 原因

原代码错误地假设 `header.stamp` 是**最后一个点**（max_t）的时刻，而非第一个点：

```cpp
// 错误 — IMU 查询时间偏早约 100 ms
int64_t t_point_ns = header_t_ns - (max_t - t_field);
//                 = scan_start - max_t + t_field
//                 ≈ (scan_start - 100ms) + t_field
```

这导致 IMU buffer 的插值查询时间整体偏移到**上一帧**的时间段：

- 直线行驶：两帧的 `wz ≈ 0`，误差可忽略，看起来正常。
- 旋转时：`wz` 非零且两帧不同，deskew 用了错误的角速度，点云被扭曲，无法 match 到地图。

---

## 修复方案

```cpp
// 正确 — header.stamp = 扫描起始时刻，绝对时间 = header_t + t_field
int64_t t_point_ns = header_t_ns + static_cast<int64_t>(*iter_t);
```

旋转角度的计算量 `delta_t_s = (max_t - t_field) * 1e-9` 本身没有问题，保持不变。只修改了 IMU 查询时间。

修改文件：`src/pointcloud_deskew.cpp`，约第 132 行。

---

## 为什么均匀性对 deskew 无关紧要

deskew 对每个点单独处理，直接使用其 `t` 值：

```cpp
for each point:
    t_point_ns = header_t_ns + t_field        // 该点的绝对采集时间
    wz         = interpolateWz(t_point_ns)    // 插值获取该时刻角速度
    theta      = wz * (max_t - t_field) * 1e-9
    // 将 (x, y) 反转 theta，对齐到参考时刻（max_t）
```

列是否按时间顺序排列、是否均匀分布，均不影响结果——每个点都有自己的硬件时间戳，独立补偿。

---

## 验证命令

```bash
python3 -c "
import rclpy, struct, sys, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

class Check(Node):
    def __init__(self):
        super().__init__('check')
        qos = QoSProfile(depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(PointCloud2, '/ouster/points', self.cb, qos)
    def cb(self, msg):
        t_off = next(f.offset for f in msg.fields if f.name == 't')
        t_vals = [struct.unpack_from('<I', msg.data,
                  (msg.width * r + c) * msg.point_step + t_off)[0]
                  for r in range(1) for c in range(msg.width)]
        t = np.array(t_vals, dtype=np.float64)
        print(f't_min={t.min()*1e-6:.3f}ms at col={t.argmin()}')
        print(f't_max={t.max()*1e-6:.3f}ms at col={t.argmax()}')
        print(f'dt mean={np.diff(t).mean()*1e-3:.2f} us')
        sys.exit(0)
rclpy.init(); rclpy.spin(Check())
"
```
