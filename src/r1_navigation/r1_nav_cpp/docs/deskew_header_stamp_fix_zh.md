# Deskew header.stamp 修复记录

## 现象

- 机器人旋转时，scan 无法正确 match 到地图，产生 snap（位置突跳）
- 直线行驶时也存在 cm 级 snap，但旋转时更严重
- 匀速旋转时 snap 幅度固定（不会越来越大），约 10-20°
- 停止后自动收敛回正确位置，无累积 drift

## 分析过程

### 为什么 snap 是固定的，不是累积的？

这是关键特征。如果是 IMU 积分 bias 导致的，误差会随时间积累（越来越大）。
但实际观察是"固定偏差 + 停止后自动收敛"，这说明是**每帧独立的系统性偏移**，不是积分误差。

### 定位到根本原因

Deskew 节点把所有点补偿到 `max_t`（scan 结束时刻）的机器人姿态：

```
scan 开始 (t=0)    ←── 100ms ──►    scan 结束 (max_t ≈ 100ms)
header.stamp = t_start              点云实际参考时刻 = t_end
```

但输出点云的 `header.stamp` 原样复制，仍然是 `t_start`。

SLAM Toolbox 收到点云后：
1. 读取 `header.stamp = t_start`
2. 查询 EKF `/odom`，拿到机器人在 `t_start` 时的 yaw
3. 用这个 yaw 对点云做 scan matching 初始预测

**问题**：点云代表的是 `t_end` 时刻的视角，但 SLAM Toolbox 用的是 `t_start` 时刻的 yaw。

误差 = `ω × (t_end - t_start)` = `ω × 100ms`

| 旋转速度 | 系统性 yaw 误差 |
|---------|----------------|
| 0.5 rad/s | ~2.9° |
| 1.0 rad/s | ~5.7° |
| 2.0 rad/s | ~11.5° |
| 3.0 rad/s | ~17.2° |

### 为什么停下来会收敛？

停止后 ω = 0，`ω × 100ms = 0`，时间戳误差对 yaw 无影响，SLAM Toolbox 可以正确匹配。

### 为什么匀速旋转时 snap 固定？

ω 恒定 → 每帧误差 `ω × 100ms` 恒定 → 偏差固定，不积累。

## 修复方案

在 deskew 节点输出点云时，将 `header.stamp` 更新为 scan 结束时刻：

```cpp
// 修复前：header.stamp = t_start（错误）
auto out = std::make_unique<sensor_msgs::msg::PointCloud2>(*msg);

// 修复后：header.stamp = t_start + max_t = t_end（正确）
int64_t end_t_ns = header_t_ns + static_cast<int64_t>(max_t);
out->header.stamp.sec     = static_cast<int32_t>(end_t_ns / 1000000000LL);
out->header.stamp.nanosec = static_cast<uint32_t>(end_t_ns % 1000000000LL);
```

修改文件：`src/pointcloud_deskew.cpp`

## 两个 bug 的叠加效应

| 修复 | Bug 内容 | 影响 |
|------|---------|------|
| Fix 1 | `t_point_ns` 用了 scan start 当 scan end | IMU 查询时间偏早 100ms，deskew 用了上一帧的角速度 |
| Fix 2 | 输出 `header.stamp` 未更新为 scan end | SLAM Toolbox 用 scan start 的 yaw 做预测，旋转时系统性偏差 `ω × 100ms` |

两个 bug 方向相同，旋转时叠加，效果特别差。
