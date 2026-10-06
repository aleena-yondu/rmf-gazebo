# SLAM & IMU Debug Log

---

## 2026-03-17 — IMU Bias 校正 + SLAM Drift 调试

### 背景

机器人在原地旋转时 slam_toolbox 产生 map drift，CW/CCW 不对称。目标是通过 IMU gyro z 轴 bias 校正消除旋转漂移。

---

### Issue 1：旋转时 map drift（已解决）

**现象**：CW/CCW 原地旋转时 SLAM 地图出现不对称漂移。

**根本原因**：Ouster IMU gyro z 轴存在静态 bias（-0.009294 rad/s）和随角速度变化的动态 bias。

**解决方案**：
- 实现 `online_imu_bias_estimator` 节点，两区间校正策略：
  - 静止时：EMA 估计静态 bias
  - 运动时：quadratic 模型 `b(ω) = quad_b0 + quad_k·ω + quad_k2·ω²`
- 多速度校准（10包 × 5速度，KISS-ICP 参考，n=11106）：
  ```yaml
  quad_b0:  0.001815
  quad_k:  -0.007437
  quad_k2: -0.016103   # R²=0.34, MAE < 0.4 mrad/s
  ```
- EKF 已确认使用 `/imu/data_bias_corrected`

**状态**：✅ 近处（<3m）旋转 drift 解决

---

### Issue 2：静止时 corrected wz 非零（已澄清，非 bug）

**现象**：stationary=true 时，log 里 corrected 值不完全等于 0。

**结论**：正常现象。`corrected = raw_wz - EMA`，EMA 是历史均值，每次采样有随机噪声，差值即为残余。EMA 收敛后应在 0 附近随机波动。

注意：`-0.009294` 是 stationary bias 的均值，不是白噪声幅度，两者是完全不同的量。

---

### Issue 3：远处物体（>5m）旋转时仍有 drift（已分析，待处理）

**现象**：近处物体（<3m）不再 drift，远处物体（>5m）仍有 drift。

**根本原因**：缺少 within-scan LiDAR deskewing。

当前管线：
```
/ouster/points → pointcloud_to_laserscan (target_frame: base_link) → /scan → slam_toolbox
```
`pointcloud_to_laserscan` 对整帧点云（100ms）只做一次单一 TF 变换，不是逐点运动补偿。以 1 rad/s 旋转时，100ms 内转过约 0.1 rad，10m 处位置误差约 **1m**，2m 处仅 **0.2m**。

**IMU bias 校正无法修复此问题**，因为它只改善帧间角度估计（EKF），无法修正帧内点云扭曲。

**待处理**：确认 Ouster driver 是否支持并已启用 IMU-based deskewing。

---

### Issue 4：直线平移时出现 yaw drift（已分析，待修复）

**现象**：旋转 drift 解决后，直线平移反而出现 yaw drift，slam_toolbox 难以纠正。

**根本原因**：calibration 方法论与代码假设不匹配。

校准脚本先减去 static bias 再拟合：
```python
error = (gyro_z - stationary_bias_z) - ref_interp
```
因此 `quad_b0 = 0.001815 ≈ 0`，**不含** static bias 分量。

但 C++ 代码假设 "b0 already encodes static component"，运动时将 EMA 项设为 0：
```cpp
double static_bias_z = stationary ? gyro_bias_[2] : 0.0;  // 运动时为 0 → 错误
```

直线平移时（wz≈0）：static bias（-0.007 rad/s）完全未校正，持续流入 EKF。
0.5 m/s 走 10m（约20秒）→ 累积 yaw 误差 ≈ 0.007 × 20 = **0.14 rad = 8°**。

**待修复**（一行代码）：
```cpp
// online_imu_bias_estimator.cpp, onImu(), line 134
// 修改前：
double static_bias_z = stationary ? gyro_bias_[2] : 0.0;
// 修改后：
double static_bias_z = gyro_bias_[2];  // 运动时 frozen，但始终施加
```

> **注意**：此修复与 calibration script pre-subtract stationary_bias_z 必须配套使用。

---

### Issue 5：旋转时 map 偏移，停下 map jump（已分析）

**现象**：旋转时 map 出现旋转方向的 mismatch 偏移，停下来后 map jump 回正确位置。

**结论**：这不是单纯的 delay，而是多层原因叠加：

| 原因 | 类型 | 优先级 |
|---|---|---|
| 无 deskewing，旋转时扫描畸变，slam_toolbox 只能找到近似 match | 系统误差，停止时被干净 scan 纠正 | 最高 |
| EKF 20Hz，每次更新最多 50ms 预测延迟（1 rad/s 时 = 2.9°） | 真正的 delay | 中 |
| `scan_queue_size` 过大（50），旋转时处理积压 | 积压延迟 | 已修复 |

**已处理**：`scan_queue_size: 50 → 5`（避免旋转时 scan 积压）

**根本解决**：需要启用 deskewing（见 Issue 3）。

---

### 配置变更记录（2026-03-17）

| 文件 | 变更 | 原因 |
|---|---|---|
| `mapping.launch.py` | Ouster TF 更新为最新标定值（无旋转偏移） | 与 localization 统一 |
| `mapping.launch.py` | synchro_odom 增加 `vx_scale_factor: 0.96`、`min_*_velocity: 0.001` | 与 localization 保持一致 |
| `slam_toolbox_mapping.yaml` | `resolution: 0.01 → 0.05` | 减少内存占用，加速 loop closure |
| `slam_toolbox_mapping.yaml` | `correlation_search_space_resolution: 0.02 → 0.05` | 与地图分辨率对齐 |
| `slam_toolbox_mapping.yaml` | `max_laser_range: 15.0 → 20.0` | 扩展有效建图范围 |
| `slam_toolbox_mapping.yaml` | `minimum_travel_heading: 0.1 → 0.02` | 更频繁更新，减少旋转延迟 |
| `slam_toolbox_mapping.yaml` | `scan_buffer_size: 15 → 30` | 更多 scan history 用于 loop closure |
| `slam_toolbox_mapping.yaml` | `scan_queue_size: 10 → 50 → 5` | 最终改为 5，避免旋转时积压 |
| `slam_toolbox_mapping.yaml` | `loop_search_space_dimension: 4.0 → 8.0` | 恢复默认，扩大 loop closure 搜索范围 |

---

### 待处理事项

- [x] 修复 `online_imu_bias_estimator.cpp` line 134（static bias 运动时不校正 bug）— 已改为 `double static_bias_z = gyro_bias_[2];`
- [x] 实现自定义 `pointcloud_deskew` 节点（`src/pointcloud_deskew.cpp` + `include/pointcloud_deskew.hpp`）
  - 读取 Ouster `t` 字段（uint32，ns from scan start）
  - 从 `/imu/data_bias_corrected` IMU buffer 插值 wz
  - 对每个 point 应用 2D rotation R(-theta) 补偿帧内旋转
  - mapping.launch.py / localization.launch.py 均已加入，`cloud_in` → `/ouster/points_deskewed`
- [ ] 验证平移 drift 修复后效果（line 134 fix）
- [ ] 验证 deskewing 启用后远处（>5m）drift 是否消失
