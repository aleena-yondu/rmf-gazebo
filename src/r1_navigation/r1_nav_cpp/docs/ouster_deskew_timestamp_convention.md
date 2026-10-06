# Ouster LiDAR Deskew — Timestamp Convention & Fix

## Overview

This document records the investigation and fix for a deskew bug caused by a
misunderstanding of the Ouster PointCloud2 timestamp convention. The bug caused
point cloud scans to fail matching against the map during rotation, while
appearing acceptable during straight-line motion.

---

## Ouster Timestamp Convention (verified from live topic)

### `t` field

Each point in the Ouster PointCloud2 message carries a `t` field (uint32, nanoseconds).
It represents the **offset from the scan start** for that individual point:

```
t = 0 ns        → acquired at scan start (= header.stamp)
t = 99,888,878  → acquired ~99.9 ms after scan start (last point in frame)
```

Verified values for a 1024-column, 10 Hz scan:

| Column | t (ms) |
|--------|--------|
| scan-start col (≈ col 4 this run) | 0.000 |
| +128 cols | ~12.1 |
| +256 cols | ~24.6 |
| +512 cols | ~49.6 |
| +1023 cols | ~99.5 |

**Column-to-column interval**: mean ≈ **97.7 µs**, std ≈ negligible.
No "burst-then-wait" gaps were found — the motor spins at a constant rate.

### Column ordering

Columns in the ROS message are ordered by **azimuth angle (0° → 360°)**,
**not** by acquisition time. The scan starts at whichever azimuth happens to
align with the frame boundary (determined by the motor position at trigger time).

Example from the verified run:

```
col 0–3   →  t ≈ 99.6–99.9 ms  (last columns acquired in this frame)
col 4     →  t = 0 ms           (first column acquired — scan start)
col 5–1023 → t increases linearly to ~99.5 ms
```

The **wrap point** (where t drops from ~100 ms back to 0) shifts each frame.
The deskew code does **not** need to know the wrap point — it uses each point's
`t` value directly.

### `header.stamp`

`header.stamp` equals the **scan start time** (the absolute ROS time of the
point with `t = 0`). This was confirmed by:

1. `t_min = 0` — the earliest point has zero offset.
2. `header.stamp` is ~184 ms behind system clock, consistent with one full
   scan period having elapsed since the hardware trigger.

---

## The Bug

The original deskew implementation assumed `header.stamp` was the timestamp of
the **last** acquired point (`max_t`), not the first:

```cpp
// WRONG — off by ~100 ms
int64_t t_point_ns = header_t_ns - (max_t - t_field);
//                 = scan_start - max_t + t_field
//                 ≈ (scan_start - 100 ms) + t_field
```

This shifted every IMU buffer lookup ~100 ms into the past, causing the
interpolator to fetch angular velocity samples from the **previous frame**.

- During straight-line motion: `wz ≈ 0` in both frames → error negligible.
- During rotation: `wz` is non-zero and different between frames → deskew
  applied the wrong (or even opposite-sign) correction, distorting the scan and
  preventing map matching.

---

## The Fix

```cpp
// CORRECT — header.stamp = scan start, so absolute time = header_t + t_field
int64_t t_point_ns = header_t_ns + static_cast<int64_t>(*iter_t);
```

The rotation-correction magnitude (`delta_t_s = (max_t - t_field) * 1e-9`) was
already correct and was not changed. Only the IMU lookup time was wrong.

File changed: `src/pointcloud_deskew.cpp`, line ~132.

---

## Why Uniformity Does Not Matter for Deskew Correctness

The deskew node iterates over every point and uses its individual `t` value:

```cpp
for each point:
    t_point_ns = header_t_ns + t_field   // absolute acquisition time
    wz         = interpolateWz(t_point_ns)
    theta      = wz * (max_t - t_field) * 1e-9
    // rotate (x, y) by -theta to bring point to reference frame (max_t)
```

Whether columns are uniformly spaced or in what order they appear in the
message is irrelevant — each point carries its own hardware timestamp and is
corrected independently.

---

## Verification Commands

```bash
# Check t field range and header.stamp convention
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
