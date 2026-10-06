## Estimate 
Rigid‑body transform between
- ~~IMU frame ↔ LiDAR frame~~ (Provided by Ouster)
- IMU frame ↔ base_link (robot body)
  

### ROS2 bag

Tool |  Comments
----|---
 ~~ros2_calibrate~~ | Calibration of sensor geometry and orientations (manual refinement) but requires _multiple independent sensors_ |
 pcl_icp | ICP
 fast_gicp | ICP: scan alignment (better for offline than pcl_icp) |
 hdl_graph_slam | ICP: scan matching + extrinsics tweaking (LiDAR odometry and mapping) |
 ~~cartographer~~ | ICP: global pose constraints **Verification** |
 ~~direct_lidar_inertial_odometry (DLIO) or KISS-ICP~~ | heavy Overkill |
 extrinsic_lidar_vehicle_calibration (Tier IV) | Offline ICP‑based LiDAR ↔ vehicle calibration |
 ~~lidar_calibration (Autoware)~~ | **Manual** + visual |
 ~~RTAB-Map~~ | SLAM-based **verification** |
 (FAST‑LIO/LIO-SAM) | fused odometry (check trajectory stability and yaw drift) **verification** |

#### [Limitations]
We cannot use checkerboard or calibration targets 

### How ICP series are used
1. Replay bag with LiDAR + IMU topic.
2. **Extract overlapping LiDAR scans** from various poses
3. Choose a reference scan or map (first scan or initial point cloud)
4. **Align them** to each other or to a reference model → accumulate point-to-point errors
5. Solve for the transform (LiDAR ↔ base_link)
6. Output: a 6-DoF transform

### Low Priority due to complexity
If problem persists after extrinsics calibration:  
- LIO‑SAM/FAST‑LIO, would fuse the LiDAR and IMU, supporting extrinsic calibration.
- point clouds + IMU → high‑quality odometry
- Replay bag through them and compare rotation estimate, yaw drift, and pose error after in-place rotation.

---

Quick Sum:
This flow directly addresses rotational map drift by targeting extrinsics first, which is the most likely cause.
Assume that ICP-based offline calibration is sufficient and gives a quantitative residual error to guide corrections.