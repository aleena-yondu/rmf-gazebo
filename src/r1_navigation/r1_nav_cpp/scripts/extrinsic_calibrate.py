#!/usr/bin/env python3
"""
Extrinsic Calibration: LiDAR (os_sensor) -> base_link
======================================================
Run one mode per bag. Combine results at the end.

Modes
-----
  stationary  -- robot still on flat floor -> estimates roll + pitch
  straight    -- robot drives straight     -> estimates yaw
  combine     -- no bag needed             -> prints final yaml + launch args

Usage
-----
  # Step 1: record bag_stationary with robot completely still (~30 s)
  #   ros2 bag record /ouster/points /ouster/imu /tf /tf_static -o ~/bag_stationary
  python3 extrinsic_calibrate.py stationary --bag ~/bag_stationary --save ~/calib_stationary.json

  # Step 2: record bag_straight with robot driving straight ~5 m
  #   ros2 bag record /ouster/points /ouster/imu /synchro_odom /tf /tf_static -o ~/bag_straight
  python3 extrinsic_calibrate.py straight --bag ~/bag_straight --save ~/calib_straight.json

  # Step 3: combine — load saved files (or pass --roll/--pitch/--yaw manually)
  python3 extrinsic_calibrate.py combine \\
      --load ~/calib_stationary.json --load ~/calib_straight.json \\
      --tx 0.144 --ty -0.2582 --tz 0.2915
"""

import argparse
import json
import math
import sys

import numpy as np
import open3d as o3d
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry

_CDR_OPTS = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr",
)


def _heading_from_line_fit(xy: np.ndarray) -> float:
    """
    Fit a line through Nx2 XY points using PCA (SVD) and return the heading angle.
    Far more robust than endpoint-to-endpoint: outlier poses at the start/end
    of a bag don't dominate the result.
    The sign is resolved so that the heading matches the net displacement direction.
    """
    centroid = xy.mean(axis=0)
    _, _, Vt = np.linalg.svd(xy - centroid)
    direction = Vt[0]  # principal axis (unit vector)
    # resolve 180-deg ambiguity: align with net displacement
    net = xy[-1] - xy[0]
    if np.dot(direction, net) < 0:
        direction = -direction
    return math.atan2(direction[1], direction[0])


def _yaw_from_quaternion(q) -> float:
    """Extract yaw from a geometry_msgs/Quaternion (or any object with x,y,z,w)."""
    # siny_cosp = 2*(w*z + x*y),  cosy_cosp = 1 - 2*(y^2 + z^2)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# ─────────────────────────────────────────────
# JSON save / load helpers
# ─────────────────────────────────────────────

def _save_result(path: str, data: dict):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[saved] {path}")


def _load_results(paths: list) -> dict:
    """Merge multiple JSON result files into one dict."""
    merged = {}
    for p in paths:
        with open(p) as f:
            merged.update(json.load(f))
    return merged


# ─────────────────────────────────────────────
# Bag helpers
# ─────────────────────────────────────────────

def _open_bag(bag_path: str, topics: list) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        _CDR_OPTS,
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def _topic_type_map(bag_path: str) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        _CDR_OPTS,
    )
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def _check_bag(bag_path: str, required_topics: list):
    """Print topics in bag and exit if a required one is missing."""
    try:
        topic_map = _topic_type_map(bag_path)
    except Exception as e:
        print(f"[error] cannot open bag '{bag_path}': {e}")
        sys.exit(1)

    print("[bag] topics in bag:")
    for t, typ in topic_map.items():
        print(f"  {t}  ({typ})")

    for t in required_topics:
        if t not in topic_map:
            print(f"\n[error] required topic '{t}' not found in bag.")
            print("        Use --lidar-topic or --odom-topic to override.")
            sys.exit(1)


# ─────────────────────────────────────────────
# Point cloud helpers
# ─────────────────────────────────────────────

def _pc2_to_o3d(msg: PointCloud2, max_range: float = 30.0) -> o3d.geometry.PointCloud:
    pts = _pc2_to_numpy(msg, max_range)
    pcd = o3d.geometry.PointCloud()
    if len(pts):
        pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def _pc2_to_numpy(msg: PointCloud2, max_range: float = 30.0) -> np.ndarray:
    """Return filtered Nx3 float64 array."""
    raw = np.array(list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)))
    if raw.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    pts = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float64)
    r = np.linalg.norm(pts, axis=1)
    return pts[(r > 0.3) & (r < max_range)]


def _prep_for_icp(pcd: o3d.geometry.PointCloud, voxel: float) -> o3d.geometry.PointCloud:
    d = pcd.voxel_down_sample(voxel)
    d.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30)
    )
    return d


# ─────────────────────────────────────────────
# ICP backends — return list of 4x4 pose matrices (one per input frame).
# Poses are expressed in the initial os_lidar frame (/ouster/points frame).
# ─────────────────────────────────────────────

def _run_kiss_icp(frames_np: list, max_range: float) -> list:
    """KISS-ICP backend. frames_np: list of Nx3 float64 arrays."""
    try:
        from kiss_icp.kiss_icp import KissICP
        from kiss_icp.config import KISSConfig
    except ImportError:
        print("[error] kiss-icp not installed. Run: pip install kiss-icp")
        sys.exit(1)

    config = KISSConfig()
    config.data.max_range = max_range
    config.data.min_range = 0.3
    config.data.deskew = False
    config.mapping.voxel_size = 0.5

    odometry = KissICP(config=config)
    poses = [np.eye(4)]
    for pts in frames_np:
        odometry.register_frame(pts, timestamps=np.zeros(len(pts)))
        poses.append(odometry.last_pose.copy())
    return poses


def _run_open3d_icp(frames_np: list, voxel: float, max_corr: float) -> list:
    """Open3D GICP backend. frames_np: list of Nx3 float64 arrays."""
    icp_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=50, relative_fitness=1e-6, relative_rmse=1e-6
    )

    def to_pcd(pts):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return _prep_for_icp(pcd, voxel)

    cumulative = np.eye(4)
    poses = [np.eye(4)]
    skipped = 0
    for i in range(1, len(frames_np)):
        src = to_pcd(frames_np[i])
        tgt = to_pcd(frames_np[i - 1])
        result = o3d.pipelines.registration.registration_generalized_icp(
            src, tgt, max_corr,
            estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
            criteria=icp_criteria,
        )
        if result.fitness < 0.3:
            skipped += 1
            poses.append(poses[-1].copy())
            continue
        cumulative = cumulative @ np.linalg.inv(result.transformation)
        poses.append(cumulative.copy())

    skip_pct = 100.0 * skipped / max(1, len(frames_np) - 1)
    print(f"[open3d] skipped {skipped}/{len(frames_np)-1} pairs ({skip_pct:.0f}%)"
          + ("  ← high skip rate, result unreliable" if skip_pct > 20 else ""))
    return poses


def _extract_icp_metrics(poses: list):
    """Return (xy_flipped Nx2 array, distance_m, heading_rad) from pose list."""
    pos = np.array([p[:3, 3] for p in poses])
    dx, dy = pos[-1, 0] - pos[0, 0], pos[-1, 1] - pos[0, 1]
    dist = math.sqrt(dx**2 + dy**2)
    xy_flipped = pos[:, :2] * np.array([-1.0, -1.0])
    return xy_flipped, dist, _heading_from_line_fit(xy_flipped)


# ─────────────────────────────────────────────
# Odometry helpers
# ─────────────────────────────────────────────

def _odom_xy_yaw(msg: Odometry):
    p = msg.pose.pose.position
    yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
    return p.x, p.y, yaw


# ─────────────────────────────────────────────
# MODE: stationary  ->  roll, pitch
# ─────────────────────────────────────────────

def mode_stationary(args):
    """
    Record the robot completely still on a flat floor for ~30 s.

    Two independent methods, both reported:
      1. IMU gravity vector (primary)  -- accelerometer mean -> roll, pitch
         os_imu has IDENTITY rotation relative to os_sensor (only ~6/12/8 mm offset).
         Formula uses Z-up convention matching the actual os_imu/os_sensor orientation.
      2. LiDAR ground plane RANSAC (cross-check) -- plane normal -> roll, pitch
         Points are in os_lidar frame (180-deg Z from os_sensor). The 180-deg flip
         and the Z-up sign correction cancel, so the formula is the same as Z-down.

    Both should agree within ~0.5 deg. If they diverge significantly, re-record.
    """
    topic_map = _topic_type_map(args.bag)
    has_imu   = args.imu_topic in topic_map
    has_lidar = args.lidar_topic in topic_map

    if not has_imu and not has_lidar:
        print(f"[error] neither '{args.imu_topic}' nor '{args.lidar_topic}' found in bag.")
        print("[bag] topics in bag:")
        for t, typ in topic_map.items():
            print(f"  {t}  ({typ})")
        sys.exit(1)

    if not has_imu:
        print(f"[warn] IMU topic '{args.imu_topic}' not in bag — using LiDAR only.")
    if not has_lidar:
        print(f"[warn] LiDAR topic '{args.lidar_topic}' not in bag — using IMU only.")

    print("[bag] topics in bag:")
    for t, typ in topic_map.items():
        print(f"  {t}  ({typ})")

    read_topics = [t for t, present in
                   [(args.imu_topic, has_imu), (args.lidar_topic, has_lidar)] if present]
    reader = _open_bag(args.bag, read_topics)

    imu_accels = []   # list of (ax, ay, az)
    pcds = []

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == args.imu_topic:
            msg = deserialize_message(data, Imu)
            a = msg.linear_acceleration
            imu_accels.append((a.x, a.y, a.z))

        elif topic == args.lidar_topic:
            msg = deserialize_message(data, PointCloud2)
            pcd = _pc2_to_o3d(msg, max_range=args.max_range)
            if len(pcd.points) >= 100:
                pcds.append(pcd)

    print(f"[stationary] IMU samples   : {len(imu_accels)}")
    print(f"[stationary] LiDAR frames  : {len(pcds)}")

    roll_imu = roll_lidar = None
    pitch_imu = pitch_lidar = None

    # ── Method 1: IMU gravity vector ──
    if imu_accels:
        acc = np.array(imu_accels)
        # discard first 0.5 s worth of samples (sensor warmup) - roughly 50 samples at 100 Hz
        acc = acc[min(50, len(acc)//4):]
        g = acc.mean(axis=0)
        g_norm = np.linalg.norm(g)
        print(f"[IMU] mean gravity vector : {g}  (|g|={g_norm:.4f} m/s²)")

        if abs(g_norm - 9.81) > 1.5:
            print(f"[warn] gravity magnitude {g_norm:.2f} deviates from 9.81 — "
                  "check IMU calibration or topic.")

        # Ouster frame relationships (confirmed from TF):
        #   os_imu  → os_sensor : identity rotation, ~6/12/8 mm translation only.
        #   os_lidar → os_sensor : 180-deg Z rotation (X and Y both flip).
        #
        # os_imu has Z pointing UP (ROS/REP-103 convention), same as os_sensor.
        # The specific-force accelerometer reads the reaction force against gravity.
        # When level: a = [0, 0, +9.81].  Small tilts produce:
        #   a_x ≈ +pitch * g   (nose-up = positive pitch → positive ax)
        #   a_y ≈ -roll  * g   (right-down = positive roll → negative ay)
        #
        # Z-up roll/pitch formulas:
        #   roll  = atan2(-ay, az)
        #   pitch = atan2( ax, sqrt(ay^2 + az^2))
        g_hat = g / g_norm

        roll_imu  = math.atan2(-g_hat[1],  g_hat[2])
        pitch_imu = math.atan2( g_hat[0],  math.sqrt(g_hat[1]**2 + g_hat[2]**2))

        print(f"[IMU] roll  = {math.degrees(roll_imu):+.4f} deg")
        print(f"[IMU] pitch = {math.degrees(pitch_imu):+.4f} deg")

    # ── Method 2: LiDAR ground plane RANSAC ──
    if pcds:
        merged = o3d.geometry.PointCloud()
        for p in pcds:
            merged += p
        merged = merged.voxel_down_sample(voxel_size=0.05)
        merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        print(f"[LiDAR] merged cloud: {len(merged.points)} pts")

        plane_model, inliers = merged.segment_plane(
            distance_threshold=0.05,
            ransac_n=3,
            num_iterations=1000,
        )
        a, b, c, d_plane = plane_model
        normal = np.array([a, b, c], dtype=float)
        normal /= np.linalg.norm(normal)
        if normal[2] < 0:
            normal = -normal
            d_plane = -d_plane
        # distance from sensor origin to ground plane = |d| / |normal| = |d| (normal is unit)
        sensor_height_above_floor = abs(d_plane)

        inlier_ratio = len(inliers) / len(merged.points)
        print(f"[LiDAR] ground plane normal : {normal}")
        print(f"[LiDAR] inlier ratio        : {inlier_ratio:.2%}")

        if inlier_ratio < 0.3:
            print("[warn] low inlier ratio — ground plane fit may be unreliable.")

        roll_lidar  = math.atan2( normal[1], normal[2])
        pitch_lidar = math.atan2(-normal[0], math.sqrt(normal[1]**2 + normal[2]**2))

        print(f"[LiDAR] roll  = {math.degrees(roll_lidar):+.4f} deg")
        print(f"[LiDAR] pitch = {math.degrees(pitch_lidar):+.4f} deg")
        print(f"[LiDAR] sensor height above floor = {sensor_height_above_floor:.4f} m")

    # ── cross-check and print both ──
    if roll_imu is not None and pitch_imu is not None \
            and roll_lidar is not None and pitch_lidar is not None:
        diff_roll  = abs(math.degrees(roll_imu  - roll_lidar))
        diff_pitch = abs(math.degrees(pitch_imu - pitch_lidar))
        if diff_roll > 1.0 or diff_pitch > 1.0:
            print(f"\n[warn] IMU and LiDAR disagree: "
                  f"dRoll={diff_roll:.2f} deg  dPitch={diff_pitch:.2f} deg")
            print("       Possible causes: floor not flat, IMU bias, or robot was bumped.")
        else:
            print(f"\n[ok] IMU and LiDAR agree within {max(diff_roll, diff_pitch):.2f} deg")

    print()
    print("=" * 55)
    print("  RESULTS — stationary mode (pick one for combine)")
    print("=" * 55)

    if roll_imu is not None and pitch_imu is not None:
        print(f"  [IMU — gravity vector]")
        print(f"    roll  = {math.degrees(roll_imu):+.4f} deg  ({roll_imu:+.6f} rad)")
        print(f"    pitch = {math.degrees(pitch_imu):+.4f} deg  ({pitch_imu:+.6f} rad)")
        print(f"    --> combine: --roll {math.degrees(roll_imu):.4f} --pitch {math.degrees(pitch_imu):.4f}")
        print()

    if roll_lidar is not None and pitch_lidar is not None:
        print(f"  [LiDAR — ground plane RANSAC]")
        print(f"    roll  = {math.degrees(roll_lidar):+.4f} deg  ({roll_lidar:+.6f} rad)")
        print(f"    pitch = {math.degrees(pitch_lidar):+.4f} deg  ({pitch_lidar:+.6f} rad)")
        print(f"    --> combine: --roll {math.degrees(roll_lidar):.4f} --pitch {math.degrees(pitch_lidar):.4f}")

    print("=" * 55)

    if args.save:
        result = {}
        if roll_imu is not None and pitch_imu is not None:
            result["roll_imu_deg"]   = math.degrees(roll_imu)
            result["pitch_imu_deg"]  = math.degrees(pitch_imu)
        if roll_lidar is not None and pitch_lidar is not None:
            result["roll_lidar_deg"]  = math.degrees(roll_lidar)
            result["pitch_lidar_deg"] = math.degrees(pitch_lidar)
            result["sensor_height_above_floor"] = sensor_height_above_floor
        _save_result(args.save, result)


# ─────────────────────────────────────────────
# MODE: straight  ->  yaw
# ─────────────────────────────────────────────

def mode_straight(args):
    """
    Record the robot driving in a straight line (~5 m).
    KISS-ICP gives the direction of travel in the LiDAR frame.
    Wheel odometry gives the direction of travel in base_link frame.
    The angular difference is the yaw extrinsic.

    KISS-ICP advantages over frame-to-frame GICP:
      - Builds an adaptive voxel map (not just frame-to-frame) -> less drift
      - Adaptive correspondence threshold -> robust to speed changes
      - No normals needed -> faster
    """
    _check_bag(args.bag, [args.lidar_topic, args.odom_topic])

    # ── load lidar frames and odom poses ──
    reader = _open_bag(args.bag, [args.lidar_topic, args.odom_topic])

    frames = []   # list of Nx3 numpy arrays
    odom   = []   # list of (t_sec, x, y, yaw_rad)

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9

        if topic == args.lidar_topic:
            msg = deserialize_message(data, PointCloud2)
            pts = _pc2_to_numpy(msg, max_range=args.max_range)
            if len(pts) >= 100:
                frames.append(pts)

        elif topic == args.odom_topic:
            msg = deserialize_message(data, Odometry)
            x, y, yaw_q = _odom_xy_yaw(msg)
            odom.append((t, x, y, yaw_q))

    print(f"[straight] lidar frames : {len(frames)}")
    print(f"[straight] odom poses   : {len(odom)}")

    if len(frames) < 5:
        print("[error] too few lidar frames")
        sys.exit(1)
    if len(odom) < 5:
        print("[error] too few odom poses — was /synchro_odom recorded?")
        sys.exit(1)

    # ── run KISS-ICP ──
    print(f"[straight] running KISS-ICP on {len(frames)} frames ...")
    poses_kiss = _run_kiss_icp(frames, args.max_range)
    icp_xy_kiss, dist_kiss, heading_kiss = _extract_icp_metrics(poses_kiss)

    # ── run Open3D GICP ──
    print(f"[straight] running Open3D GICP on {len(frames)-1} pairs ...")
    poses_o3d = _run_open3d_icp(frames, args.voxel, args.max_correspondence)
    icp_xy_o3d,  dist_o3d,  heading_o3d  = _extract_icp_metrics(poses_o3d)

    # ── select primary backend ──
    if args.backend == "kiss":
        heading_lidar, dist_lidar, primary_label = heading_kiss, dist_kiss, "KISS-ICP"
        heading_xcheck, dist_xcheck, xcheck_label = heading_o3d, dist_o3d, "Open3D GICP"
    else:
        heading_lidar, dist_lidar, primary_label = heading_o3d, dist_o3d, "Open3D GICP"
        heading_xcheck, dist_xcheck, xcheck_label = heading_kiss, dist_kiss, "KISS-ICP"

    if dist_lidar < 0.3:
        print(f"[error] {primary_label} total displacement is only {dist_lidar:.3f} m.")
        print("        Did the robot actually move? Try re-recording with more motion.")
        sys.exit(1)

    # ── odom heading: two methods, take the best ──
    odom_arr = np.array(odom)          # (N, 4): t, x, y, yaw_q
    odom_xy  = odom_arr[:, 1:3]        # Nx2
    dx_odom  = odom_arr[-1, 1] - odom_arr[0, 1]
    dy_odom  = odom_arr[-1, 2] - odom_arr[0, 2]
    dist_odom = math.sqrt(dx_odom**2 + dy_odom**2)

    # Method A: PCA line fit through odom XY positions
    heading_odom_pos = _heading_from_line_fit(odom_xy)
    # Method B: mean of odom quaternion yaw angles (circular mean)
    odom_yaws = odom_arr[:, 3]
    heading_odom_quat = math.atan2(np.sin(odom_yaws).mean(), np.cos(odom_yaws).mean())

    # Prefer quaternion heading (not affected by path curvature); use position as cross-check
    pos_quat_diff = abs(math.degrees(
        (heading_odom_pos - heading_odom_quat + math.pi) % (2 * math.pi) - math.pi
    ))
    if pos_quat_diff > 3.0:
        print(f"[warn] odom position heading ({math.degrees(heading_odom_pos):+.2f} deg) "
              f"vs quaternion heading ({math.degrees(heading_odom_quat):+.2f} deg) "
              f"differ by {pos_quat_diff:.1f} deg — robot likely curved.")
        print("       Using quaternion heading (more reliable). Re-record straighter if > 5 deg.")
    heading_odom = heading_odom_quat

    # ── yaw offset ──
    # heading_lidar = heading_odom + yaw_offset  =>  yaw_offset = heading_lidar - heading_odom
    yaw = heading_lidar - heading_odom
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi  # normalise to [-pi, pi]

    # ── cross-check: heading difference between the two backends ──
    backend_heading_diff = abs(math.degrees(
        (heading_lidar - heading_xcheck + math.pi) % (2 * math.pi) - math.pi
    ))

    print()
    print(f"[straight] {primary_label} travel  : {dist_lidar:.3f} m, heading {math.degrees(heading_lidar):+.2f} deg  (os_sensor frame, line fit)  [primary]")
    print(f"[straight] {xcheck_label} travel   : {dist_xcheck:.3f} m, heading {math.degrees(heading_xcheck):+.2f} deg  [cross-check]")
    if backend_heading_diff > 2.0:
        print(f"[warn] ICP backends disagree by {backend_heading_diff:.1f} deg — environment may be ambiguous or featureless.")
    print(f"[straight] odom travel         : {dist_odom:.3f} m")
    print(f"[straight] odom heading (quat) : {math.degrees(heading_odom_quat):+.2f} deg  [used]")
    print(f"[straight] odom heading (pos)  : {math.degrees(heading_odom_pos):+.2f} deg  [cross-check]")
    heading_diff = abs(math.degrees(yaw))
    if heading_diff > 10.0:
        print(f"[warn] yaw offset = {heading_diff:.1f} deg — unusually large. Re-record if environment was ambiguous.")
    print()
    print("=" * 50)
    print("  RESULT — straight mode")
    print("=" * 50)
    print(f"  yaw = {math.degrees(yaw):+.4f} deg  ({yaw:+.6f} rad)")
    print()
    print("  Pass this to the combine step:")
    print(f"    --yaw {math.degrees(yaw):.4f}")
    print("=" * 50)

    if args.save:
        _save_result(args.save, {
            "yaw_deg": math.degrees(yaw),
            "backend_primary": primary_label,
            "heading_lidar_deg": math.degrees(heading_lidar),
            "heading_lidar_xcheck_deg": math.degrees(heading_xcheck),
            "backend_xcheck": xcheck_label,
            "heading_odom_quat_deg": math.degrees(heading_odom_quat),
            "heading_odom_pos_deg": math.degrees(heading_odom_pos),
            "dist_lidar_m": dist_lidar,
            "dist_lidar_xcheck_m": dist_xcheck,
        })


# ─────────────────────────────────────────────
# MODE: combine  ->  final yaml output
# ─────────────────────────────────────────────

def _rpy_to_matrix(roll, pitch, yaw) -> np.ndarray:
    """ZYX (ROS convention): R = Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rx = np.array([[1,  0,   0], [0, cr, -sr], [0,  sr,  cr]])
    Ry = np.array([[cp, 0,  sp], [0,  1,   0], [-sp, 0,  cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy,   0], [0,   0,   1]])
    return Rz @ Ry @ Rx


def _fit_circle(pts: np.ndarray):
    """
    Algebraic circle fit (Kasa method) on Nx2 XY points.
    Returns (cx, cy, radius).
    """
    x, y = pts[:, 0], pts[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x**2 + y**2
    result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = result[0], result[1]
    radius = math.sqrt(result[2] + cx**2 + cy**2)
    return cx, cy, radius


def mode_spin(args):
    """
    Record the robot doing one full slow in-place spin (~360 deg).
    ICP builds the LiDAR trajectory which traces a circle.
    The center of that circle = base_link position in LiDAR frame = (tx, ty).
    The radius cross-checks the physical offset magnitude sqrt(tx^2 + ty^2).
    """
    _check_bag(args.bag, [args.lidar_topic])

    # ── load lidar frames ──
    reader = _open_bag(args.bag, [args.lidar_topic])
    frames = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.lidar_topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        pts = _pc2_to_numpy(msg, max_range=args.max_range)
        if len(pts) >= 100:
            frames.append(pts)

    print(f"[spin] loaded {len(frames)} lidar frames")
    if len(frames) < 10:
        print("[error] too few frames — re-record a full spin")
        sys.exit(1)

    # ── KISS-ICP trajectory ──
    print(f"[spin] running KISS-ICP on {len(frames)} frames ...")
    pose_list = _run_kiss_icp(frames, args.max_range)

    positions = np.array([p[:2, 3] for p in pose_list])  # Nx2 XY only
    print(f"[spin] done — {len(frames)} frames processed")

    # ── outlier filtering: remove ICP poses that jumped too far frame-to-frame ──
    # Pure in-place rotation should move the sensor origin very little between frames.
    # Large jumps indicate ICP failed for that frame and should be excluded.
    diffs = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    median_step = np.median(diffs)
    good_mask = np.concatenate([[True], diffs < max(median_step * 5.0, 0.05)])
    n_removed = np.sum(~good_mask)
    if n_removed > 0:
        print(f"[spin] removed {n_removed} outlier poses (step > 5× median={median_step:.4f} m)")
    pts = positions[good_mask]

    if len(pts) < 10:
        print("[error] too few good poses after outlier filtering — re-record spin")
        sys.exit(1)

    # ── check we actually have a circular trajectory ──
    total_disp = np.linalg.norm(pts[-1] - pts[0])
    if total_disp > 0.5:
        print(f"[warn] start-to-end displacement is {total_disp:.3f} m — "
              "robot may not have completed a full spin. Circle fit may be off.")

    # ── fit circle ──
    cx, cy, radius = _fit_circle(pts)

    # residuals — how well points lie on the circle
    dists = np.linalg.norm(pts - np.array([cx, cy]), axis=1)
    residual_std = float(np.std(dists - radius))

    # ── circle center in os_lidar frame = base_link position in os_lidar frame.
    # Step 1: position of os_lidar in base_link, expressed in os_lidar frame = -center.
    #           tx_lidar = -cx,  ty_lidar = -cy
    # Step 2: /ouster/points is published in os_lidar frame, which has a ~180-deg Z
    #         rotation relative to os_sensor.  Converting to os_sensor frame negates
    #         X and Y again:  tx_sensor = -tx_lidar = cx,  ty_sensor = -ty_lidar = cy.
    # The two negations cancel — the final result in os_sensor frame equals the
    # circle center coordinates directly.
    tx = cx
    ty = cy
    expected_radius = math.sqrt(tx**2 + ty**2)

    print()
    print("=" * 55)
    print("  RESULT — spin mode")
    print("=" * 55)
    print(f"  Circle center (LiDAR frame) : ({cx:+.4f}, {cy:+.4f}) m")
    print(f"  Circle radius               : {radius:.4f} m")
    print(f"  Circle fit residual std     : {residual_std:.4f} m")
    print()
    print(f"  Estimated tx = {tx:+.4f} m  (base_link→sensor X)")
    print(f"  Estimated ty = {ty:+.4f} m  (base_link→sensor Y)")
    print(f"  Radius cross-check sqrt(tx²+ty²) = {expected_radius:.4f} m")
    if residual_std > 0.05:
        print()
        print("  [warn] high residual — the trajectory is not a clean circle.")
        print("         Possible causes: spin was not purely in-place, ICP drift,")
        print("         or too few frames. Re-record a slower, fuller spin.")
    print()
    print("  Pass to combine:")
    print(f"    --tx {tx:.4f} --ty {ty:.4f}")
    print("=" * 55)

    if args.save:
        _save_result(args.save, {"tx": tx, "ty": ty, "radius": radius,
                                 "residual_std": residual_std})


def _resolve(loaded: dict, arg_val, *keys: str, name: str):
    """Return (value, source_label) from CLI arg or JSON dict; exit if not found."""
    if arg_val is not None:
        return arg_val, "manual"
    for k in keys:
        if k in loaded:
            return loaded[k], k.replace("_deg", "").replace("_", " ") + " (from JSON)"
    print(f"[error] {name} not found — pass --{name} or --load a result file")
    sys.exit(1)


def mode_combine(args):
    """
    No bag needed. Loads saved JSON results and/or takes manual --roll/--pitch/--yaw,
    then prints ready-to-paste extrinsic output.
    """
    # start from loaded files, then let explicit args override
    loaded = _load_results(args.load) if args.load else {}

    roll_deg,  roll_src  = _resolve(loaded, args.roll,  "roll_imu_deg",  "roll_lidar_deg",  name="roll")
    pitch_deg, pitch_src = _resolve(loaded, args.pitch, "pitch_imu_deg", "pitch_lidar_deg", name="pitch")
    yaw_deg,   yaw_src   = _resolve(loaded, args.yaw,   "yaw_deg",                          name="yaw")

    # ── resolve tx / ty: spin JSON > CLI arg ──
    tx, tx_src = (float(loaded["tx"]), "spin (from JSON)") if "tx" in loaded else (args.tx, "manual --tx")
    ty, ty_src = (float(loaded["ty"]), "spin (from JSON)") if "ty" in loaded else (args.ty, "manual --ty")

    # ── resolve tz: ground plane height - base_link height > CLI --tz ──
    if "sensor_height_above_floor" in loaded and args.tz == 0.2915:
        # Sanity-check the LiDAR RANSAC before trusting sensor_height_above_floor.
        # If roll_lidar or pitch_lidar is large the RANSAC hit a wall, not the floor.
        lidar_roll_deg  = abs(loaded.get("roll_lidar_deg",  0.0))
        lidar_pitch_deg = abs(loaded.get("pitch_lidar_deg", 0.0))
        lidar_tz_raw    = loaded["sensor_height_above_floor"]
        if lidar_roll_deg > 10.0 or lidar_pitch_deg > 10.0 or lidar_tz_raw > 2.0:
            print(f"\n[warn] LiDAR ground-plane RANSAC looks invalid:")
            print(f"       roll_lidar={lidar_roll_deg:.2f} deg, "
                  f"pitch_lidar={lidar_pitch_deg:.2f} deg, "
                  f"sensor_height={lidar_tz_raw:.3f} m")
            print(f"       The RANSAC likely hit a wall instead of the floor.")
            print(f"       Ignoring sensor_height_above_floor — falling back to --tz default ({args.tz} m).")
            print(f"       Re-record the stationary bag in an open area with clear floor,")
            print(f"       OR pass --tz manually with the known sensor height.\n")
            tz = args.tz
            tz_src = f"fallback --tz default (LiDAR RANSAC rejected)"
        else:
            tz = lidar_tz_raw - args.base_height
            tz_src = (f"ground plane ({lidar_tz_raw:.4f} m) "
                      f"- base_height ({args.base_height:.4f} m)")
    else:
        tz = args.tz
        tz_src = "manual --tz"

    roll  = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw   = math.radians(yaw_deg)

    R = _rpy_to_matrix(roll, pitch, yaw)

    sep = "=" * 60
    print()
    print(sep)
    print("  FINAL EXTRINSIC CALIBRATION RESULT")
    print(sep)
    print(f"  roll  = {roll_deg:+.4f} deg  [{roll_src}]")
    print(f"  pitch = {pitch_deg:+.4f} deg  [{pitch_src}]")
    print(f"  yaw   = {yaw_deg:+.4f} deg  [{yaw_src}]")
    print(f"  tx    = {tx:.4f} m  [{tx_src}]")
    print(f"  ty    = {ty:.4f} m  [{ty_src}]")
    print(f"  tz    = {tz:.4f} m  [{tz_src}]")
    print(sep)
    print()
    print("  -- fast_lio_ouster.yaml / fast_lio_ouster_mapping.yaml --")
    print()
    print(f"  extrinsic_T: [ {tx:.4f}, {ty:.4f}, {tz:.4f} ]")
    print(f"  extrinsic_R: [ {R[0,0]:.6f}, {R[0,1]:.6f}, {R[0,2]:.6f},")
    print(f"                 {R[1,0]:.6f}, {R[1,1]:.6f}, {R[1,2]:.6f},")
    print(f"                 {R[2,0]:.6f}, {R[2,1]:.6f}, {R[2,2]:.6f}]")
    print()
    print("  -- static_transform_publisher in launch files --")
    print("     (x, y, z, roll, pitch, yaw  in metres / radians)")
    print()
    print(f'  arguments=[ "{tx:.4f}", "{ty:.4f}", "{tz:.4f}",')
    print(f'              "{roll:.6f}", "{pitch:.6f}", "{yaw:.6f}",')
    print(f'              "base_link", "os_sensor" ]')
    print(sep)


# ─────────────────────────────────────────────
# Argument parsing
# ─────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Extrinsic calibration: LiDAR -> base_link, one mode per bag",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = p.add_subparsers(dest="mode", required=True)

    # ── shared topic / sensor args ──
    def add_sensor_args(sp):
        sp.add_argument("--bag",              required=True,             help="ROS2 bag directory")
        sp.add_argument("--lidar-topic",      default="/ouster/points",  help="PointCloud2 topic")
        sp.add_argument("--max-range",        default=30.0, type=float,  help="Max point range (m)")
        sp.add_argument("--voxel",            default=0.10, type=float,  help="ICP voxel size (m)")
        sp.add_argument("--max-correspondence", default=1.0, type=float, help="ICP max correspondence dist (m)")
        sp.add_argument("--save", default=None, metavar="FILE.json",     help="Save results to JSON file")
        sp.add_argument("--backend", default="open3d", choices=["kiss", "open3d"], help="ICP backend (default: open3d)")

    # stationary
    sp_stat = sub.add_parser("stationary", help="robot still -> roll, pitch")
    add_sensor_args(sp_stat)
    sp_stat.add_argument("--imu-topic", default="/ouster/imu", help="IMU topic (sensor_msgs/Imu)")

    # spin
    sp_spin = sub.add_parser("spin", help="robot spins in place -> tx, ty")
    add_sensor_args(sp_spin)

    # straight
    sp_str = sub.add_parser("straight", help="robot drives straight -> yaw")
    add_sensor_args(sp_str)
    sp_str.add_argument("--odom-topic", default="/synchro_odom", help="Odometry topic")

    # combine
    sp_comb = sub.add_parser("combine", help="combine results -> final output (no bag needed)")
    sp_comb.add_argument("--load",  nargs="+", metavar="FILE.json",  help="Load one or more saved JSON result files")
    sp_comb.add_argument("--roll",  default=None, type=float, help="Override roll  (degrees)")
    sp_comb.add_argument("--pitch", default=None, type=float, help="Override pitch (degrees)")
    sp_comb.add_argument("--yaw",   default=None, type=float, help="Override yaw   (degrees)")
    sp_comb.add_argument("--tx", default=0.144,   type=float, help="Override X translation (m)")
    sp_comb.add_argument("--ty", default=-0.2582, type=float, help="Override Y translation (m)")
    sp_comb.add_argument("--tz", default=0.2915,  type=float, help="Override Z translation (m)")
    sp_comb.add_argument("--base-height", default=0.0, type=float,
                         help="Height of base_link above floor (m). "
                              "Used with ground plane tz estimate. Default 0.0 (base_link at floor level)")

    return p.parse_args()


def main():
    args = parse_args()
    if args.mode == "stationary":
        mode_stationary(args)
    elif args.mode == "spin":
        mode_spin(args)
    elif args.mode == "straight":
        mode_straight(args)
    elif args.mode == "combine":
        mode_combine(args)


if __name__ == "__main__":
    main()
