import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource
from r1_nav_cpp.config_resolver import (
    get_map_path,
    get_nav_config_path,
    get_shared_nav_config_path,
)


def generate_launch_description():
    """
    Alternative localization pipeline using FAST-LIO2 instead of
    synchro_odom + bias_estimator + EKF.

    FAST-LIO2 tightly couples Ouster LiDAR + IMU to produce odometry,
    with automatic IMU bias estimation in its state vector (no manual seeding).

    Pipeline:
        /ouster/points  ─┐
                         ├→ FAST-LIO2 → /odom (odom→base_link TF)
        /ouster/imu     ─┘
                                ↓
                          SLAM Toolbox → map→odom TF

    Replaces from localization.launch.py:
        - synchro_drive_odometry
        - online_imu_bias_estimator
        - ekf_filter_node

    Keeps from localization.launch.py:
        - ouster_launch
        - ouster_static_tf
        - slam_toolbox_node
        - cmd_vel_relay
        - dynamic_footprint
    """

    pkg_share = get_package_share_directory("r1_nav_cpp")
    pkg_r1_nav_cpp = FindPackageShare("r1_nav_cpp")

    # === Launch Arguments ===
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=get_nav_config_path(pkg_share, "slam_toolbox_localization.yaml", lidar="OUSTER"),
        description="Path to SLAM Toolbox parameters for localization mode",
    )

    declare_map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=get_map_path(pkg_share, with_extension=False),
        description="Path to map file for SLAM Toolbox (without extension)",
    )

    declare_enable_slam_arg = DeclareLaunchArgument(
        "enable_slam_localization",
        default_value="true",
        description="Enable SLAM Toolbox localization",
    )

    declare_fast_lio_config_arg = DeclareLaunchArgument(
        "fast_lio_config",
        default_value=get_nav_config_path(pkg_share, "fast_lio.yaml", lidar="OUSTER"),
        description="Path to FAST-LIO2 config file",
    )

    # === Ouster LiDAR ===
    ros_namespace = os.environ.get("ROS_NAMESPACE", "")
    if ros_namespace and not ros_namespace.startswith("/"):
        ros_namespace = "/" + ros_namespace

    ouster_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [FindPackageShare("ouster_ros"), "/launch/sensor.launch.xml"]
        ),
        launch_arguments={
            "sensor_hostname": "169.254.154.193",
            "udp_dest": "169.254.1.1",
            "viz": "false",
        }.items(),
    )

    # Static TF: base_link → os_sensor (Ouster mount position)
    # Must match extrinsic_T in fast_lio_ouster.yaml: [0.2, -0.2, 0.2]
    ouster_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ouster_base_tf",
        arguments=["0.2", "-0.2", "0.2", "0", "0", "0", "base_link", "os_sensor"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # === Ouster → FAST-LIO2 Bridge (single process, two fixes) ===
    # 1. IMU QoS: /ouster/imu BEST_EFFORT → /ouster/imu_reliable RELIABLE
    # 2. PointCloud ring: uint16 → uint8  (/ouster/points → /ouster/points_fastlio)
    ouster_fastlio_bridge_node = Node(
        package="r1_nav_cpp",
        executable="ouster_fastlio_bridge.py",
        name="ouster_fastlio_bridge",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # === FAST-LIO2 ===
    # Replaces: synchro_odom + bias_estimator + EKF
    # Outputs:  /odom topic + odom→base_link TF
    # IMU bias: estimated automatically in FAST-LIO state vector
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fast_lio",
        output="screen",
        parameters=[
            LaunchConfiguration("fast_lio_config"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            # FAST-LIO publishes odometry on /Odometry → remap to /odom for SLAM Toolbox
            ("/Odometry", "/odom"),
        ],
    )

    # Static TF: camera_init → odom (FAST-LIO world frame → ROS odom frame alias)
    # FAST-LIO uses "camera_init" as world frame name; this bridges to "odom"
    fast_lio_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fast_lio_frame_bridge",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Static TF: body → base_link (FAST-LIO robot frame → ROS base frame alias)
    # FAST-LIO publishes camera_init→body TF dynamically; "body" must equal "base_link"
    # for the rest of the nav stack (costmap, footprint, cmd_vel) to work correctly.
    fast_lio_body_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fast_lio_body_bridge",
        arguments=["0", "0", "0", "0", "0", "0", "body", "base_link"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # === CMD Velocity Relay ===
    cmd_vel_relay_node = Node(
        package="r1_nav_cpp",
        executable="cmd_vel_relay",
        name="cmd_vel_relay",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # === SLAM Toolbox Localization ===
    # Same as localization.launch.py — unchanged
    from launch.conditions import IfCondition
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_slam_localization")),
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_file_name": LaunchConfiguration("map_file"),
                "scan_topic": "/ouster/scan",
            },
        ],
    )

    # === Dynamic Footprint Visualizer ===
    dynamic_footprint_node = Node(
        package="r1_nav_cpp",
        executable="dynamic_footprint",
        name="dynamic_footprint",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # === AprilTag pose corrector ===
    # Computes the robot's map-frame pose from tag detections and publishes it
    # on /pose_correction; slam_toolbox localization fuses those as unary priors
    # and owns the map->odom TF itself (transform_publish_period > 0 in
    # slam_toolbox_localization.yaml). This node no longer broadcasts any TF.
    apriltag_pose_corrector_node = Node(
        package="r1_nav_cpp",
        executable="apriltag_pose_corrector",
        name="apriltag_pose_corrector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_slam_localization")),
        parameters=[
            get_shared_nav_config_path(pkg_share, "apriltag_pose_corrector.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # === AprilTag marker visualizer ===
    # Publishes the known tag map poses as a latched MarkerArray on
    # /apriltag_map_markers for RViz overlay. Runs whenever the pose corrector
    # runs (same enable_slam_localization gate).
    apriltag_marker_node = Node(
        package="r1_nav_cpp",
        executable="publish_apriltag_markers.py",
        name="apriltag_marker_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_slam_localization")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim_time_arg,
        declare_slam_params_arg,
        declare_map_file_arg,
        declare_enable_slam_arg,
        declare_fast_lio_config_arg,
        #
        # LiDAR
        #ouster_launch,
        #ouster_static_tf,
        #
        # Ouster → FAST-LIO compatibility bridge (single process)
        ouster_fastlio_bridge_node,
        #
        # FAST-LIO2: LiDAR-inertial odometry (replaces synchro_odom + bias_estimator + EKF)
        fast_lio_node,
        fast_lio_frame_tf,   # camera_init = odom  (world frame alias)
        fast_lio_body_tf,    # body = base_link     (robot frame alias)
        #
        # Control
        cmd_vel_relay_node,
        #
        # Localization
        slam_toolbox_node,
        apriltag_pose_corrector_node,
        apriltag_marker_node,
        #
        # Visualization
        dynamic_footprint_node,
    ])
