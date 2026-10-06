import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource
from r1_nav_cpp.config_resolver import get_nav_config_path, get_shared_nav_config_path


def generate_launch_description():
    """
    FAST-LIO2 mapping pipeline.

    Replaces mapping.launch.py (which uses wheels + EKF) with LiDAR-inertial odometry.
    Drive the robot around to build the map, then save it via SLAM Toolbox.

    Pipeline:
        /ouster/points  ─┐
                         ├→ FAST-LIO2 → /odom (odom→base_link TF) → SLAM Toolbox async
        /ouster/imu     ─┘                                           ↓
                                                              /map + map→odom TF

    Outputs (save with: ros2 run nav2_map_server map_saver_cli -f <map_name>):
        - <map_name>.yaml + <map_name>.pgm  (2D occupancy map for Nav2)
        - SLAM Toolbox serialized map: <map_name>.posegraph + <map_name>.data
        - FAST-LIO PCD: fast_lio_map.pcd  (3D point cloud, saved on shutdown)
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
        default_value=get_shared_nav_config_path(pkg_share, "slam_toolbox_mapping.yaml"),
        description="Path to SLAM Toolbox parameters for mapping mode",
    )

    declare_fast_lio_config_arg = DeclareLaunchArgument(
        "fast_lio_config",
        default_value=get_nav_config_path(pkg_share, "fast_lio_mapping.yaml", lidar="OUSTER"),
        description="Path to FAST-LIO2 config file (mapping mode, PCD save enabled)",
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

    ouster_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ouster_base_tf",
        arguments=["-0.1491", "0.2300", "0.3359", "-0.007705", "-0.002639", "-0.050379", "base_link", "os_sensor"],
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
    # Builds a local 3D map incrementally and outputs odometry.
    # PCD is saved to fast_lio_map.pcd on shutdown (pcd_save_en: true in yaml).
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
            ("/Odometry", "/odom"),
        ],
    )

    # FAST-LIO world frame "camera_init" aliased to "odom"
    fast_lio_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fast_lio_frame_bridge",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # FAST-LIO robot frame "body" aliased to "base_link"
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

    # === SLAM Toolbox (async mapping) ===
    # Builds the 2D occupancy map + posegraph from /ouster/scan.
    # After mapping: call the serialize_map service or use map_saver_cli to save.
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "scan_topic": "/ouster/scan",
            },
        ],
    )

    # === Visualization ===
    map_throttle_node = Node(
        package="topic_tools",
        executable="throttle",
        name="map_throttle",
        arguments=["messages", "/map", "1", "/map_viz"],
        output="screen",
    )

    dynamic_footprint_node = Node(
        package="r1_nav_cpp",
        executable="dynamic_footprint",
        name="dynamic_footprint",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim_time_arg,
        declare_slam_params_arg,
        declare_fast_lio_config_arg,
        #
        # LiDAR
        ouster_launch,
        ouster_static_tf,
        #
        # Ouster → FAST-LIO compatibility bridge (single process)
        ouster_fastlio_bridge_node,
        #
        # FAST-LIO2: LiDAR-inertial odometry + 3D map building
        fast_lio_node,
        fast_lio_frame_tf,
        fast_lio_body_tf,
        #
        # Control
        cmd_vel_relay_node,
        #
        # 2D mapping
        slam_toolbox_node,
        #
        # Visualization
        map_throttle_node,
        dynamic_footprint_node,
    ])
