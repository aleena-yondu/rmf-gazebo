from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from r1_nav_cpp.config_resolver import (
    get_lidar,
    get_map_path,
    get_nav_config_path,
    get_shared_nav_config_path,
)
from r1_nav_cpp.sensor_pipeline import build_sensor_container


def generate_launch_description():
    """
    Unified localization pipeline for both Livox and Ouster LiDARs.
    Reads $LIDAR env var to select sensor pipeline, EKF config, SLAM params,
    and IMU bias estimator configuration.
    """
    pkg_share = get_package_share_directory('r1_nav_cpp')
    pkg_find = FindPackageShare('r1_nav_cpp')
    lidar = get_lidar()
    lidar_lower = lidar.lower()

    # --- Declare Launch Arguments ---
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=get_nav_config_path(pkg_share, 'slam_toolbox_localization.yaml'),
        description='Path to SLAM Toolbox parameters for localization mode'
    )

    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=get_map_path(pkg_share, with_extension=False),
        description=(
            'Path to the map file for localization (without extension). '
            'Defaults to maps/<LOCATION>/<LIDAR>/<MAP_NAME>.'
        ),
    )

    declare_enable_slam_arg = DeclareLaunchArgument(
        'enable_slam_localization',
        default_value='true',
        description='Enable SLAM Toolbox localization node (set false when using AMCL)'
    )

    declare_ekf_config_arg = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=get_nav_config_path(pkg_share, 'ekf.yaml'),
        description='Path to EKF configuration file'
    )

    declare_laser_filter_config_arg = DeclareLaunchArgument(
        'laser_filter_config_file',
        default_value=get_shared_nav_config_path(pkg_share, 'laser_filter_config.yaml'),
        description='Path to laser filter configuration file'
    )

    declare_vx_scale_arg = DeclareLaunchArgument(
        'vx_scale_factor', default_value='1.0',
        description='Calibration factor for forward velocity'
    )
    declare_vy_scale_arg = DeclareLaunchArgument(
        'vy_scale_factor', default_value='1.0',
        description='Calibration factor for lateral velocity'
    )
    declare_yaw_scale_arg = DeclareLaunchArgument(
        'yaw_scale_factor', default_value='1.0',
        description='Calibration factor for odometry rotation (increase if under-rotating)'
    )

    print(f"[localization] Using LIDAR={lidar}")

    # --- Shared Nodes ---

    # Subscribe directly to the raw HDAS chassis feedback topic. The previous
    # `<ros_namespace>/hdas/feedback_chassis` indirection was the corrected
    # republish from `timestamp_corrector`, which is no longer launched -- the
    # HDAS clock is system-aligned within ~1 ms (validated 2026-05-04) so the
    # corrector was both unnecessary and harmful (it overwrote header.stamp
    # per-message and flattened HDAS's intra-burst 5 ms spacing during wire
    # stalls). See r1_nav_cpp/scripts/hdas_clock_probe.py for the validation.
    chassis_feedback_topic = "/hdas/feedback_chassis"

    if lidar == 'LIVOX':
        # 0.96 -> 1.0: chassis suspected to under-report vx; 0.96 was actively
        # shrinking readings and contributed to along-aisle compression. MUST
        # stay in sync with mapping.launch.py and lifelong.launch.py.
        odom_vx_scale = 1.0
        odom_vy_scale = 1.0
        odom_yaw_scale = 1.0
    else:
        odom_vx_scale = 1.0
        odom_vy_scale = 1.0
        odom_yaw_scale = 1.0

    # Sensor pipeline (synchro_odom + imu_bias + p2l + (deskew/laser_filter on
    # OUSTER)) all in one intra-process container. See
    # r1_nav_cpp/sensor_pipeline.py for the rationale and full topology.
    sensor_container = build_sensor_container(
        lidar=lidar,
        pkg_share=pkg_share,
        use_sim_time=LaunchConfiguration('use_sim_time'),
        chassis_feedback_topic=chassis_feedback_topic,
        odom_yaw_scale=odom_yaw_scale,
        odom_vx_scale=odom_vx_scale,
        odom_vy_scale=odom_vy_scale,
        laser_filter_config_file=LaunchConfiguration('laser_filter_config_file'),
        # Keep localization aligned with mapping/lifelong: LIVOX p2l range_max
        # 32.0 gives 4 m headroom over slam max_laser_range=28.0, and
        # min_height=0.1 keeps the scan signature consistent across modes.
        livox_p2l_min_height=0.1,
        livox_p2l_range_max=32.0,
    )

    cmd_vel_relay_node = Node(
        package='r1_nav_cpp',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        # 'both' (vs the previous 'screen') routes stdout AND stderr to BOTH
        # the live console AND ROS_LOG_DIR/<exe>_<pid>_*.log. With 'screen'
        # the per-node log was empty whenever the node only used std::cout
        # (which made post-mortem debugging impossible -- see the 7pm May 5
        # localization incident). Same change applied to every Node below.
        output='both',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # robot_localization::ekf_node is NOT a rclcpp::Component in Humble, so it
    # cannot join the intra-process container. Kept as a regular Node; its
    # inputs (/imu/data_bias_corrected, /synchro_odom) are small + low-rate
    # and the cross-process hop is acceptable.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='both',
        parameters=[
            LaunchConfiguration('ekf_config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('odometry/filtered', '/odom')]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        # 'both' instead of 'screen': slam_toolbox writes most of its
        # diagnostics (Karto match scores, pose graph events, scan-matching
        # warnings) to std::cout, NOT through rclcpp. With output='screen'
        # those went only to the tmux pane and were lost the moment the
        # session was killed. 'both' tees them into the per-node log file.
        output='both',
        condition=IfCondition(LaunchConfiguration('enable_slam_localization')),
        # NOTE: slam_toolbox publishes /map directly via the `map_name: /map`
        # parameter set in {livox,ouster}/slam_toolbox_localization.yaml.
        # There is no dedicated nav2_map_server in navigation.launch.py at
        # the moment (it was removed to match the older flow), so this
        # publisher IS the source the costmap StaticLayer subscribes to.
        # See slam_toolbox_common.cpp `setROSInterfaces()` for both
        # publishers (OccupancyGrid + MapMetaData) being constructed off
        # `map_name_`.
        # Parameter precedence (ROS 2 reads sources in order; later wins):
        #   1. (lowest) Inline defaults dict — provides use_sim_time,
        #      scan_topic, AND a map_file_name fallback derived from
        #      $LIDAR + $LOCATION + $MAP_NAME via get_map_path(). The
        #      `map_file:=/path/no_ext` launch arg overrides that env-var
        #      default from the CLI without touching any file.
        #   2. (highest) slam_params_file YAML — if it sets
        #      `map_file_name:` directly, the YAML wins over both the
        #      launch arg and the env-var path. Use this when you want
        #      the YAML to be the per-run source of truth (e.g. picking
        #      a hand-saved map off disk for a one-off localization
        #      session). Leave map_file_name COMMENTED in the YAML to
        #      fall back to the env-var/launch-arg default.
        # NOTE: this is the *opposite* of the historical layout, where
        # the inline dict came last and stomped on the YAML's
        # map_file_name. If you find the env-var path is being ignored
        # unexpectedly, check that map_file_name isn't set in the YAML.
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_name': LaunchConfiguration('map_file'),
                'scan_topic': '/scan'
            },
            LaunchConfiguration('slam_params_file'),
        ]
    )

    dynamic_footprint_node = Node(
        package='r1_nav_cpp',
        executable='dynamic_footprint',
        name='dynamic_footprint',
        output='both',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # AprilTag pose corrector. Computes the robot's map-frame pose from tag
    # detections and publishes it on /pose_correction; slam_toolbox localization
    # fuses those as unary priors and owns the map->odom TF itself
    # (transform_publish_period > 0 in slam_toolbox_localization.yaml). This
    # node no longer broadcasts any TF.
    apriltag_pose_corrector_node = Node(
        package='r1_nav_cpp',
        executable='apriltag_pose_corrector',
        name='apriltag_pose_corrector',
        output='both',
        condition=IfCondition(LaunchConfiguration('enable_slam_localization')),
        parameters=[
            get_shared_nav_config_path(pkg_share, 'apriltag_pose_corrector.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # AprilTag marker visualizer. Publishes the known tag map poses as a latched
    # MarkerArray on /apriltag_map_markers for RViz overlay. Runs whenever the
    # pose corrector runs (same enable_slam_localization gate).
    apriltag_marker_node = Node(
        package='r1_nav_cpp',
        executable='publish_apriltag_markers.py',
        name='apriltag_marker_publisher',
        output='both',
        condition=IfCondition(LaunchConfiguration('enable_slam_localization')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_slam_params_arg,
        declare_map_file_arg,
        declare_enable_slam_arg,
        declare_ekf_config_arg,
        declare_laser_filter_config_arg,
        declare_vx_scale_arg,
        declare_vy_scale_arg,
        declare_yaw_scale_arg,

        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, context: print("[Localization] Shutting down cleanly...")
            )
        ),

        # One process for the whole sensor chain (intra-process zero-copy):
        # synchro_drive_odometry, online_imu_bias_estimator,
        # pointcloud_to_laserscan, and (OUSTER only) pointcloud_deskew +
        # laser_filters/ScanToScanFilterChain.
        sensor_container,

        cmd_vel_relay_node,

        # ekf_node, slam_toolbox, dynamic_footprint stay as their own
        # processes -- none are rclcpp::Component plugins (or, for
        # slam_toolbox, intentionally have their own thread).
        ekf_node,
        slam_toolbox_node,
        apriltag_pose_corrector_node,
        apriltag_marker_node,
        # dynamic_footprint_node intentionally disabled (was already
        # commented out before this refactor).
    ])
