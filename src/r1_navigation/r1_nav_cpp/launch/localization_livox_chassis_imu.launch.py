from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from r1_nav_cpp.config_resolver import (
    get_map_path,
    get_nav_config_path,
    get_shared_nav_config_path,
)


def generate_launch_description():
    """
    LIVOX-only localization stack for the no-ZED robot.

    Instead of computing wheel odometry and fusing it with the ZED IMU, this
    pipeline fuses the already-computed chassis twist (/motion_control/chassis_speed)
    with the Livox IMU (/livox/imu) in a robot_localization EKF. SLAM Toolbox
    parameters/map and the AprilTag map->odom relayer are reused unchanged.
    """
    pkg_share = get_package_share_directory('r1_nav_cpp')

    # --- Declare Launch Arguments ---
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=get_nav_config_path(pkg_share, 'slam_toolbox_localization.yaml', lidar='LIVOX'),
        description='Path to SLAM Toolbox parameters for localization mode'
    )

    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=get_map_path(pkg_share, lidar='LIVOX', with_extension=False),
        description='Path to the map file for localization (without extension)'
    )

    declare_enable_slam_arg = DeclareLaunchArgument(
        'enable_slam_localization',
        default_value='true',
        description='Enable SLAM Toolbox localization node and AprilTag relayer'
    )

    declare_ekf_config_arg = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=get_nav_config_path(pkg_share, 'ekf_chassis_imu.yaml', lidar='LIVOX'),
        description='Path to EKF configuration file (chassis twist + Livox IMU)'
    )

    print("[localization_livox_chassis_imu] LIVOX no-ZED localization (chassis twist + Livox IMU)")

    # --- Nodes ---

    chassis_twist_relay_node = Node(
        package='r1_nav_cpp',
        executable='chassis_twist_relay',
        name='chassis_twist_relay',
        output='screen',
        parameters=[{
            'input_topic': '/motion_control/chassis_speed',
            'output_topic': '/motion_control/chassis_twist_cov',
            'vx_variance': 0.01,
            'vy_variance': 0.01,
            'vyaw_variance': 0.05,
            'min_linear_velocity': 0.001,
            'min_angular_velocity': 0.001,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    livox_imu_converter_node = Node(
        package='r1_nav_cpp',
        executable='livox_imu_converter',
        name='livox_imu_converter',
        output='screen',
        parameters=[{
            'input_topic': '/livox/imu',
            'output_topic': '/livox/imu/converted',
            'accel_scale': 9.80665,
            'angular_velocity_variance': 0.01,
            'linear_acceleration_variance': 0.05,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    cmd_vel_relay_node = Node(
        package='r1_nav_cpp',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('ekf_config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('odometry/filtered', '/odom')]
    )

    # Re-stamps the raw chassis LiDAR cloud (timestamps can be seconds off) and
    # republishes it as /livox/points, which is then converted to /scan.
    pointcloud_timestamp_fixer_node = Node(
        package='r1_nav_cpp',
        executable='pointcloud_timestamp_fixer',
        name='pointcloud_timestamp_fixer',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/points',
            'calibration_samples': 10,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    sensor_processing_container = ComposableNodeContainer(
        name='sensor_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pointcloud_to_laserscan',
                plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                name='pointcloud_to_laserscan',
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.5,
                    'min_height': 0.05,
                    'max_height': 1.5,
                    'angle_min': -3.14159,
                    'angle_max': 3.14159,
                    'angle_increment': 0.008,
                    'scan_time': 0.1,
                    'range_min': 0.35,
                    'range_max': 20.0,
                    'inf_epsilon': 1.0,
                    'use_cloud_timestamp': False,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'concurrency_level': 1,
                    'queue_size': 10,
                    'qos_overrides./scan.reliability': 'reliable'
                }],
                remappings=[
                    ('cloud_in', '/livox/points'),
                    ('scan', '/scan')
                ],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
        ],
        output='screen',
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_slam_localization')),
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_name': LaunchConfiguration('map_file'),
                'scan_topic': '/scan'
            }
        ]
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
        output='screen',
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
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_slam_localization')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    dynamic_footprint_node = Node(
        package='r1_nav_cpp',
        executable='dynamic_footprint',
        name='dynamic_footprint',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_slam_params_arg,
        declare_map_file_arg,
        declare_enable_slam_arg,
        declare_ekf_config_arg,

        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, context: print("[Localization] Shutting down cleanly...")
            )
        ),

        chassis_twist_relay_node,
        livox_imu_converter_node,
        cmd_vel_relay_node,
        pointcloud_timestamp_fixer_node,
        sensor_processing_container,
        ekf_node,
        slam_toolbox_node,
        apriltag_pose_corrector_node,
        apriltag_marker_node,
        dynamic_footprint_node,
    ])
