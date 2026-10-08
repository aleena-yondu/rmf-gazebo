import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Ground-truth localization that keeps the REP-105 tree.

    map -> odom is a static identity. DiffDrive is the only publisher of
    odom -> base_link. AMCL and slam_toolbox are not started.

    Usage:
        ros2 launch r1_sim goal_sim_gt_rep105.launch.py
    """

    pkg_r1_sim = get_package_share_directory('r1_sim')
    warehouse_models_dir = os.path.join(
        get_package_share_directory('rmf_traffic_editor_test_maps'),
        'maps', 'yondu_warehouse', 'models'
    )
    sim_models_dir = '/home/yondu/yondu_fleet_ws/src/r1_sim/models'
    sim_map_yaml = '/home/yondu/yondu_fleet_ws/src/r1_nav_cpp/maps/yondu_simmap.yaml'
    nav2_params_file = os.path.join(pkg_r1_sim, 'config', 'nav2_sim_params_gt.yaml')

    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([
            sim_models_dir,
            warehouse_models_dir,
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_r1_sim, 'worlds', 'yondu_warehouse_sim.sdf'),
        description='Warehouse world. DiffDrive in model.sdf supplies odom -> base_link.'
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_r1_sim, 'rviz', 'nav2_sim.rviz'),
        description='Path to the RViz2 config file'
    )
    declare_map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=sim_map_yaml,
        description='Full path to map YAML file'
    )

    startup_banner = LogInfo(
        msg='\n' + '=' * 70 + '\n'
        + '  GOAL SIMULATION - REP-105 ground truth\n'
        + '  static map -> odom (identity)\n'
        + '  DiffDrive odom -> base_link on /model/sim_robot/tf\n'
        + '  No AMCL, no slam_toolbox, no second pose plugin.\n'
        + '=' * 70
    )

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
        }.items()
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/sim_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/sim_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
        ],
        remappings=[
            ('/model/sim_robot/odometry', '/odom'),
            ('/model/sim_robot/tf', '/tf'),
        ],
    )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.14',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'sim_robot/lidar_link/gpu_lidar',
        ],
        parameters=[{'use_sim_time': True}],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': LaunchConfiguration('map_yaml'),
        }],
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_r1_sim, 'launch', 'navigation_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        startup_banner,
        gz_resource_path,
        declare_world_arg,
        declare_rviz_config_arg,
        declare_map_yaml_arg,
        ign_gazebo,
        ros_gz_bridge,
        static_map_to_odom,
        static_tf_lidar,
        map_server_node,
        TimerAction(period=8.0, actions=[nav2_navigation_launch]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
