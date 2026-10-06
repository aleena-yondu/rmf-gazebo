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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    GPAL Simulation Launch File

    Launches the complete navigation stack for testing gpal_random_navigator
    in the yondu_warehouse Ignition Gazebo world.

    Components:
    1. Ignition Gazebo (yondu_warehouse_sim world)
    2. ros_gz_bridge (cmd_vel, scan, odom, clock, tf)
    3. Static TF: base_link -> lidar_link
    4. AMCL (map -> odom from the occupancy grid; no posegraph on disk)
    5. Map Server (publishes /map OccupancyGrid for costmaps)
    6. Nav2 navigation stack (DiffDrive MPPI controller)
    7. GPAL Random Navigator
    8. RViz2 visualization

    Usage:
        ros2 launch r1_sim gpal_sim.launch.py

    Then start navigation with:
        ros2 service call /gpal/navigate r1_interfaces/srv/GpalNavigate "{command: 'start'}"
    """

    pkg_r1_sim = get_package_share_directory('r1_sim')

    # The generated warehouse model directory (from rmf_traffic_editor_test_maps)
    warehouse_models_dir = os.path.join(
        get_package_share_directory('rmf_traffic_editor_test_maps'),
        'maps', 'yondu_warehouse', 'models'
    )

    # Source models: sim_robot and yondu_warehouse_yondu live here.
    sim_models_dir = '/home/yondu/yondu_fleet_ws/src/r1_sim/models'

    # r1_nav_cpp maps directory — try package resolution, fall back to source path
    try:
        pkg_r1_nav_cpp = get_package_share_directory('r1_nav_cpp')
    except Exception:
        # Fall back to source tree path if r1_nav_cpp isn't installed
        pkg_r1_nav_cpp = os.path.join(
            os.path.expanduser('~'),
            'yondu_ws', 'src', 'r1_navigation', 'Yondu_Nav', 'src', 'r1_nav_cpp'
        )

    # === Environment Variables ===
    # Ignition needs to find both the warehouse model and our sim_robot model.
    # sim_models_dir is listed FIRST so our modified building model (with wall
    # collision removed) takes priority over the original from rmf_traffic_editor.
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([
            sim_models_dir,
            warehouse_models_dir,
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    # === Launch Arguments ===
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='/home/yondu/yondu_fleet_ws/src/r1_sim/worlds/yondu_warehouse_sim.sdf',
        description='Path to the Ignition Gazebo world file'
    )

    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_r1_sim, 'rviz', 'nav2_sim.rviz'),
        description='Path to the RViz2 config file'
    )

    declare_warehouse_config_arg = DeclareLaunchArgument(
        'warehouse_config',
        default_value='gpa_logistics_warehouse.yaml',
        description='Warehouse configuration file with target locations'
    )

    # Linked to the workspace-root occupancy grid (yondu_simmap.yaml + .pgm).
    sim_map_yaml = '/home/yondu/yondu_fleet_ws/src/r1_nav_cpp/maps/yondu_simmap.yaml'

    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/yondu/yondu_fleet_ws/yondu_simmap',
        description='Posegraph prefix (yondu_simmap). Unused: this launch uses AMCL.'
    )

    declare_map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=sim_map_yaml,
        description='Full path to map YAML file'
    )

    # === Startup Banner ===
    startup_banner = LogInfo(
        msg='\n' + '=' * 70 + '\n'
        + '  GPAL SIMULATION - yondu_warehouse\n'
        + '  Robot: sim_robot (diff-drive + lidar)\n'
        + '  Localization: map_server + AMCL (yondu_simmap)\n'
        + '=' * 70 + '\n'
        + '  Gazebo starts unpaused (-r).\n'
        + '  In RViz, set the 2D Pose Estimate, then use Nav2 Goal.\n'
        + '    ros2 service call /gpal/navigate r1_interfaces/srv/GpalNavigate '
        + '"{command: \'start\'}"\n'
        + '=' * 70
    )

    # === 1. IGNITION GAZEBO ===
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            # -r starts the world running. The world path stays overridable.
            'gz_args': ['-r ', LaunchConfiguration('world')],
        }.items()
    )

    # === 2. ROS-GZ BRIDGE ===
    # Single config. DiffDrive publishes gz.msgs.Pose_V on
    # /model/sim_robot/tf and gz.msgs.Odometry on /model/sim_robot/odometry.
    # Do not also bridge global /tf: that topic has no publisher and a
    # second bridge shows up as an extra /tf publisher with no data.
    bridge_config = os.path.join(pkg_r1_sim, 'config', 'bridge_config.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': bridge_config,
        }],
    )

    # === 3. STATIC TF: base_link -> lidar frame ===
    # Ignition uses scoped frame names: sim_robot/lidar_link/gpu_lidar
    # The lidar is mounted 0.14m above base_link center
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

    # === 4. MAP SERVER ===
    # Publishes /map. lifecycle_manager_navigation activates it together
    # with AMCL. A second lifecycle manager would fight that bond.
    # yaml_filename is the absolute path ending in .yaml.
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

    # === 5. AMCL ===
    # map -> odom. slam_toolbox localization needs a posegraph
    # (yondu_simmap.posegraph) that is not on disk.
    nav2_params_file = os.path.join(pkg_r1_sim, 'config', 'nav2_sim_params.yaml')

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}],
    )

    # === 6. NAV2 NAVIGATION STACK ===

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

    delayed_nav2 = TimerAction(
        period=8.0,
        actions=[nav2_navigation_launch]
    )

    # === 7. GPAL RANDOM NAVIGATOR ===
    # Not built in r1_nav_cpp. Send goals from RViz (Nav2 Goal) instead.
    # gpal_navigator_node = Node(
    #     package='r1_nav_cpp',
    #     executable='gpal_random_navigator',
    #     name='gpal_random_navigator',
    #     output='screen',
    #     parameters=[{
    #         'warehouse_config': LaunchConfiguration('warehouse_config'),
    #         'map_frame': 'map',
    #         'nav_loop_rate_ms': 500,
    #         'use_sim_time': True,
    #     }]
    # )
    #
    # delayed_gpal_navigator = TimerAction(
    #     period=12.0,
    #     actions=[gpal_navigator_node]
    # )

    # === 8. RVIZ2 ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
    )

    # === Launch Description ===
    return LaunchDescription([
        # Startup
        startup_banner,

        # Environment
        gz_resource_path,

        # Arguments
        declare_world_arg,
        declare_rviz_config_arg,
        declare_warehouse_config_arg,
        declare_map_file_arg,
        declare_map_yaml_arg,

        # Gazebo + Bridge
        ign_gazebo,
        ros_gz_bridge,

        # Static TF
        static_tf_lidar,

        # map_server and amcl are activated by lifecycle_manager_navigation.
        map_server_node,
        amcl_node,
        delayed_nav2,
        # delayed_gpal_navigator,

        # RViz2
        rviz_node,
    ])
