import os
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_alignment():
    import sys
    script = Path(__file__).resolve().parent.parent / 'src' / 'align_map_to_gazebo.py'
    spec = importlib.util.spec_from_file_location('align_map_to_gazebo', script)
    module = importlib.util.module_from_spec(spec)
    # Register before exec so the dataclass in that file can see its module.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.align()


def _gazebo(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    # -s skips the GUI. --headless-rendering keeps the lidar camera running.
    prefix = '-s -r --headless-rendering ' if headless == 'true' else '-r '
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': prefix + world}.items(),
        )
    ]


def _spawn_and_map_tf(context, *args, **kwargs):
    fit = _load_alignment()
    sim_models_dir = os.path.join(
        get_package_share_directory('r1_sim'), 'models')
    spawn_sim_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_sim_robot',
        output='screen',
        arguments=[
            '-world', 'sim_world',
            '-name', 'sim_robot',
            '-file', os.path.join(sim_models_dir, 'sim_robot', 'model.sdf'),
            '-x', f'{fit.spawn_x:.2f}',
            '-y', f'{fit.spawn_y:.2f}',
            '-z', f'{fit.spawn_z:.2f}',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        parameters=[{'use_sim_time': True}],
    )
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        output='screen',
        arguments=[
            '--x', f'{fit.map_x:.2f}',
            '--y', f'{fit.map_y:.2f}',
            '--z', f'{fit.map_z:.2f}',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', f'{fit.map_yaw:.4f}',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': True}],
    )
    return [
        LogInfo(msg='\n' + fit.report),
        TimerAction(period=3.0, actions=[spawn_sim_robot]),
        static_tf_map_to_odom,
    ]


def generate_launch_description():
    """
    Ground-truth localization on a single REP-105 chain.

    OdometryPublisher writes odom -> base_link from the simulated world pose.
    DiffDrive only turns the wheels. map -> odom is the rigid alignment of
    the Gazebo world onto the saved map. AMCL and slam_toolbox are not
    started. lifecycle_manager_navigation activates map_server.

    Usage:
        ros2 launch r1_sim goal_sim_ground_truth.launch.py
    """

    pkg_r1_sim = get_package_share_directory('r1_sim')
    sim_models_dir = os.path.join(pkg_r1_sim, 'models')
    sim_map_yaml = os.path.join(pkg_r1_sim, 'maps', 'yondu_simmap.yaml')
    nav2_params_file = os.path.join(pkg_r1_sim, 'config', 'nav2_sim_params_gt.yaml')

    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([
            sim_models_dir,
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_r1_sim, 'worlds', 'yondu_warehouse_sim_spawn.sdf'),
        description='Warehouse world without sim_robot. create spawns the robot.'
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
    # yondu_simmap: origin (-21.428, 16.507), 378 x 399 px, 0.05 m/px.
    declare_goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='-8.5',
        description='Goal x in the map frame. Valid map x is -21.43 to -2.53.'
    )
    declare_goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='26.5',
        description='Goal y in the map frame. Valid map y is 16.51 to 36.46.'
    )
    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo Sim server only, with lidar rendering and no GUI.'
    )
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Open RViz. Set false when there is no display.'
    )
    declare_send_test_goal_arg = DeclareLaunchArgument(
        'send_test_goal',
        default_value='true',
        description='Send goal_x/goal_y. Set false when the fleet manager sends NavigateToPose.'
    )

    startup_banner = LogInfo(
        msg='\n' + '=' * 70 + '\n'
        + '  GOAL SIMULATION - ground truth REP-105\n'
        + '  Map x range: -21.43 to -2.53\n'
        + '  Map y range:  16.51 to 36.46\n'
        + '  Pick a free cell inside that box, then:\n'
        + '    goal_x:=-8.5 goal_y:=26.5\n'
        + '  Open-RMF should send NavigateToPose itself. These args are for a test goal.\n'
        + '=' * 70
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

    send_goal = ExecuteProcess(
        cmd=[
            'ros2', 'action', 'send_goal',
            '/navigate_to_pose',
            'nav2_msgs/action/NavigateToPose',
            [
                '{pose: {header: {frame_id: map}, pose: {position: {x: ',
                LaunchConfiguration('goal_x'),
                ', y: ',
                LaunchConfiguration('goal_y'),
                ', z: 0.0}, orientation: {w: 1.0}}}}',
            ],
            '--feedback',
        ],
        output='screen',
    )

    return LaunchDescription([
        startup_banner,
        gz_resource_path,
        declare_world_arg,
        declare_rviz_config_arg,
        declare_map_yaml_arg,
        declare_goal_x_arg,
        declare_goal_y_arg,
        declare_headless_arg,
        declare_use_rviz_arg,
        declare_send_test_goal_arg,
        OpaqueFunction(function=_gazebo),
        OpaqueFunction(function=_spawn_and_map_tf),
        ros_gz_bridge,
        static_tf_lidar,
        map_server_node,
        TimerAction(period=8.0, actions=[nav2_navigation_launch]),
        TimerAction(
            period=25.0,
            actions=[send_goal],
            condition=IfCondition(LaunchConfiguration('send_test_goal')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
