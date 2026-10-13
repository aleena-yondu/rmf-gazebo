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


def _robot_description(model_sdf):
    script = Path(__file__).resolve().parent.parent / 'src' / 'sdf_to_urdf.py'
    spec = importlib.util.spec_from_file_location('sdf_to_urdf', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.convert(model_sdf)


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


def generate_launch_description():
    """
    Ground-truth localization on a single REP-105 chain.

    OdometryPublisher writes odom -> base_link from the simulated world pose,
    so odom is the Gazebo world frame. yondu_simmap was built by slam_toolbox
    on that same odometry, so map -> odom is identity and map coordinates are
    Gazebo coordinates. AMCL and slam_toolbox are not started.
    lifecycle_manager_navigation activates map_server.

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
            pkg_r1_sim, 'worlds', 'robotnik_warehouse_r1.sdf'),
        description='Warehouse world. robot_1 is included in the world file.'
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
    # yondu_simmap: origin (-9.635, -9.732), 397 x 391 px, 0.05 m/px.
    declare_goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='-6.0',
        description='Goal x in the map frame. Valid map x is -9.63 to 10.21.'
    )
    declare_goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='0.0',
        description='Goal y in the map frame. Valid map y is -9.73 to 9.81.'
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
        + '  Map frame = Gazebo world frame.\n'
        + '  Map x range: -9.63 to 10.21\n'
        + '  Map y range: -9.73 to  9.81\n'
        + '  Pick a free cell inside that box, then:\n'
        + '    goal_x:=-6.0 goal_y:=0.0\n'
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
            '/model/robot_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/robot_1/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
        ],
        remappings=[
            ('/model/robot_1/odometry', '/odom'),
            ('/model/robot_1/tf', '/tf'),
        ],
    )

    # Publishes /robot_description for RViz and the fixed base_link -> os_lidar chain.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': _robot_description(
                os.path.join(pkg_r1_sim, 'models', 'robot_1', 'model.sdf')),
        }],
    )

    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        output='screen',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
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
        static_tf_map_to_odom,
        ros_gz_bridge,
        robot_state_publisher,
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
