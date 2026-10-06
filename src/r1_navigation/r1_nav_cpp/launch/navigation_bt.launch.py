import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from r1_nav_cpp.config_resolver import get_map_path, get_nav_config_path


def generate_launch_description():
    """
    Composed Nav2 BT launch without collision monitor.

    This launch is navigation-only (no localization nodes). It assumes a
    separate localization stack publishes /map, /odom, and TF.

    Composition model:
      - Start a single component_container_isolated named nav2_container
      - Include nav2_bringup/navigation_launch.py with use_composition:=True
      - Keep Nav2's normal velocity_smoother remap to /cmd_vel so cmd_vel_relay
        receives commands directly.

    BT model:
      - default NavigateToPose BT XML is bin_tote.xml
      - can be overridden at launch-time with nav_to_pose_bt_xml:=...
    """
    pkg_r1_nav_cpp = get_package_share_directory('r1_nav_cpp')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=get_nav_config_path(pkg_r1_nav_cpp, 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=get_map_path(pkg_r1_nav_cpp, with_extension=True),
        description=(
            'Full path to map YAML file. Currently UNUSED in this launch; '
            'kept for compatibility with callers that pass map:=...yaml.'
        ),
    )

    default_bt_xml = PathJoinSubstitution([
        pkg_r1_nav_cpp,
        'config',
        'behaviour_trees',
        'bin_tote.xml'
    ])

    declare_nav_to_pose_bt_xml_cmd = DeclareLaunchArgument(
        'nav_to_pose_bt_xml',
        default_value=default_bt_xml,
        description='Path to NavigateToPose BT XML used by bt_navigator',
    )

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
        },
        convert_types=True
    )

    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='both',
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items()
    )

    # Delay Nav2 startup briefly to allow localization/map providers to settle.
    delayed_nav2_cmd = TimerAction(
        period=2.0,
        actions=[nav2_navigation_launch]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_map_cmd,
        declare_nav_to_pose_bt_xml_cmd,

        nav2_container,
        delayed_nav2_cmd
    ])
