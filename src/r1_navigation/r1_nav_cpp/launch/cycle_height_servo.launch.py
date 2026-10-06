import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Minimal launch for the cycle-height-servo behavior tree.

    Publishes static map->odom and odom->base_link transforms, then starts
    only the bt_navigator (+ lifecycle manager).  No planner, controller,
    behavior, or smoother servers are launched.

    Usage:
        ros2 launch r1_nav_cpp cycle_height_servo.launch.py

    Triggering the BT:
        ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}"
    """

    pkg_r1_nav_cpp = get_package_share_directory('r1_nav_cpp')

    # ── Launch arguments ─────────────────────────────────────────────────

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # ── BT XML path ──────────────────────────────────────────────────────

    bt_xml = PathJoinSubstitution([
        pkg_r1_nav_cpp, 'config', 'behaviour_trees', 'cycle_height_servo.xml'
    ])

    # ── Static TF: map -> odom  (x=1.0, y=1.0, yaw=0.0) ────────────────

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=[
            '--x', '1.0',
            '--y', '1.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    # ── Static TF: odom -> base_link  (x=0.0, y=0.0, yaw=0.0) ──────────

    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link',
        ],
    )

    # ── bt_navigator only ────────────────────────────────────────────────

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'odom_topic': '/odom',
            'bt_loop_duration': 50,
            'default_server_timeout': 60,
            'transform_tolerance': 2.0,
            'enable_groot_monitoring': False,
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': bt_xml,
            'plugin_lib_names': [
                'r1_nav_cpp_bt_nodes',
            ],
        }],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_cycle_height',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['bt_navigator'],
            'bond_timeout': 0.0,
        }],
    )

    # ── Assemble ─────────────────────────────────────────────────────────

    return LaunchDescription([
        declare_use_sim_time,

        static_tf_map_odom,
        static_tf_odom_base,
        bt_navigator_node,
        lifecycle_manager_node,
    ])
