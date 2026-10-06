"""Publish AprilTag map poses as RViz markers for visual verification.

Brings up a single node, ``publish_apriltag_markers.py``, which parses the
same ``warehouse.yaml`` the ``apriltag_pose_corrector`` loads (selected
by the LOCATION/LIDAR env vars) and republishes every resolved tag pose as a
latched ``visualization_msgs/MarkerArray`` on ``/apriltag_map_markers``.

Overlay those markers on the static map (the costmap StaticLayer / ``/map``)
in RViz to catch a mis-placed tag at a glance: one sitting off an aisle, at
the wrong height, or with its facing arrow pointing into the shelf instead of
out toward the robot. The node also logs a warning for every duplicate id,
missing shelf-B height, or skipped end tag it finds while parsing.

USAGE (env vars select the warehouse.yaml, exactly like the relayer):
    LOCATION=gpa LIDAR=livox ros2 launch r1_nav_cpp apriltag_markers.launch.py

Add a MarkerArray display on /apriltag_map_markers (Fixed Frame = map) in your
existing RViz, or start one with this launch:
    ros2 launch r1_nav_cpp apriltag_markers.launch.py launch_rviz:=true

ARGS (all optional):
    warehouse_yaml    Explicit path to warehouse.yaml. Empty (default) ->
                      resolve from LOCATION/LIDAR in the node, matching the
                      relayer's own resolution.
    map_frame         Frame the markers are published in (default "map").
    marker_size       Tag plate edge length in meters (default 0.20).
    normal_length     Facing-arrow length in meters (default 0.50).
    text_z_offset     Height of the id label above the tag (default 0.25).
    publish_period_s  Re-publish period in seconds (default 2.0).
    rotate_about_origin_deg  DEBUG ONLY: rotate every marker pose about the map
                      origin by this angle (default 0.0 = faithful overlay).
    translate_x_m     DEBUG ONLY: translate markers in x after the rotation
                      (default 0.0).
    translate_y_m     DEBUG ONLY: translate markers in y after the rotation
                      (default 0.0).
    use_sim_time      Forward sim time to the node (default false).
    launch_rviz       Also start RViz with r1_navigation.rviz (default false).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from r1_nav_cpp.config_resolver import get_lidar, get_location


def generate_launch_description():
    pkg_share = get_package_share_directory('r1_nav_cpp')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'r1_navigation.rviz')

    declare_warehouse_yaml = DeclareLaunchArgument(
        'warehouse_yaml',
        default_value='',
        description='Explicit warehouse.yaml path. Empty -> resolve from '
                    'LOCATION/LIDAR env vars in the node (matches the relayer).',
    )
    declare_map_frame = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Frame the markers are published in.',
    )
    declare_marker_size = DeclareLaunchArgument(
        'marker_size',
        default_value='0.20',
        description='Tag plate edge length in meters.',
    )
    declare_normal_length = DeclareLaunchArgument(
        'normal_length',
        default_value='0.50',
        description='Facing-arrow length in meters (along the tag +Z normal).',
    )
    declare_text_z_offset = DeclareLaunchArgument(
        'text_z_offset',
        default_value='0.25',
        description='Height of the id text label above the tag.',
    )
    declare_publish_period = DeclareLaunchArgument(
        'publish_period_s',
        default_value='2.0',
        description='Re-publish period (markers are also latched).',
    )
    declare_rotate_about_origin = DeclareLaunchArgument(
        'rotate_about_origin_deg',
        default_value='0.0',
        description='DEBUG/VERIFICATION ONLY: rotate every published marker '
                    'pose about the map origin by this angle (deg). Default '
                    '0.0 = faithful overlay. Use 90.0 to test the "tags appear '
                    'rotated -90 deg from true" report. Does not affect the '
                    'relayer/correction path.',
    )
    declare_translate_x = DeclareLaunchArgument(
        'translate_x_m',
        default_value='0.0',
        description='DEBUG/VERIFICATION ONLY: translate every published marker '
                    'pose in x (meters) after the rotation about the map '
                    'origin. Default 0.0. Does not affect the relayer path.',
    )
    declare_translate_y = DeclareLaunchArgument(
        'translate_y_m',
        default_value='0.0',
        description='DEBUG/VERIFICATION ONLY: translate every published marker '
                    'pose in y (meters) after the rotation about the map '
                    'origin. Default 0.0. Does not affect the relayer path.',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Forward sim time to the node.',
    )
    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Also start RViz with the r1_navigation config.',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    log_source = LogInfo(
        msg=[f'[apriltag_markers] LOCATION={get_location()} LIDAR={get_lidar()}; '
             'warehouse_yaml override = ', LaunchConfiguration('warehouse_yaml')],
    )

    marker_node = Node(
        package='r1_nav_cpp',
        executable='publish_apriltag_markers.py',
        name='apriltag_marker_publisher',
        output='both',
        parameters=[{
            'warehouse_yaml': LaunchConfiguration('warehouse_yaml'),
            'map_frame': LaunchConfiguration('map_frame'),
            'marker_size': LaunchConfiguration('marker_size'),
            'normal_length': LaunchConfiguration('normal_length'),
            'text_z_offset': LaunchConfiguration('text_z_offset'),
            'publish_period_s': LaunchConfiguration('publish_period_s'),
            'rotate_about_origin_deg': ParameterValue(
                LaunchConfiguration('rotate_about_origin_deg'), value_type=float),
            'translate_x_m': ParameterValue(
                LaunchConfiguration('translate_x_m'), value_type=float),
            'translate_y_m': ParameterValue(
                LaunchConfiguration('translate_y_m'), value_type=float),
            'use_sim_time': use_sim_time,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_warehouse_yaml,
        declare_map_frame,
        declare_marker_size,
        declare_normal_length,
        declare_text_z_offset,
        declare_publish_period,
        declare_rotate_about_origin,
        declare_translate_x,
        declare_translate_y,
        declare_use_sim_time,
        declare_launch_rviz,
        log_source,
        marker_node,
        rviz_node,
    ])
