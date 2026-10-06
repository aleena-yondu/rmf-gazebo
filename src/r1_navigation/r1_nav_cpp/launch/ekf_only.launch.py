"""Sensor pipeline + EKF only -- a SLAM-free, Nav2-free test stack.

Brings up exactly the nodes you need to visually validate the EKF tuning
against the live laser scan in rviz, and NO MORE:

    Sensor container (intra-process):
        synchro_drive_odometry  -> /synchro_odom (50 Hz)
        pointcloud_deskew       -> /livox/lidar_deskewed (LIVOX) or
                                   /ouster/points_deskewed (OUSTER)
        pointcloud_to_laserscan -> /scan
        laser_filter            -> /scan (OUSTER only)
        online_imu_bias_estimator -> /imu/data_bias_corrected (OUSTER only)

    ekf_filter_node             -> /odom (30 Hz) + odom->base_link TF

This is the same sensor topology the production localization launch uses,
just without slam_toolbox, costmaps, controllers, planners, or the Nav2
behavior tree. Use it to A/B EKF parameter changes while watching /scan
in rviz against /odom -- if the scan walls slide during in-place
rotation, the EKF is the suspect, not the SLAM layer.

PREREQUISITES (none of these are started by this launch file):
  - CV7 driver publishing /imu/data at 200 Hz (start_base.sh / the
    cv7_imu tmux window inside r1_container).
  - LIVOX or OUSTER driver publishing the source point cloud.
  - /eva/robot_state_publisher publishing the URDF TF tree.
  - /hdas/feedback_chassis live (wheel encoder stream).

IMPORTANT -- run this INSIDE r1_container, not on the host.
The sensor pipeline composes pointcloud_to_laserscan (and, on OUSTER,
laser_filters); both plugins live only inside the container, not in
/opt/ros/humble on the host. Launching from the host fails with
"Could not find requested resource in ament index". Inside the
container they load cleanly.

USAGE (inside r1_container):
    docker exec -it r1_container bash -lc '
      export ROS_DOMAIN_ID=47
      export ROS_NAMESPACE=eva
      export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/yondu_ws/src/r1_startup/launch_scripts/fastrtps_profiles.xml
      source /opt/ros/humble/setup.bash
      source /home/nvidia/yondu_ws/install/setup.bash
      LIDAR=LIVOX ros2 launch r1_nav_cpp ekf_only.launch.py
    '

Or to background it inside the container:
    docker exec -d r1_container bash -lc '... ros2 launch r1_nav_cpp ekf_only.launch.py > /tmp/ekf_only.log 2>&1'

ARGS (all optional):
    ekf_config_file       -- override the EKF yaml (default: per-LIDAR
                             config/nav_configs/<lidar>/ekf.yaml)
    laser_filter_config_file
                          -- override the laser_filters yaml (only used
                             on OUSTER; LIVOX ignores it)
    use_sim_time          -- forward to every node (default: false)
    vx_scale_factor, vy_scale_factor, yaw_scale_factor
                          -- wheel odom calibration scales (default 1.0,
                             matching the production launches)
    livox_p2l_min_height  -- LIVOX pointcloud_to_laserscan min_height
                             (default 0.05; production loc uses 0.1)
    livox_p2l_range_max   -- LIVOX p2l range_max (default 30.0; produc-
                             tion loc uses 32.0)
    enable_scan_restamp   -- route p2l through /scan_raw and republish /scan
                             stamped at now() (default FALSE; debug-only
                             band-aid that shifts scan vs robot in RViz
                             without changing geometry. Field-validated
                             2026-05-20 via scan_render_offset.py:
                             dx_p99 = 1.9 cm at 0.6 m/s with this off.)
    livox_linear_deskew   -- enable wheel-odom-driven translational deskew
                             of the LIVOX cloud (default TRUE; locked in
                             2026-05-20 after the synchro_odom_audit and
                             scan_render_offset.py runs confirmed
                             /synchro_odom is calibrated and the resulting
                             rendered offset is sub-centimeter at peak
                             commanded speed)
    debug_motion_log_period_s
                          -- when > 0, pointcloud_deskew prints a per-scan
                             motion summary at this cadence (default 0)

WHAT TO LOOK AT AFTER LAUNCH:
    ros2 topic hz /imu/data           # 200 Hz
    ros2 topic hz /synchro_odom       # 50 Hz
    ros2 topic hz /scan               # 10 Hz (cloud rate)
    ros2 topic hz /odom               # 30 Hz
    ros2 node info /ekf_filter_node   # subs /imu/data + /synchro_odom
    ros2 run rviz2 rviz2              # add: /scan, /odom, TF (frame=odom)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from r1_nav_cpp.config_resolver import (
    get_lidar,
    get_nav_config_path,
    get_shared_nav_config_path,
)
from r1_nav_cpp.sensor_pipeline import build_sensor_container


def generate_launch_description():
    pkg_share = get_package_share_directory('r1_nav_cpp')
    lidar = get_lidar()

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Forward sim time to every node',
    )
    declare_ekf_config = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=get_nav_config_path(pkg_share, 'ekf.yaml'),
        description='Path to robot_localization EKF yaml',
    )
    declare_laser_filter_config = DeclareLaunchArgument(
        'laser_filter_config_file',
        default_value=get_shared_nav_config_path(pkg_share, 'laser_filter_config.yaml'),
        description='Path to laser_filters yaml (OUSTER only; LIVOX ignores)',
    )
    declare_vx_scale = DeclareLaunchArgument(
        'vx_scale_factor', default_value='1.0',
        description='Wheel-odom forward velocity calibration',
    )
    declare_vy_scale = DeclareLaunchArgument(
        'vy_scale_factor', default_value='1.0',
        description='Wheel-odom lateral velocity calibration',
    )
    declare_yaw_scale = DeclareLaunchArgument(
        'yaw_scale_factor', default_value='1.0',
        description='Wheel-odom yaw calibration',
    )
    declare_livox_min_h = DeclareLaunchArgument(
        'livox_p2l_min_height', default_value='0.05',
        description='LIVOX pointcloud_to_laserscan min_height',
    )
    declare_livox_range_max = DeclareLaunchArgument(
        'livox_p2l_range_max', default_value='30.0',
        description='LIVOX pointcloud_to_laserscan range_max',
    )
    declare_enable_scan_restamp = DeclareLaunchArgument(
        'enable_scan_restamp', default_value='false',
        description='Debug-only: republish /scan_raw to /scan stamped at '
                    'now(). Off by default because SE(2) deskew already '
                    'pins points to lidar(T_e); restamping just shifts '
                    'the scan vs robot model in RViz by vx*dt without '
                    'changing the underlying geometry. Field-validated '
                    '2026-05-20: dx_p99 of 1.9 cm at 0.6 m/s with this off.',
    )
    declare_linear_deskew = DeclareLaunchArgument(
        'livox_linear_deskew', default_value='true',
        description='Enable wheel-odom-driven translational LIVOX deskew. '
                    'On by default (locked in 2026-05-20 after the '
                    'scan_render_offset audit). Set false to fall back to '
                    'yaw-only deskew if /synchro_odom is unhealthy.',
    )
    declare_debug_motion = DeclareLaunchArgument(
        'debug_motion_log_period_s', default_value='0.0',
        description='When > 0, pointcloud_deskew logs a per-scan motion '
                    'summary at this cadence so operators can sanity-check '
                    '/synchro_odom magnitudes vs commanded motion.',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_scan_restamp = LaunchConfiguration('enable_scan_restamp')
    livox_linear_deskew = LaunchConfiguration('livox_linear_deskew')
    # Case-insensitive routing: "true"/"True"/"1"/"yes"/"on" all enable the
    # restamp path, anything else routes p2l directly to /scan.
    livox_scan_topic = PythonExpression([
        "'/scan_raw' if '", enable_scan_restamp,
        "'.strip().lower() in ('true', '1', 'yes', 'on') else '/scan'",
    ])

    log_lidar = LogInfo(msg=[f'[ekf_only] LIDAR={lidar}; ekf yaml = ',
                             LaunchConfiguration('ekf_config_file')])

    # Same chassis feedback wiring as localization.launch.py -- subscribe
    # directly to the raw HDAS topic; timestamp_corrector is gone.
    chassis_feedback_topic = '/hdas/feedback_chassis'

    # Forward the odom scale launch args into the composed odometry node so
    # ekf_only can be used for calibration sweeps without editing this file.
    sensor_container = build_sensor_container(
        lidar=lidar,
        pkg_share=pkg_share,
        use_sim_time=use_sim_time,
        chassis_feedback_topic=chassis_feedback_topic,
        odom_yaw_scale=LaunchConfiguration('yaw_scale_factor'),
        odom_vx_scale=LaunchConfiguration('vx_scale_factor'),
        odom_vy_scale=LaunchConfiguration('vy_scale_factor'),
        laser_filter_config_file=LaunchConfiguration('laser_filter_config_file'),
        livox_p2l_min_height=LaunchConfiguration('livox_p2l_min_height'),
        livox_p2l_range_max=LaunchConfiguration('livox_p2l_range_max'),
        # Test the observed scan lag with CV7 yaw-rate deskew, but keep
        # projection off until the CV7-to-LIVOX row is recalibrated.
        livox_deskew_enabled=True,
        livox_project_imu_to_lidar=False,
        # Correct residual forward/lateral rolling-shutter distortion in the
        # 100 ms LIVOX sweep using the low-latency composed wheel odometry.
        # Off by default until /synchro_odom signs are validated against the
        # field-test maneuver list (`livox_linear_deskew:=true` to flip on).
        # ROS 2 parameter type coercion converts the LaunchConfiguration
        # "true"/"false" string into the bool the deskew node declares.
        livox_linear_deskew_enabled=livox_linear_deskew,
        livox_odom_topic='/synchro_odom',
        livox_debug_motion_log_period_s=LaunchConfiguration('debug_motion_log_period_s'),
        # Debug restamp is opt-in. Default /scan keeps physical scan timing.
        livox_scan_topic=livox_scan_topic,
    )

    scan_timestamp_fixer = Node(
        package='r1_nav_cpp',
        executable='scan_timestamp_fixer',
        name='scan_timestamp_fixer',
        output='both',
        condition=IfCondition(enable_scan_restamp),
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': '/scan',
            'use_sim_time': use_sim_time,
        }],
    )

    # robot_localization::ekf_node is NOT a rclcpp::Component in Humble,
    # so it stays as a standalone Node outside the intra-process container.
    # /imu/data + /synchro_odom are both small messages; cross-process is fine.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='both',
        parameters=[
            LaunchConfiguration('ekf_config_file'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('odometry/filtered', '/odom')],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_ekf_config,
        declare_laser_filter_config,
        declare_vx_scale,
        declare_vy_scale,
        declare_yaw_scale,
        declare_livox_min_h,
        declare_livox_range_max,
        declare_enable_scan_restamp,
        declare_linear_deskew,
        declare_debug_motion,
        log_lidar,
        sensor_container,
        scan_timestamp_fixer,
        ekf_node,
    ])
