import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from r1_nav_cpp.config_resolver import (
    get_lidar,
    get_location,
    get_map_name,
    get_nav_config_path,
    get_shared_nav_config_path,
)
from r1_nav_cpp.sensor_pipeline import build_sensor_container


def generate_launch_description():
    """
    Unified SLAM mapping pipeline for both Livox and Ouster LiDARs.
    Reads $LIDAR env var to select sensor pipeline, EKF config, and IMU setup.
    """
    pkg_share = get_package_share_directory('r1_nav_cpp')
    lidar = get_lidar()

    # Resolve the maps tree the same way lifelong.launch.py does so a CLI
    # `map_file:=` override picks up auto-saves from src/, not just installed
    # share. See lifelong.launch.py for the full rationale.
    _install_share = pkg_share
    _src_maps = os.path.join(
        _install_share.split('/install/')[0], 'src', 'r1_navigation',
        'r1_nav_cpp', 'maps'
    ) if '/install/' in _install_share else None
    _maps_root = _src_maps if (_src_maps and os.path.isdir(_src_maps)) \
        else os.path.join(pkg_share, 'maps')
    _loc = get_location().lower()
    _lid = lidar.lower()
    _map_name = get_map_name()
    default_map_file_path = os.path.join(_maps_root, _loc, _lid, _map_name)

    # Both pipelines fuse the bias-corrected IMU into the EKF for mapping.
    # On LIVOX, the bias estimator defaults to /livox/imu to match main.
    default_ekf = get_nav_config_path(pkg_share, 'ekf.yaml')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=get_nav_config_path(pkg_share, 'slam_toolbox_mapping.yaml'),
        description='Path to SLAM Toolbox parameters for mapping mode'
    )
    declare_ekf_config_arg = DeclareLaunchArgument(
        'ekf_config_file', default_value=default_ekf,
        description='Path to EKF configuration file'
    )
    declare_laser_filter_config_arg = DeclareLaunchArgument(
        'laser_filter_config_file',
        default_value=get_shared_nav_config_path(pkg_share, 'laser_filter_config.yaml'),
        description='Path to laser filter configuration file'
    )
    # slam_toolbox logger level. Karto's per-candidate loop-closure prints
    # (FindPossibleLoopClosure / TryCloseLoop / FireLoopClosureCheck) are ROS
    # DEBUG macros, so they only surface when this logger is at debug. DEBUG is
    # noisy and adds noticeable I/O cost on large tours, so default to info and
    # pass slam_log_level:=debug only while tuning loop-closure gates.
    declare_slam_log_level_arg = DeclareLaunchArgument(
        'slam_log_level',
        default_value='info',
        description=(
            'ROS log level for the slam_toolbox logger. Use debug to surface '
            'per-candidate loop-closure prints while tuning; info otherwise.'
        )
    )
    # Seconds to hold off slam_toolbox after the sensor pipeline + EKF come up.
    # Sensor pipeline, online_imu_bias_estimator, and ekf_filter_node start
    # immediately and warm up during this window. Critical for resume runs
    # (deserialize_map): without it the first new scan accepted after
    # deserialize anchors at a still-drifting EKF pose and corrupts the
    # loaded graph (we observed this on a GPA_99p resume -- a wrong-aisle
    # closure fired against the bad anchor and ripped the map). 15.0 s is
    # usually enough on this hardware for the IMU bias estimator to
    # converge with the robot stationary. Default 0.0 preserves the
    # fresh-mapping bringup timing. Pass slam_start_delay:=15.0 (or
    # whatever your bias estimator needs) when resuming.
    declare_slam_start_delay_arg = DeclareLaunchArgument(
        'slam_start_delay',
        default_value='0.0',
        description=(
            'Seconds to delay slam_toolbox startup so the EKF and IMU bias '
            'estimator can converge before scans are ingested. Use ~15.0 '
            'for resume (deserialize_map) runs; 0.0 for fresh maps.'
        )
    )

    # Optional resume / map-extension entry point. Leave EMPTY for a fresh
    # mapping run (the historical default behavior). Set to an existing
    # map basename WITHOUT extension (e.g. `.../GPA_zui_zhong`) to load
    # that posegraph at startup and APPEND to it; slam_toolbox will keep
    # extending the loaded graph as the robot drives. Pair with
    # `slam_start_delay:=15.0` so the EKF + IMU bias estimator converge
    # before the first new scan anchors against the loaded graph.
    #
    # The map_file_name parameter is injected only when a sibling
    # `<map_file>.posegraph` actually exists -- typo-safe so a stale CLI
    # override doesn't silently produce a fresh map.
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description=(
            'Path (without extension) to an existing posegraph to RESUME '
            'mapping from. Empty (default) starts a fresh map. Set to '
            'e.g. maps/<location>/<lidar>/GPA_zui_zhong to extend the '
            'GPA map with a missed aisle. ALWAYS pair with '
            'slam_start_delay:=15.0 on resume runs to avoid the '
            '"first scan after deserialize anchors at a drifted EKF '
            'pose -> wrong closure -> map ripped" failure.'
        ),
    )

    print(f"[mapping] Default resume map_file (when set): "
          f"{default_map_file_path}")

    use_sim_time = LaunchConfiguration('use_sim_time')

    print(f"[mapping] Using LIDAR={lidar}")

    # Subscribe directly to the raw HDAS chassis feedback topic. The previous
    # `<ros_namespace>/hdas/feedback_chassis` indirection was the corrected
    # republish from `timestamp_corrector`, which is no longer launched -- the
    # HDAS clock is system-aligned within ~1 ms (validated 2026-05-04). See
    # the matching comment in localization.launch.py for the full rationale.
    chassis_feedback_topic = "/hdas/feedback_chassis"

    # mapping intentionally uses (vx, vy, yaw) = (1.0, 1.0, 1.0) on BOTH
    # lidars. Keep this aligned with localization/lifelong unless a field
    # calibration is recorded as an explicit override.
    sensor_container = build_sensor_container(
        lidar=lidar,
        pkg_share=pkg_share,
        use_sim_time=use_sim_time,
        chassis_feedback_topic=chassis_feedback_topic,
        odom_yaw_scale=1.0,
        odom_vx_scale=1.0,
        odom_vy_scale=1.0,
        laser_filter_config_file=LaunchConfiguration('laser_filter_config_file'),
        # mapping LIVOX uses min_height=0.1 and range_max=32.0 (4 m headroom
        # over slam max_laser_range=28.0). Matches lifelong.launch.py.
        livox_p2l_min_height=0.1,
        livox_p2l_range_max=32.0,
    )

    cmd_vel_relay_node = Node(
        package='r1_nav_cpp',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # robot_localization::ekf_node is NOT a rclcpp::Component in Humble, so it
    # cannot join the intra-process container. Stays as its own process.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[LaunchConfiguration('ekf_config_file'), {'use_sim_time': use_sim_time}],
        output='both',
        remappings=[('odometry/filtered', '/odom')]
    )

    # `debug_logging: true` in slam_toolbox_mapping.yaml only flips Karto's
    # internal verbose flag -- the per-candidate loop-closure prints
    # (FindPossibleLoopClosure / TryCloseLoop / FireLoopClosureCheck etc.)
    # are emitted via ROS DEBUG-level macros, so they only reach the
    # terminal if the slam_toolbox logger is also at DEBUG. That logger level
    # is now controlled by the slam_log_level launch arg (default info); pass
    # slam_log_level:=debug while tuning the GPA warehouse parallel-aisle
    # loop-closure gates, and leave it at info for normal mapping runs since
    # DEBUG is noisy and adds noticeable I/O cost on large tours.
    def _build_slam_node(context, *args, **kwargs):
        """Resolve map_file at execution time so a CLI `map_file:=` override
        wins over the (empty) default. If a sibling `.posegraph` exists,
        inject map_file_name so slam_toolbox deserializes and appends; if
        not, start fresh (matches historical mapping.launch.py behavior).
        """
        map_file = LaunchConfiguration('map_file').perform(context).strip()
        params = [
            LaunchConfiguration('slam_params_file').perform(context),
            {'use_sim_time': LaunchConfiguration('use_sim_time')
                .perform(context).lower() == 'true',
             'scan_topic': '/scan'},
        ]
        if map_file and os.path.isfile(map_file + '.posegraph'):
            params.append({'map_file_name': map_file})
            print(f"[mapping] RESUME mode: loading existing posegraph "
                  f"{map_file} and appending new nodes")
            print("[mapping] -- ensure slam_start_delay >= 15.0 so the "
                  "EKF + IMU bias estimator have converged before the "
                  "first new scan is anchored against the loaded graph.")
        elif map_file:
            print(f"[mapping] map_file='{map_file}' set but no "
                  f"'{map_file}.posegraph' on disk -- starting FRESH "
                  f"to avoid silently producing a fresh map under a typo")
        # else: empty map_file -> historical fresh-mapping behavior

        slam_log_level = LaunchConfiguration('slam_log_level') \
            .perform(context).strip()
        slam_toolbox_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='both',
            parameters=params,
            ros_arguments=['--log-level', f'slam_toolbox:={slam_log_level}'],
        )
        # Wrap slam_toolbox in a TimerAction so the EKF + IMU bias
        # estimator have converged before the matcher starts ingesting
        # scans. See declare_slam_start_delay_arg above for the full
        # rationale; tl;dr this prevents the "first scan after
        # deserialize anchors at a drifted EKF pose -> wrong closure ->
        # map ripped" failure mode on resume runs. period=0.0 is a
        # no-op for fresh-mapping bringup.
        return [TimerAction(
            period=float(LaunchConfiguration('slam_start_delay')
                         .perform(context)),
            actions=[slam_toolbox_node],
        )]

    delayed_slam_toolbox_node = OpaqueFunction(function=_build_slam_node)

    # dynamic_footprint is intentionally NOT launched during mapping (it was
    # already disabled before the refactor); nav modes own the footprint.

    # OUSTER mapping also wants a /map throttle; it doesn't belong in the
    # sensor pipeline, so add it separately.
    extra_nodes = []
    if lidar == 'OUSTER':
        extra_nodes.append(Node(
            package='topic_tools',
            executable='throttle',
            name='map_throttle',
            arguments=['messages', '/map', '1', '/map_viz'],
            output='both',
        ))

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_slam_params_arg,
        declare_ekf_config_arg,
        declare_laser_filter_config_arg,
        declare_slam_log_level_arg,
        declare_slam_start_delay_arg,
        declare_map_file_arg,

        # One process for the whole sensor chain (intra-process zero-copy):
        # synchro_drive_odometry, online_imu_bias_estimator,
        # pointcloud_to_laserscan, and (OUSTER only) pointcloud_deskew +
        # laser_filters/ScanToScanFilterChain.
        sensor_container,

        cmd_vel_relay_node,

        *extra_nodes,

        # ekf_node and slam_toolbox stay separate processes: robot_localization
        # is not a Component in Humble; slam_toolbox intentionally has its own
        # thread.
        ekf_node,
        # slam_toolbox is wrapped in a TimerAction (see slam_start_delay
        # arg above). With the default of 0.0 s this is functionally
        # identical to launching the node directly.
        delayed_slam_toolbox_node,
    ])
