import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from r1_nav_cpp.config_resolver import (
    get_lidar,
    get_location,
    get_map_name,
    get_map_path,
    get_nav_config_path,
    get_shared_nav_config_path,
)
from r1_nav_cpp.sensor_pipeline import build_sensor_container


def generate_launch_description():
    """
    Lifelong mapping pipeline: localizes in an existing map while continuously
    adding new scan nodes and pruning outdated ones. Useful for environments
    that change over time (warehouse pallet rearrangements, etc.).

    NOTE: Requires slam_toolbox built from source -- the lifelong_slam_toolbox_node
    executable is not included in the apt binary package.
    """
    pkg_share = get_package_share_directory('r1_nav_cpp')
    lidar = get_lidar()

    # Prefer the src/ maps directory so we pick up auto-saves without needing a rebuild.
    # Falls back to install/ share directory if src/ can't be resolved.
    _install_share = pkg_share  # e.g. ~/yondu_ws/install/r1_nav_cpp/share/r1_nav_cpp
    _src_maps = os.path.join(
        _install_share.split('/install/')[0], 'src', 'r1_navigation', 'r1_nav_cpp', 'maps'
    ) if '/install/' in _install_share else None
    _maps_root = _src_maps if (_src_maps and os.path.isdir(_src_maps)) else os.path.join(pkg_share, 'maps')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    )

    declare_slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=get_nav_config_path(pkg_share, 'slam_toolbox_lifelong.yaml'),
        description='Path to SLAM Toolbox lifelong parameters'
    )

    # Default map_file always resolves to the env-configured slug under
    # _maps_root/<location>/<lidar>/<MAP_NAME>. Whether to actually load it
    # (vs start fresh) is decided at execution time inside _build_slam_node so
    # CLI overrides like `map_file:=/abs/path/other_map` and `map_file:=`
    # (empty == fresh) are honored.
    _loc = get_location().lower()
    _lid = lidar.lower()
    _map_name = get_map_name()
    default_map_file_path = os.path.join(_maps_root, _loc, _lid, _map_name)

    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file_path,
        description=(
            'Path to the serialized pose graph to load (without extension). '
            'Defaults to maps/<LOCATION>/<LIDAR>/<MAP_NAME>. '
            'Pass map_file:= (empty) to start a fresh map.'
        ),
    )

    declare_ekf_config_arg = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=get_nav_config_path(pkg_share, 'ekf.yaml'),
        description='Path to EKF configuration file'
    )

    declare_laser_filter_config_arg = DeclareLaunchArgument(
        'laser_filter_config_file',
        default_value=get_shared_nav_config_path(pkg_share, 'laser_filter_config.yaml'),
        description='Path to laser filter configuration file'
    )

    # Lifelong is ALWAYS a resume run (loads existing posegraph). Same
    # failure mode mapping.launch.py was patched to prevent applies here:
    # without a startup delay, the first scan after deserialize anchors at
    # a still-drifting EKF pose; if loop closure fires against that bad
    # anchor, the loaded graph rips. We observed this on a GPA_99p resume
    # via mapping.launch.py before the fix, and the lifelong path has the
    # exact same code path through deserialize_map. Default 15.0 s
    # matches mapping.launch.py's recommended resume value -- typically
    # enough on this hardware for the IMU bias estimator to converge with
    # the robot stationary. Override at the CLI when you need the robot
    # to start moving sooner.
    declare_slam_start_delay_arg = DeclareLaunchArgument(
        'slam_start_delay',
        default_value='15.0',
        description=(
            'Seconds to delay slam_toolbox startup so the EKF and IMU bias '
            'estimator can converge before scans are ingested. Lifelong '
            'always loads an existing graph, so this is REQUIRED to avoid '
            'the "first scan after deserialize anchors at a drifted EKF '
            'pose -> wrong closure -> map ripped" failure. Set to 0.0 ONLY '
            'if you know the EKF is already converged at lifelong startup.'
        )
    )

    # Wheel-yaw scale: leave at the localization-time value (1.015 for
    # LIVOX) by default so a long-running lifelong session matches
    # whatever wheel calibration the rest of the stack uses. For
    # one-shot "extend the existing map" runs against a graph that was
    # built with `mapping.launch.py` (yaw_scale=1.0), pass
    # `match_mapping_calibration:=true` so new graph nodes are placed
    # under the same yaw integration as the loaded nodes -- prevents a
    # subtle yaw seam between the original mapping segments and the
    # lifelong-appended segments.
    declare_match_mapping_arg = DeclareLaunchArgument(
        'match_mapping_calibration',
        default_value='false',
        description=(
            'If true, use the same wheel scales as mapping.launch.py '
            '(1.0/1.0/1.0) so lifelong-appended graph nodes integrate '
            'yaw under the same calibration as the loaded graph. Use '
            'this for one-shot map-extension runs against a graph built '
            'by mapping.launch.py. Leave false for production lifelong-'
            'during-deployment.'
        )
    )

    print(f"[lifelong] Using LIDAR={lidar} LOCATION={_loc.upper()} MAP_NAME={_map_name}")
    print(f"[lifelong] Default map_file: {default_map_file_path} (override with map_file:=...)")

    # Subscribe directly to the raw HDAS chassis feedback topic. The previous
    # `<ros_namespace>/hdas/feedback_chassis` indirection was the corrected
    # republish from `timestamp_corrector`, which is no longer launched -- the
    # HDAS clock is system-aligned within ~1 ms (validated 2026-05-04). See
    # the matching comment in localization.launch.py for the full rationale.
    chassis_feedback_topic = "/hdas/feedback_chassis"

    cmd_vel_relay_node = Node(
        package='r1_nav_cpp',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='both',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # robot_localization::ekf_node is NOT a rclcpp::Component in Humble, so
    # it cannot join the intra-process container. Stays as its own process.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='both',
        parameters=[
            LaunchConfiguration('ekf_config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('odometry/filtered', '/odom')]
    )

    def _build_runtime_actions(context, *args, **kwargs):
        """Resolve every execution-time arg in one place and return the
        sensor container + delayed slam node together.

        Why one OpaqueFunction for both:
          * `match_mapping_calibration` decides odom_yaw_scale, which is a
            python float baked into the synchro_drive_odometry component
            parameters at sensor-container construction time -- it cannot
            be a LaunchConfiguration substitution because
            `build_sensor_container` takes plain floats. So the container
            must be built AFTER the launch arg is resolved, which means
            inside an OpaqueFunction.
          * `slam_start_delay` likewise needs to be a python float for
            TimerAction's `period` to parse correctly across all
            launch_ros versions on Humble.
          * `map_file` is resolved here too so a CLI override
            `map_file:=...` takes effect.
        """
        map_file = LaunchConfiguration('map_file').perform(context).strip()
        slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
        use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()
        use_sim_time = use_sim_time_str == 'true'
        slam_start_delay = float(
            LaunchConfiguration('slam_start_delay').perform(context)
        )
        match_mapping = (
            LaunchConfiguration('match_mapping_calibration').perform(context).lower()
            == 'true'
        )

        # Wheel-yaw calibration. See declare_match_mapping_arg above.
        if match_mapping:
            odom_vx_scale = 1.0
            odom_vy_scale = 1.0
            odom_yaw_scale = 1.0
            print("[lifelong] match_mapping_calibration=true -> "
                  "wheel scales (1.0, 1.0, 1.0). Use this for one-shot "
                  "extension of a map built by mapping.launch.py.")
        else:
            if lidar == 'LIVOX':
                # 0.96 -> 1.0: chassis suspected to under-report vx; 0.96 was actively
                # shrinking readings and contributed to along-aisle compression.
                # Keep default wheel scales aligned with mapping/localization.
                # Record any future yaw calibration as an explicit override,
                # not as a hidden lifelong-only default.
                odom_vx_scale = 1.0
                odom_vy_scale = 1.0
                odom_yaw_scale = 1.0
            else:
                odom_vx_scale = 1.0
                odom_vy_scale = 1.0
                odom_yaw_scale = 1.0
            print(f"[lifelong] match_mapping_calibration=false -> "
                  f"wheel scales ({odom_vx_scale}, {odom_vy_scale}, "
                  f"{odom_yaw_scale}). Use match_mapping_calibration:=true "
                  f"if extending a map built by mapping.launch.py.")

        # Sensor pipeline (synchro_odom + imu_bias + p2l + (deskew/laser_filter
        # on OUSTER)) all in one intra-process container. See
        # r1_nav_cpp/sensor_pipeline.py for the rationale and full topology.
        sensor_container = build_sensor_container(
            lidar=lidar,
            pkg_share=pkg_share,
            use_sim_time=use_sim_time,
            chassis_feedback_topic=chassis_feedback_topic,
            odom_yaw_scale=odom_yaw_scale,
            odom_vx_scale=odom_vx_scale,
            odom_vy_scale=odom_vy_scale,
            laser_filter_config_file=LaunchConfiguration('laser_filter_config_file'),
            # lifelong matches mapping LIVOX (min_height=0.1, range_max=32 with
            # 4 m headroom over slam max_laser_range=28).
            livox_p2l_min_height=0.1,
            livox_p2l_range_max=32.0,
        )

        params = [
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': '/scan',
            },
        ]
        if map_file and os.path.isfile(map_file + '.posegraph'):
            params.append({'map_file_name': map_file})
            print(f"[lifelong] Loading existing map: {map_file}")
        elif map_file:
            print(f"[lifelong] No posegraph at '{map_file}.posegraph' "
                  f"-- starting fresh")
        else:
            print("[lifelong] map_file is empty -- starting fresh")

        slam_node = Node(
            package='slam_toolbox',
            executable='lifelong_slam_toolbox_node',
            name='slam_toolbox',
            output='both',
            parameters=params,
        )

        # Wrap slam_toolbox in a TimerAction so the EKF + IMU bias
        # estimator have converged before the matcher starts ingesting
        # scans. Lifelong always loads an existing graph, so without
        # this delay the first scan after deserialize anchors at a
        # still-drifting EKF pose -- if loop closure fires against that
        # bad anchor, the loaded graph rips. We observed this on a
        # GPA_99p resume via mapping.launch.py before the equivalent
        # fix was added there. Default 15.0 s; override at the CLI if
        # the EKF is already known-converged.
        delayed_slam_node = TimerAction(
            period=slam_start_delay,
            actions=[slam_node],
        )
        if slam_start_delay > 0.0:
            print(f"[lifelong] slam_toolbox delayed {slam_start_delay:.1f}s "
                  f"to let EKF + IMU bias estimator converge")

        return [sensor_container, delayed_slam_node]

    lifelong_runtime_action = OpaqueFunction(function=_build_runtime_actions)

    # AprilTag pose corrector. Same node the localization launches use: it
    # computes the robot's map-frame pose from tag detections and publishes it
    # on /pose_correction. slam_toolbox owns the map->odom TF directly
    # (transform_publish_period > 0 in the lifelong YAMLs). Tags sit on fixed
    # rack infrastructure, so they remain valid anchors while the graph mutates.
    # If no tag poses load (missing apriltag_poses.yaml/warehouse.yaml), the
    # corrector simply publishes nothing, so this node is safe to run here.
    apriltag_pose_corrector_node = Node(
        package='r1_nav_cpp',
        executable='apriltag_pose_corrector',
        name='apriltag_pose_corrector',
        output='both',
        parameters=[
            get_shared_nav_config_path(pkg_share, 'apriltag_pose_corrector.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # AprilTag marker visualizer. Publishes the known tag map poses as a latched
    # MarkerArray on /apriltag_map_markers for RViz overlay. Runs alongside the
    # pose corrector.
    apriltag_marker_node = Node(
        package='r1_nav_cpp',
        executable='publish_apriltag_markers.py',
        name='apriltag_marker_publisher',
        output='both',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    dynamic_footprint_node = Node(
        package='r1_nav_cpp',
        executable='dynamic_footprint',
        name='dynamic_footprint',
        output='both',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_slam_params_arg,
        declare_map_file_arg,
        declare_ekf_config_arg,
        declare_laser_filter_config_arg,
        declare_slam_start_delay_arg,
        declare_match_mapping_arg,

        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, context: print("[Lifelong] Shutting down cleanly...")
            )
        ),

        cmd_vel_relay_node,

        # ekf_node stays separate (robot_localization is not a Component
        # in Humble). slam_toolbox is wrapped inside the OpaqueFunction
        # along with the sensor container so both pick up runtime args
        # like match_mapping_calibration and slam_start_delay.
        ekf_node,
        lifelong_runtime_action,
        apriltag_pose_corrector_node,
        apriltag_marker_node,
        # dynamic_footprint_node intentionally disabled (was already
        # commented out before this refactor).
    ])
