"""Shared launch helper: builds the intra-process sensor pipeline container.

All five composable nodes in the localization-time sensor chain live in ONE
rclcpp_components container with use_intra_process_comms: true. The heavy
edges (Ouster PointCloud2 ~1.5 MB @ 10 Hz, ZED IMU ~720 B @ 200 Hz, LIVOX
scan/cloud) become pointer-passing instead of DDS serialize round-trips
between processes -- the single biggest nav-side CPU win.

Members (lidar-dependent):
  LIVOX :  synchro_drive_odometry, online_imu_bias_estimator,
           pointcloud_deskew, pointcloud_to_laserscan
           (no laser_filter -- LIVOX returns are clean enough without it)
  OUSTER:  synchro_drive_odometry, online_imu_bias_estimator,
           pointcloud_deskew, pointcloud_to_laserscan,
           laser_filters/ScanToScanFilterChain

Both lidars now feed online_imu_bias_estimator from $IMU_SOURCE. The default
source is /livox/imu to match main's LIVOX path; override IMU_SOURCE for
field experiments. The EKF and deskew path consume the de-biased output on
/imu/data_bias_corrected.

Source-switching note: when adding the new CV7/chassis IMU, this helper should
not need topology changes. Set IMU_SOURCE to the new raw IMU topic, adjust
livox/imu_bias_estimator.yaml projection/TF settings, and update
livox/ekf.yaml only if that source provides validated yaw orientation in
addition to yaw rate.

NOT composed (each for a specific reason):
  - ekf_filter_node (robot_localization)
      robot_localization in Humble has NO rclcpp_components plugin. There
      is no upstream EkfNode component. Composing the rest still removes
      the largest serialize hops (cloud, scan, IMU); EKF then receives
      inter-process /imu/data_bias_corrected plus /synchro_odom, which is
      OK (small messages, 30-200 Hz). Reconsider once you upgrade past
      Humble.
  - laser_filter on the LIVOX branch
      The LIVOX path goes pointcloud_to_laserscan -> /scan_raw -> /scan
      directly without a filter chain (matches lifelong.launch.py). Only
      the OUSTER branch needs scan_to_scan_filter_chain composed in.
  - dynamic_footprint, slam_toolbox, cmd_vel_relay
      slam_toolbox is not a rclcpp::Component and intentionally has its own
      thread; cmd_vel_relay and dynamic_footprint are <0.05 core each and
      not on the hot intra-process chain.

This helper is shared by localization.launch.py, mapping.launch.py, and
lifelong.launch.py so the three modes get an identical sensor topology.
"""

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from r1_nav_cpp.config_resolver import (
    get_imu_source,
    get_nav_config_path,
    get_shared_nav_config_path,
)


def build_sensor_container(
    *,
    lidar: str,
    pkg_share: str,
    use_sim_time,
    chassis_feedback_topic: str,
    odom_yaw_scale: float,
    odom_vx_scale: float,
    odom_vy_scale: float,
    laser_filter_config_file,
    livox_p2l_range_max: float = 20.0,
    livox_p2l_min_height: float = 0.05,
    livox_deskew_enabled: bool = True,
    livox_project_imu_to_lidar: bool = False,
    livox_linear_deskew_enabled: bool = True,
    livox_odom_topic: str = '/synchro_odom',
    livox_scan_topic: str = '/scan',
    livox_debug_motion_log_period_s=0.0,
    wheel_velocity_units: str = 'mps',
    steer_offsets_rad=None,
    slip_constraint_weight=None,
    wheel_positions_xy=None,
    adaptive_covariance_overrides=None,
):
    """Construct the ComposableNodeContainer for the sensor pipeline.

    Returns a launch action that, when executed, spawns one process hosting
    all the composable nodes for the given lidar, with intra-process comms.

    Parameters
    ----------
    lidar : str
        'LIVOX' or 'OUSTER'.
    pkg_share : str
        ament share path of r1_nav_cpp (used to locate yaml configs).
    use_sim_time
        LaunchConfiguration or bool to forward to every composed node.
    chassis_feedback_topic : str
        /hdas/feedback_chassis topic (synchro_drive_odom input). Plumbed
        through from the launch files; defaults to the raw publisher topic
        now that timestamp_corrector is no longer in the chain.
    odom_*_scale : float
        Wheel odometry calibration scales (must match between mapping/loc/
        lifelong; see comments at the call sites in each launch file).
    laser_filter_config_file
        Substitution or string path to the laser_filters yaml. Only used on
        OUSTER -- the LIVOX branch does not run a filter chain.
    livox_p2l_range_max : float
        pointcloud_to_laserscan range_max for LIVOX. Mapping, localization,
        and lifelong use 32.0 m (4 m headroom over slam max_laser_range=28).
    livox_p2l_min_height : float
        LIVOX pointcloud_to_laserscan min_height. Current nav modes use 0.1
        so mapping/localization/lifelong see the same scan signature; kept
        as an arg for ekf_only and one-off launch overrides.
    livox_deskew_enabled : bool
        Enable LIVOX pointcloud yaw deskew. Defaults TRUE since 2026-05-20
        after the SE(2) audit (synchro_odom_audit.py + scan_render_offset.py)
        confirmed sub-cm geometric accuracy at peak commanded speed. Set
        false explicitly to fall back to the pre-2026-05-20 baseline.
    livox_project_imu_to_lidar : bool
        Project IMU angular velocity into the lidar frame before deskew.
        Keep false on CV7 until the CV7-to-LIVOX row is recalibrated;
        the CV7 mounts close to base axes so the residual is sub-1%.
    livox_linear_deskew_enabled : bool
        Use /synchro_odom twist + a one-shot lever-arm correction to also
        compensate LIVOX translation across each cloud sweep. Defaults
        TRUE since 2026-05-20 (same audit as livox_deskew_enabled). Set
        false to skip the linear leg if /synchro_odom is unhealthy.
    livox_odom_topic : str
        Odometry topic providing vx/vy for LIVOX linear deskew.
    livox_scan_topic : str
        Output topic for the LIVOX pointcloud_to_laserscan node. Defaults
        to /scan; ekf_only can route to /scan_raw for timestamp restamping.
    livox_debug_motion_log_period_s
        When > 0, pointcloud_deskew prints a per-scan motion summary at
        this cadence so a field operator can sanity-check /synchro_odom
        magnitudes. 0 (default) silences the log.
    wheel_velocity_units : str
        Units for HDAS wheel velocities consumed by synchro_drive_odometry.
        Defaults to 'mps' to preserve deployed behavior; use 'radps' only if
        the source publisher is verified to provide wheel angular velocity.
    steer_offsets_rad, slip_constraint_weight, wheel_positions_xy,
    adaptive_covariance_overrides
        Optional odometry tuning overrides layered on top of
        synchro_odometry_params.yaml. Leave None to use the yaml defaults.
    """
    intra = {'use_intra_process_comms': True}

    odom_overrides = {
        # Production must keep TF ownership in robot_localization. The yaml is
        # also used standalone, where publish_tf=true is useful for direct
        # odom testing, so force the production override here.
        'publish_tf': False,
        'publish_topic': '/synchro_odom',
        'chassis_feedback_topic': chassis_feedback_topic,
        'vx_scale_factor': odom_vx_scale,
        'vy_scale_factor': odom_vy_scale,
        'yaw_scale_factor': odom_yaw_scale,
        'wheel_velocity_units': wheel_velocity_units,
        'use_sim_time': use_sim_time,
    }
    if steer_offsets_rad is not None:
        odom_overrides['steer_offsets_rad'] = steer_offsets_rad
    if slip_constraint_weight is not None:
        odom_overrides['slip_constraint_weight'] = slip_constraint_weight
    if wheel_positions_xy is not None:
        odom_overrides['wheel_positions_xy'] = wheel_positions_xy
    if adaptive_covariance_overrides:
        odom_overrides.update(adaptive_covariance_overrides)

    composable_nodes = [
        ComposableNode(
            package='r1_nav_cpp',
            plugin='r1_nav_cpp::SynchroDriveOdometry',
            name='synchro_drive_odometry',
            parameters=[
                get_shared_nav_config_path(pkg_share, 'synchro_odometry_params.yaml'),
                odom_overrides,
            ],
            extra_arguments=[intra],
        ),
    ]

    # online_imu_bias_estimator runs for BOTH lidars. Both pipelines now
    # consume the bias-corrected IMU (input_topic defaults to /livox/imu via
    # $IMU_SOURCE in config_resolver) and publish on /imu/data_bias_corrected.
    composable_nodes.append(ComposableNode(
        package='r1_nav_cpp',
        plugin='r1_nav_cpp::OnlineImuBiasEstimator',
        name='online_imu_bias_estimator',
        parameters=[
            get_nav_config_path(pkg_share, 'imu_bias_estimator.yaml'),
            {
                'use_sim_time': use_sim_time,
                # Lidar-aware so OUSTER doesn't inherit the Livox IMU topic.
                'input_topic': get_imu_source(lidar),
            },
        ],
        extra_arguments=[intra],
    ))

    if lidar == 'LIVOX':
        # LIVOX deskew: same component as Ouster, auto-detects the FLOAT64
        # 'timestamp' field (livox_ros_driver2 publishes ns-since-scan-start
        # via offset_time -> double cast in lddc.cpp).
        # IMU source is /imu/data_bias_corrected, produced by
        # online_imu_bias_estimator above off the raw Livox IMU by default
        # (configurable via $IMU_SOURCE). The Livox IMU path is already close
        # enough to the lidar/base axes that project_imu_to_lidar stays False.
        # Required because pointcloud_to_laserscan does ONE TF lookup per
        # 100 ms frame and would otherwise project an entire 100 ms sweep
        # as if base_link was frozen at scan start -- visible as walls
        # sliding during rotation.
        composable_nodes.append(ComposableNode(
            package='r1_nav_cpp',
            plugin='r1_nav_cpp::PointCloudDeskew',
            name='pointcloud_deskew',
            parameters=[{
                # 2026-05-12 A/B TEST: deskew disabled (pure passthrough) to
                # isolate the "scan bounces on linear acceleration" symptom
                # reported after `project_imu_to_lidar: True` was enabled
                # 2026-05-11. Mechanism: ZED head IMU on the torso/zed-head
                # chain (compliant mount) sees transient wx/wy from chassis
                # pitch/roll under linear accel; those project into
                # wz_lidar through the calibrated row (pv0=-0.142, pv1=-0.012)
                # and produce spurious yaw deskew → cloud rotates in azimuth
                # during accel events. The calibration row was derived from
                # PURE STEADY yaw rotation, which doesn't excite the torso
                # compliance, so the artifact was invisible at calibration
                # time. With enabled=False the node passes the cloud through
                # unchanged. If the bounce stops with this single change,
                # the next step is: enabled=True, project_imu_to_lidar=False
                # (gives up the 1% steady-rotation correction but kills the
                # accel-artifact path). If the bounce persists with deskew
                # off, look at EKF dynamics under accel instead.
                #
                # 2026-05-13 step 2: tried deskew=True, project_imu_to_lidar
                # =False. Field result: yaw still drifted ("worse and
                # worse"), map->odom corrections from slam_toolbox were
                # jumpy in the same aisle-2 location that was stable at
                # 3735c4b. Reverted to enabled=False below.
                #
                # 2026-05-13 step 3: deskew DISABLED again to exactly
                # match the 3735c4b "very stable in aisle 2" baseline.
                # The loaded map (/home/nvidia/yondu_ws/gpa_04081700_final
                # saved 2026-05-09 from commit d94cf6f) WAS built with
                # deskew=True+project_imu_to_lidar=True, but 3735c4b
                # localization ran undeskewed against it and was stable.
                # Re-enabling deskew on top of the same map produced
                # the jumpy-slam drift symptom -- slam_toolbox apparently
                # prefers the same scan signature it saw during mapping
                # (or its tolerance for the change is small at this
                # quality of map). Visual rotation distortion seen in
                # rviz under this config is the well-known raw-LIVOX
                # intra-scan azimuth smear; addressing that requires
                # either rebuilding the map with a chosen deskew config
                # or a viz-only deskew topic that does NOT feed slam.
                #
                # 2026-05-20 step 4: SE(2) deskew RE-ENABLED with the
                # synchro_odom topology. Current default keeps the raw IMU
                # source aligned with main (/livox/imu -> bias estimator ->
                # /imu/data_bias_corrected) and leaves project_imu_to_lidar
                # false.
                #   - Linear deskew was added (livox_linear_deskew_enabled
                #     above) so translational rolling-shutter is also
                #     compensated, not just rotational. Step 2 only had
                #     the yaw correction, which left a residual that
                #     slam_toolbox saw as a moving scan signature.
                #   - Lever-arm correction baked in so /synchro_odom
                #     base_link twist is re-expressed at livox_frame.
                # Audit (scripts/synchro_odom_audit.py +
                # scripts/scan_render_offset.py, 2026-05-20):
                #   - /synchro_odom calibrated to 1.00 +/- 0.01 on every
                #     axis; stationary bias < 1 mm/s.
                #   - dx_p99 = 1.9 cm at 0.6 m/s commanded (rendered scan
                #     vs truth in odom is sub-cm). dy_p99 = 0.9 cm.
                #     dyaw_p99 = 0.55 deg. dt_p99 = 100 ms (one scan).
                # The map-vs-localization scan signature mismatch from
                # 2026-05-13 is therefore expected to be much smaller
                # with this config; if jumpy slam_toolbox corrections
                # reappear, REBUILD THE MAP with mapping.launch.py
                # (which now also runs SE(2) deskew by default) before
                # disabling deskew here.
                'enabled': livox_deskew_enabled,
                'input_topic': '/livox/lidar',
                'output_topic': '/livox/lidar_deskewed',
                # Output of online_imu_bias_estimator on the configured IMU source
                # (see the long comment at the top of this `if` branch).
                # Matches the OUSTER deskew topic exactly so both lidars
                # now consume the same de-biased rate stream.
                'imu_topic': '/imu/data_bias_corrected',
                'timestamp_field': 'timestamp',
                # Historical note for ZED/head-IMU override experiments:
                # zed_tracking_link_imu is mounted via the torso/zed-head
                # chain and has a non-trivial rotation relative to
                # base_link/livox_frame. Projection was calibrated for that
                # path only. The default /livox/imu source leaves this false.
                #
                # ENABLED 2026-05-11 after calibration validation. With
                # projection on, deskew uses
                #   wz_lidar = imu_lidar_row2 . [wx, wy, wz]_imu
                # which is true base-Z rotation rate. project_imu_to_
                # lidar=False under-corrects yaw distortion by
                # 1 - cos(8.2°) ~= 1% of omega per scan -- small per
                # scan but visibly biased over repeated in-place
                # rotations.
                #
                # Why hardcoded row instead of TF-derived: the URDF's
                # zed_link rpy=(-1.92,0,-1.57) chain implies a 20.1° IMU-z
                # tilt, but the torso joints are NOT at the URDF-nominal
                # zero pose at runtime, so tf2's row gives ~12° more tilt
                # than reality. Re-derive whenever the head/torso pose
                # changes (or the ZED is remounted):
                #   ros2 run r1_nav_cpp calibrate_imu_to_lidar_row.py
                # Calibrated 2026-05-08 at 1.0 rad/s, 15 s steady,
                # |mean|/commanded=0.9994, |row|=1.0000, tilt=8.2°.
                #
                # 2026-05-13: project_imu_to_lidar set False because the
                # upstream bias estimator already publishes pre-projected
                # wz in base_link (see the long comment block above);
                # this row is wrong for a base_link-tagged input. Keep
                # the row recorded here so we can flip back if/when the
                # bias estimator goes back to passing through raw ZED-frame
                # wx, wy, wz (project_angular_velocity_z=false path).
                'project_imu_to_lidar': livox_project_imu_to_lidar,
                'imu_lidar_row2': [-0.1419, -0.0120, 0.9898],
                'linear_deskew_enabled': livox_linear_deskew_enabled,
                'odom_topic': livox_odom_topic,
                'debug_motion_log_period_s': livox_debug_motion_log_period_s,
                'use_sim_time': use_sim_time,
            }],
            extra_arguments=[intra],
        ))
        # LIVOX: pointcloud_to_laserscan normally publishes directly on
        # /scan (no filter chain). Debug launches can route this to
        # /scan_raw and republish /scan with adjusted timestamps.
        composable_nodes.append(ComposableNode(
            package='pointcloud_to_laserscan',
            plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.5,
                'min_height': livox_p2l_min_height,
                'max_height': 1.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.008,
                'scan_time': 0.1,
                'range_min': 0.35,
                'range_max': livox_p2l_range_max,
                'inf_epsilon': 1.0,
                'use_inf': True,
                'use_cloud_timestamp': False,
                'use_sim_time': use_sim_time,
                'concurrency_level': 1,
                'queue_size': 10,
                'qos_overrides./scan.reliability': 'reliable',
            }],
            remappings=[
                # Subscribe to the deskewed cloud, not the raw driver topic.
                ('cloud_in', '/livox/lidar_deskewed'),
                ('scan', livox_scan_topic),
            ],
            extra_arguments=[intra],
        ))
    else:  # OUSTER
        composable_nodes.append(ComposableNode(
            package='r1_nav_cpp',
            plugin='r1_nav_cpp::PointCloudDeskew',
            name='pointcloud_deskew',
            parameters=[{
                'enabled': True,
                'input_topic': '/ouster/points',
                'output_topic': '/ouster/points_deskewed',
                'imu_topic': '/imu/data_bias_corrected',
                'use_sim_time': use_sim_time,
            }],
            extra_arguments=[intra],
        ))
        composable_nodes.append(ComposableNode(
            package='pointcloud_to_laserscan',
            plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.5,
                'min_height': 0.20,
                'max_height': 2.0,
                'angle_min': -3.14159,
                'angle_max':  3.14159,
                'angle_increment': 0.008,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 20.0,
                'use_inf': True,
                'concurrency_level': 1,
                'queue_size': 10,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cloud_in', '/ouster/points_deskewed'),
                ('scan', '/scan_raw'),
            ],
            extra_arguments=[intra],
        ))
        # NOTE: laser_filters registers its components without a namespace
        # in laser_filters_plugins.xml -- the plugin name is the bare
        # "ScanToScanFilterChain". Verify with:
        #   ros2 component types laser_filters
        composable_nodes.append(ComposableNode(
            package='laser_filters',
            plugin='ScanToScanFilterChain',
            name='laser_filter',
            parameters=[
                laser_filter_config_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
            extra_arguments=[intra],
        ))

    return ComposableNodeContainer(
        name='sensor_processing_container',
        namespace='',
        package='rclcpp_components',
        # Use the *isolated* container, NOT component_container_mt.
        #
        # - component_container    : one SingleThreadedExecutor for ALL nodes.
        #     Cheap, but the 10 Hz Ouster cloud callback (multi-ms) would
        #     head-of-line-block the 200 Hz IMU + 100 Hz chassis callbacks.
        # - component_container_mt : ONE MultiThreadedExecutor whose default
        #     thread count is std::thread::hardware_concurrency() = 12 cores
        #     on AGX Orin. With only 5 lightweight callback-based nodes,
        #     11 of those 12 threads sit in rcl_wait() polling the wait set
        #     and show up as a permanent ~5-15 % background CPU slug in top.
        #     Visible enough to be mistaken for the workload itself.
        # - component_container_isolated : one SingleThreadedExecutor per
        #     composed node, each in its own thread. No thread pool, no busy-
        #     spin pool, no head-of-line blocking between the IMU / chassis /
        #     cloud streams. Intra-process zero-copy still works because that
        #     is a process-wide ring buffer, NOT executor-bound. This is the
        #     correct default for a small static set of composed nodes on
        #     Jetson-class hardware.
        executable='component_container_isolated',
        composable_node_descriptions=composable_nodes,
        # 'both' so synchro_drive_odometry / pointcloud_to_laserscan stdout
        # (and online_imu_bias_estimator on OUSTER, plus any deskew /
        # laser_filter chatter) is captured to the per-process log file
        # as well as the console -- previously 'screen'-only meant the
        # wheel-feedback gap warnings only existed in tmux scrollback.
        output='both',
    )
