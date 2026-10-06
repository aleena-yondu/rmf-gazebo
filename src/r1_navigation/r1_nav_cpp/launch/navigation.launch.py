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
    Composed Nav2 launch without collision_monitor and with a custom
    NavigateToPose BT with periodic IsPathValid + wait-then-replan.

    slam_toolbox publishes /map directly; no dedicated nav2_map_server here.
    The costmap StaticLayer subscribes to /map from slam_toolbox's own
    rasterised output (see {livox,ouster}/slam_toolbox_localization.yaml --
    `map_name: /map`), matching the pre-import legacy flow. The `map:=...yaml`
    launch arg is therefore currently unused; it is kept declared so callers
    that pass `map:=${MAP_FILE}.yaml` (e.g. start_nav.sh) do not error.

    Mirrors navigation_bt.launch.py argument surface (use_sim_time, params_file,
    autostart, map) so start_nav.sh / RViz / external callers stay drop-in
    compatible. The default_nav_to_pose_bt_xml param is overridden to point
    at config/behaviour_trees/navigate_to_pose_w_wait_replan.xml so the BT
    halts FollowPath when an obstacle steps onto the planned path, waits 4 s
    for it to clear, then replans -- before escalating to the standard
    backup/spin recovery if the obstacle persists. RViz / nav2_msgs goal
    requests still work normally because the entry point (NavigateToPose)
    is unchanged; only the recovery pattern around FollowPath differs.

    Composition (CPU): every Nav2-side node lives inside a single
    `component_container_isolated` named `nav2_container`. The upstream
    nav2_bringup/navigation_launch.py is included with use_composition:=True
    and container_name:=nav2_container so its own LoadComposableNodes block
    (controller_server, planner_server, smoother_server, behavior_server,
    bt_navigator, waypoint_follower, velocity_smoother, plus
    lifecycle_manager_navigation) targets the same container.
    Net effect: ~10 separate Nav2 processes collapse into 1, every topic hop
    on the hot path (/cmd_vel_nav -> velocity_smoother -> /cmd_vel,
    costmap updates, BT->action plumbing)
    becomes intra-process zero-copy, and DDS discovery / heartbeat thread
    overhead collapses to a single participant. We use
    component_container_isolated rather than the multithreaded variant so
    each component still gets its own SingleThreadedExecutor in its own
    thread (the Nav2-recommended pattern -- avoids one slow callback
    head-of-line blocking the rest).

    Canonical cmd_vel pipeline:
        controller_server  -- /cmd_vel_nav        -->  velocity_smoother
        velocity_smoother  -- /cmd_vel            -->  cmd_vel_relay
        cmd_vel_relay      -- /motion_target/target_speed_chassis --> chassis

    Trade-off: a single component crash takes the whole container down, and
    the upstream `use_respawn` flag only applies to the non-composed code
    path so it has no effect here.
    """
    pkg_r1_nav_cpp = get_package_share_directory('r1_nav_cpp')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')
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
            'Full path to map YAML file. Currently UNUSED: slam_toolbox '
            'publishes /map directly (see {livox,ouster}/'
            'slam_toolbox_localization.yaml -> map_name: /map), so the '
            'costmap StaticLayer feeds from slam_toolbox rather than a '
            'dedicated nav2_map_server. Argument is kept declared so '
            'callers passing map:=<...>.yaml do not error.'
        ),
    )

    # Default custom BT XML: navigate_to_pose.xml.
    # Can be overridden at launch time via nav_to_pose_bt_xml:=... to run a
    # simpler RViz goal tree (e.g. Nav2 default recovery tree) without
    # editing params files.
    default_bt_xml = PathJoinSubstitution([
        get_package_share_directory('r1_nav_cpp'),
        'config',
        'behaviour_trees',
        'navigate_to_pose.xml'
    ])
    declare_nav_to_pose_bt_xml_cmd = DeclareLaunchArgument(
        'nav_to_pose_bt_xml',
        default_value=default_bt_xml,
        description=(
            'Path to the NavigateToPose BT XML used by bt_navigator. '
            'Override with a standard Nav2 tree for simple RViz goal mode.'
        ),
    )

    # Inject use_sim_time into every Nav2 lifecycle node and override
    # default_nav_to_pose_bt_xml so bt_navigator picks up our wait-then-replan
    # BT instead of the upstream Nav2 default. Mirrors the pattern in
    # navigation_bt.launch.py.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
        },
        convert_types=True
    )

    # --- Component container ---
    # Single isolated container that hosts every Nav2-side node in this
    # launch file. `component_container_isolated` gives each loaded
    # ComposableNode its own SingleThreadedExecutor in its own thread, which
    # is the Nav2-recommended pattern: it preserves per-node parallelism
    # while still giving us intra-process zero-copy IPC between siblings
    # (controller -> velocity_smoother, costmap layers, BT -> action plumbing,
    # etc). The container starts immediately; the
    # upstream nav2_bringup LoadComposableNodes inside the included launch
    # will retry against this name until it is up.
    #
    # CRITICAL: configured_params (the full nav2_params.yaml) is loaded on
    # the CONTAINER, not just on individual ComposableNodes. Reason:
    # controller_server / planner_server are composable nodes, but they
    # internally spawn `local_costmap` and `global_costmap` Costmap2DROS
    # SUB-nodes (not separate ComposableNode descriptors). Those sub-nodes
    # read their params from the container process's parameter store. If
    # the YAML is only loaded onto the controller_server ComposableNode and
    # not the container, the local_costmap / global_costmap sub-nodes find
    # nothing under their node name and silently fall back to upstream
    # Nav2 defaults -- no footprint:, default robot_radius=0.1, default
    # plugin list ["static_layer", "obstacle_layer", "inflation_layer"].
    # That fallback then crashes MPPI's ObstaclesCritic
    # (consider_footprint:true vs costmap reporting use_radius:true is
    # inconsistent and throws a PlannerException), which Magick's signal
    # handler (installed by map_server in the same process) converts to
    # abort() and takes the whole container down. Mirror upstream
    # nav2_bringup/bringup_launch.py exactly.
    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        # 'both' so all composed-node std::cout (controller_server's MPPI
        # diagnostics, planner failure traces, BT messages) lands in the
        # per-process log file in addition to the tmux pane.
        output='both',
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # --- Static map publisher ---
    # Intentionally NOT launched here. slam_toolbox owns /map directly via
    # the `map_name: /map` parameter in {livox,ouster}/
    # slam_toolbox_localization.yaml; the costmap StaticLayer subscribes to
    # that. A dedicated nav2_map_server + lifecycle_manager_localization
    # were used in a previous integration; they have been removed to match
    # the older flow and are kept out of nav2_container.
    #
    # If you ever need to reintroduce a dedicated map_server (e.g. for an
    # AMCL/static-map deployment), reload the yaml referenced by the
    # `map:=...` launch arg (declare_map_cmd above) and add a
    # LoadComposableNodes block targeting nav2_container with map_server +
    # lifecycle_manager_localization. Also revert the `map_name: /map`
    # rename in slam_toolbox_localization.yaml back to `/slam_toolbox/map`
    # so the two publishers don't collide on /map.

    # --- Nav2 stack (controller, planner, behaviors, bt_navigator, smoother) ---
    # use_composition:=True flips the upstream launch from spawning N
    # standalone Node processes to issuing a single LoadComposableNodes
    # against `container_name`. We point that at our nav2_container so the
    # entire stack runs in-process. With no collision monitor in this launch,
    # keep the upstream velocity_smoother remap so it publishes directly on
    # /cmd_vel for cmd_vel_relay.
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items()
    )

    # Delay Nav2 long enough for slam_toolbox to deserialise the posegraph
    # and rasterise + publish its first /map. On GPA-class maps that warm-
    # up takes a few hundred ms once the posegraph is loaded; the 2s buffer
    # gives the StaticLayer subscription a latched /map message by the
    # time controller/planner activation triggers its first read.
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
        delayed_nav2_cmd,
    ])
