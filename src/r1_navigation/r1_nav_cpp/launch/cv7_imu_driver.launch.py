from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """
    Launch the MicroStrain 3DM-CV7-AR IMU driver for the LIVOX pipeline.

    Thin wrapper around microstrain_inertial_driver/microstrain_launch.py
    that injects our LIVOX-side params file
    (config/nav_configs/livox/cv7_imu_driver.yaml) and otherwise lets the
    upstream lifecycle / configure / activate flow run unchanged.

    Publishes:
        /imu/data           — filtered orientation + accel + gyro
                              (frame_id = cv7_imu_link)
        /imu/data_raw       — raw IMU
        /ekf/status         — on-device EKF state (CV7-AR only -- no
                              GNSS/mag aiding; see params file)

    Why we do NOT use microstrain_inertial_examples/cv7_launch.py:
        It hardcodes the CV7-INS example yml (port /dev/microstrain_main,
        magnetometer-based heading alignment, wheeled-vehicle constraint
        on) and silently ignores any `params_file:=` argument because the
        wrapper does not declare it. Symptom: driver tries to open
        /dev/microstrain_main, fails, exits. Even if the port matched,
        the CV7-AR firmware rejects the INS-style aiding settings.
        See agent_context/Cv7_agent_context.md §"Smoke test session" #3.

    Prereqs (handled automatically by launch_docker.sh in r1_container):
        - --group-add=dialout on the docker run, so the container user
          can open /dev/ttyACM0 without sudo
        - ros-humble-microstrain-inertial-driver installed (apt-installed
          on first launch by section 8 of launch_docker.sh)

    Usage:
        ros2 launch r1_nav_cpp cv7_imu_driver.launch.py
        ros2 launch r1_nav_cpp cv7_imu_driver.launch.py debug:=true
        ros2 launch r1_nav_cpp cv7_imu_driver.launch.py namespace:=/cv7
        ros2 launch r1_nav_cpp cv7_imu_driver.launch.py \\
            params_file:=/abs/path/to/override.yaml
    """

    pkg_r1_nav_cpp = get_package_share_directory('r1_nav_cpp')
    pkg_driver = get_package_share_directory('microstrain_inertial_driver')

    # ── Launch arguments ────────────────────────────────────────────────

    namespace = LaunchConfiguration('namespace')
    node_name = LaunchConfiguration('node_name')
    params_file = LaunchConfiguration('params_file')
    debug = LaunchConfiguration('debug')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='/',
        description='ROS namespace under which the driver node is launched.')

    declare_node_name = DeclareLaunchArgument(
        'node_name', default_value='microstrain_inertial_driver',
        description='Name for the driver node (kept at upstream default '
                    'so existing service/topic remaps from MicroStrain '
                    'docs still apply).')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            pkg_r1_nav_cpp, 'config', 'nav_configs', 'livox', 'cv7_imu_driver.yaml',
        ]),
        description='Driver params file. Override to point at a tuning '
                    'fork (e.g. for OUSTER pipeline or bench tests) '
                    'rather than editing the LIVOX yaml in place.')

    declare_debug = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Enable verbose driver logging (passed through to '
                    'microstrain_launch.py).')

    # ── Include the upstream driver launch ──────────────────────────────
    #
    # microstrain_launch.py from the *driver* package (NOT the examples
    # package) honors `params_file:=`. It also takes care of lifecycle
    # configure + activate so the node is publishing by the time this
    # LaunchDescription finishes setup.

    inner_launch = PathJoinSubstitution([
        pkg_driver, 'launch', 'microstrain_launch.py',
    ])

    microstrain_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(inner_launch),
        launch_arguments={
            'namespace': namespace,
            'node_name': node_name,
            'params_file': params_file,
            'debug': debug,
        }.items(),
    )

    # ── Assemble ────────────────────────────────────────────────────────

    return LaunchDescription([
        declare_namespace,
        declare_node_name,
        declare_params_file,
        declare_debug,

        microstrain_driver,
    ])
