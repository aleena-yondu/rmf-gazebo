cmake_minimum_required(VERSION 3.10)
project(r1_nav_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(r1_interfaces REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Dependencies list for easier reuse
set(DEPENDENCIES
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  r1_interfaces
  std_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

# ===== Composable Components Library =====
#
# synchro_drive_odometry, online_imu_bias_estimator, and pointcloud_deskew
# are compiled into a single SHARED library so they can be loaded into a
# rclcpp_components ComposableNodeContainer with use_intra_process_comms.
# The downstream chain
#   chassis_feedback -> synchro_odom -> ekf
#   /<imu_in> -> imu_bias_estimator -> /imu/data_bias_corrected -> deskew + ekf
#   /ouster/points -> deskew -> pointcloud_to_laserscan -> laser_filter -> slam
# all becomes pointer-passing instead of DDS serialize round-trips.
#
# rclcpp_components_register_node() ALSO auto-generates a thin standalone
# executable per node, so existing scripts that do
#   `ros2 run r1_nav_cpp online_imu_bias_estimator`
# keep working with no changes.
add_library(r1_nav_cpp_components SHARED
  src/synchro_drive_odometry.cpp
  src/online_imu_bias_estimator.cpp
  src/pointcloud_deskew.cpp
)
ament_target_dependencies(r1_nav_cpp_components
  ${DEPENDENCIES}
  rclcpp_components
)
rclcpp_components_register_node(r1_nav_cpp_components
  PLUGIN "r1_nav_cpp::SynchroDriveOdometry"
  EXECUTABLE synchro_drive_odometry
)
rclcpp_components_register_node(r1_nav_cpp_components
  PLUGIN "r1_nav_cpp::OnlineImuBiasEstimator"
  EXECUTABLE online_imu_bias_estimator
)
rclcpp_components_register_node(r1_nav_cpp_components
  PLUGIN "r1_nav_cpp::PointCloudDeskew"
  EXECUTABLE pointcloud_deskew
)

install(TARGETS r1_nav_cpp_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ===== Robot Footprint/Workspace Visualizer Node =====
add_executable(dynamic_footprint
  src/dynamic_footprint.cpp
)
ament_target_dependencies(dynamic_footprint ${DEPENDENCIES})

# ===== CMD_VEL RELAY (Twist -> TwistStamped) =====
add_executable(cmd_vel_relay
  src/cmd_vel_relay.cpp
)
ament_target_dependencies(cmd_vel_relay ${DEPENDENCIES})

# ===== CHASSIS TWIST RELAY (TwistStamped -> TwistWithCovarianceStamped) =====
add_executable(chassis_twist_relay
  src/chassis_twist_relay.cpp
)
ament_target_dependencies(chassis_twist_relay ${DEPENDENCIES})

# ===== LIVOX IMU CONVERTER (accel g -> m/s^2, covariances) =====
add_executable(livox_imu_converter
  src/livox_imu_converter.cpp
)
ament_target_dependencies(livox_imu_converter ${DEPENDENCIES})

# ===== SCAN TIMESTAMP FIXER =====
add_executable(scan_timestamp_fixer
  src/scan_timestamp_fixer.cpp
)
ament_target_dependencies(scan_timestamp_fixer ${DEPENDENCIES})

# ===== POINTCLOUD TIMESTAMP FIXER =====
add_executable(pointcloud_timestamp_fixer
  src/pointcloud_timestamp_fixer.cpp
)
ament_target_dependencies(pointcloud_timestamp_fixer ${DEPENDENCIES})

# ===== APRILTAG POSE CORRECTOR =====
add_executable(apriltag_pose_corrector
  src/apriltag_pose_corrector.cpp
)
ament_target_dependencies(apriltag_pose_corrector
  ${DEPENDENCIES}
  ament_index_cpp
  yaml_cpp_vendor
  std_srvs
)
target_link_libraries(apriltag_pose_corrector
  yaml-cpp
)

# ===== BT NODES =====
add_library(r1_nav_cpp_bt_nodes SHARED
  src/bt_nodes/update_order_state.cpp
  src/bt_nodes/servo_to_target_action.cpp
  src/bt_nodes/get_chassis_target_service.cpp
  src/bt_nodes/set_blackboard_string.cpp
  src/bt_nodes/debug_print.cpp
  src/bt_nodes/bt_nodes_plugin.cpp
  src/bt_nodes/run_inference_bt_node.cpp
  src/bt_nodes/publish_pose.cpp
  src/bt_nodes/notify_operator_ready.cpp
  src/bt_nodes/set_tolerance.cpp
  src/bt_nodes/pan_scanner.cpp
  src/bt_nodes/scan_for_barcode_action.cpp
  src/bt_nodes/compute_spin_angle.cpp
  src/bt_nodes/solve_ik_bt_node.cpp
  src/bt_nodes/cycle_height_servo_action.cpp
  src/bt_nodes/highlight_target_bt_node.cpp
  src/bt_nodes/search_bins_bt_node.cpp
  src/bt_nodes/follow_right_arm_trajectory.cpp
  src/bt_nodes/select_inference_action_type.cpp
)

ament_target_dependencies(r1_nav_cpp_bt_nodes
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  behaviortree_cpp
  r1_interfaces
  sensor_msgs
  std_msgs
  ament_index_cpp
  yaml_cpp_vendor
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(r1_nav_cpp_bt_nodes
  yaml-cpp
  behaviortree_cpp::behaviortree_cpp
)

target_compile_definitions(r1_nav_cpp_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS r1_nav_cpp_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install non-composable executables. The composable trio
# (synchro_drive_odometry, online_imu_bias_estimator, pointcloud_deskew)
# are installed by rclcpp_components_register_node above.
install(TARGETS
  dynamic_footprint
  cmd_vel_relay
  chassis_twist_relay
  livox_imu_converter
  scan_timestamp_fixer
  pointcloud_timestamp_fixer
  apriltag_pose_corrector
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install maps directory
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts.
# CONFIGURE_DEPENDS re-globs at build time so newly added scripts (e.g.
# publish_apriltag_markers.py) get installed on a normal `colcon build`
# without needing a clean/reconfigure. Without it, a plain GLOB is only
# evaluated at configure time and new files silently fail to install
# (symptom: "executable '<name>.py' not found on the libexec directory").
file(GLOB_RECURSE PYTHON_SCRIPTS CONFIGURE_DEPENDS
  scripts/*.py
  scripts/*.sh
)
install(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_synchro_drive_kinematics
    test/test_synchro_drive_kinematics.cpp
  )
  if(TARGET test_synchro_drive_kinematics)
    target_include_directories(test_synchro_drive_kinematics PRIVATE
      include
      ${EIGEN3_INCLUDE_DIRS}
    )
  endif()
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
