#include "r1_nav_cpp/bt_nodes/solve_ik_bt_node.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace r1_bt_nodes
{

SolveIkBtNode::SolveIkBtNode(const std::string & name,
                               const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard) {
    throw BT::RuntimeError("SolveIkBtNode: blackboard is null.");
  }

  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "SolveIkBtNode: 'node' not found on blackboard. "
      "This BT node must be used inside Nav2 bt_navigator.");
  }

  client_ = rclcpp_action::create_client<SolveIK>(node_, "/solve_ik");

  auto transient_local_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                  .reliable()
                                  .transient_local();

  order_sub_ = node_->create_subscription<r1_interfaces::msg::PickTicket>(
    "/teleop/order",
    transient_local_qos,
    std::bind(&SolveIkBtNode::orderCallback, this, std::placeholders::_1));

  loadConfig();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  loadWarehouseConfig();
}

void SolveIkBtNode::orderCallback(
  const r1_interfaces::msg::PickTicket::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (msg->item_name == "NONE") {
    current_order_.reset();
  } else {
    current_order_ = msg;
  }
}

void SolveIkBtNode::loadConfig()
{
  if (config_loaded_) {
    return;
  }

  std::string pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("r1_nav_cpp");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "SolveIkBtNode: failed to get package share directory: %s", e.what());
    throw;
  }

  const char * loc_env = std::getenv("LOCATION");
  std::string location = (loc_env && loc_env[0] != '\0') ? loc_env : "yondu";
  std::transform(location.begin(), location.end(), location.begin(), ::tolower);

  const char * lidar_env = std::getenv("LIDAR");
  std::string lidar = (lidar_env && lidar_env[0] != '\0') ? lidar_env : "livox";
  std::transform(lidar.begin(), lidar.end(), lidar.begin(), ::tolower);

  std::string config_path =
    pkg_share + "/config/warehouse_configs/" + location + "/" + lidar + "/ik.yaml";

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const std::exception &) {
    RCLCPP_WARN(node_->get_logger(),
      "SolveIkBtNode: '%s' not found, falling back to yondu/livox/ik.yaml",
      config_path.c_str());
    config_path = pkg_share + "/config/warehouse_configs/yondu/livox/ik.yaml";
    try {
      root = YAML::LoadFile(config_path);
    } catch (const std::exception & e2) {
      RCLCPP_ERROR(node_->get_logger(),
        "SolveIkBtNode: failed to load fallback '%s': %s",
        config_path.c_str(), e2.what());
      throw;
    }
  }

  RCLCPP_INFO(node_->get_logger(),
    "SolveIkBtNode: loaded IK config from '%s'", config_path.c_str());

  YAML::Node ik_config = root["solve_ik_poses"];
  if (!ik_config) {
    RCLCPP_ERROR(node_->get_logger(),
      "SolveIkBtNode: 'solve_ik_poses' key not found in '%s'", config_path.c_str());
    throw std::runtime_error("Missing 'solve_ik_poses' key in config");
  }

  const std::string default_target_link =
    ik_config["target_link"].as<std::string>("left_gripper_link");

  gripper_standoff_ = ik_config["gripper_standoff"].as<double>(0.30);

  YAML::Node shelves = ik_config["shelves"];
  if (!shelves || !shelves.IsMap()) {
    RCLCPP_WARN(node_->get_logger(),
      "SolveIkBtNode: no 'shelves' found in config");
    config_loaded_ = true;
    return;
  }

  for (auto it = shelves.begin(); it != shelves.end(); ++it) {
    const std::string shelf_key = it->first.as<std::string>();
    YAML::Node shelf_node = it->second;

    ShelfIkConfig shelf_cfg;
    shelf_cfg.target_link =
      shelf_node["target_link"].as<std::string>(default_target_link);

    geometry_msgs::msg::Quaternion orientation;
    YAML::Node orientation_node = shelf_node["orientation"];
    if (orientation_node) {
      orientation.x = orientation_node["x"].as<double>(0.0);
      orientation.y = orientation_node["y"].as<double>(0.0);
      orientation.z = orientation_node["z"].as<double>(0.0);
      orientation.w = orientation_node["w"].as<double>(1.0);
    }

    YAML::Node positions = shelf_node["positions"];
    if (positions && positions.IsSequence()) {
      for (size_t i = 0; i < positions.size(); ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = positions[i]["x"].as<double>(0.0);
        pose.position.y = positions[i]["y"].as<double>(0.0);
        pose.position.z = positions[i]["z"].as<double>(0.0);
        pose.orientation = orientation;
        shelf_cfg.poses.push_back(pose);
      }
    }

    shelf_configs_[shelf_key] = shelf_cfg;
    RCLCPP_INFO(node_->get_logger(),
      "SolveIkBtNode: loaded %zu poses for shelf '%s' (link: %s)",
      shelf_cfg.poses.size(), shelf_key.c_str(), shelf_cfg.target_link.c_str());
  }

  config_loaded_ = true;
}

void SolveIkBtNode::loadWarehouseConfig()
{
  if (warehouse_loaded_) {
    return;
  }

  std::string pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("r1_nav_cpp");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "SolveIkBtNode: failed to get package share directory: %s", e.what());
    return;
  }

  const char * loc_env = std::getenv("LOCATION");
  std::string location = (loc_env && loc_env[0] != '\0') ? loc_env : "yondu";
  std::transform(location.begin(), location.end(), location.begin(), ::tolower);

  const char * lidar_env = std::getenv("LIDAR");
  std::string lidar = (lidar_env && lidar_env[0] != '\0') ? lidar_env : "livox";
  std::transform(lidar.begin(), lidar.end(), lidar.begin(), ::tolower);

  const std::string config_path =
    pkg_share + "/config/warehouse_configs/" + location + "/" + lidar + "/warehouse.yaml";

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "SolveIkBtNode: failed to load warehouse config '%s': %s",
      config_path.c_str(), e.what());
    return;
  }

  YAML::Node rows = root["warehouse"]["rows"];
  if (!rows || !rows.IsMap()) {
    RCLCPP_WARN(node_->get_logger(),
      "SolveIkBtNode: no 'warehouse.rows' in '%s'", config_path.c_str());
    return;
  }

  for (auto row_it = rows.begin(); row_it != rows.end(); ++row_it) {
    const std::string row_key = row_it->first.as<std::string>();
    YAML::Node row_node = row_it->second;

    RowData row_data;

    YAML::Node segments = row_node["segments"];
    if (!segments || !segments.IsSequence()) {
      continue;
    }

    // Walk segments -> bays -> shelves; bin positions are interpolated along
    // each segment between bay.start_distance and bay.end_distance using the
    // same centered formula as HierarchicalWarehouse::interpolateBinPositions:
    //   bin n (1..N) at fractional position (n - 0.5) / N.
    for (size_t s = 0; s < segments.size(); ++s) {
      YAML::Node seg_node = segments[s];
      const double seg_yaw_rad =
        seg_node["yaw_deg"].as<double>(0.0) * M_PI / 180.0;
      const double sx = seg_node["start_point"]["x"].as<double>(0.0);
      const double sy = seg_node["start_point"]["y"].as<double>(0.0);
      const double ex = seg_node["end_point"]["x"].as<double>(0.0);
      const double ey = seg_node["end_point"]["y"].as<double>(0.0);
      const double dx = ex - sx;
      const double dy = ey - sy;
      const double seg_len = std::sqrt(dx * dx + dy * dy);
      if (seg_len < 1e-6) {
        continue;
      }
      const double ux = dx / seg_len;
      const double uy = dy / seg_len;

      YAML::Node bays = seg_node["bays"];
      if (!bays || !bays.IsMap()) {
        continue;
      }

      for (auto bay_it = bays.begin(); bay_it != bays.end(); ++bay_it) {
        const std::string bay_key = bay_it->first.as<std::string>();
        YAML::Node bay_node = bay_it->second;

        BayData bay_data;
        bay_data.yaw_rad = seg_yaw_rad;

        const double start_d = bay_node["start_distance"].as<double>(0.0);
        const double end_d = bay_node["end_distance"].as<double>(0.0);
        const double span = end_d - start_d;

        YAML::Node shelves = bay_node["shelves"];
        if (shelves && shelves.IsMap()) {
          for (auto shelf_it = shelves.begin(); shelf_it != shelves.end(); ++shelf_it) {
            const std::string shelf_key = shelf_it->first.as<std::string>();
            YAML::Node shelf_node = shelf_it->second;
            const int num_bins = shelf_node["num_bins"].as<int>(0);
            if (num_bins <= 0) {
              continue;
            }
            for (int n = 1; n <= num_bins; ++n) {
              const double t =
                (static_cast<double>(n) - 0.5) / static_cast<double>(num_bins);
              const double d = start_d + t * span;
              BinMapPosition pos;
              pos.x = sx + d * ux;
              pos.y = sy + d * uy;
              bay_data.bins[shelf_key][n] = pos;
            }
          }
        }

        row_data.bays[bay_key] = bay_data;
      }
    }

    warehouse_rows_[row_key] = row_data;
  }

  warehouse_loaded_ = true;
  RCLCPP_INFO(node_->get_logger(),
    "SolveIkBtNode: loaded warehouse config '%s' (%zu rows, standoff=%.2fm)",
    location.c_str(), warehouse_rows_.size(), gripper_standoff_);
}

std::vector<geometry_msgs::msg::Pose> SolveIkBtNode::computeAdjustedPoses(
  const ShelfIkConfig & cfg,
  const r1_interfaces::msg::PickTicket & order)
{
  // Build lookup keys from order
  const std::string row_key =
    order.bin.side + std::to_string(order.bin.row);

  char bay_buf[4];
  std::snprintf(bay_buf, sizeof(bay_buf), "%02d",
    static_cast<int>(order.bin.bay));
  const std::string bay_key(bay_buf);
  const std::string shelf_key = order.bin.shelf;
  const int bin_num = static_cast<int>(order.bin.bin_number);

  // Look up row geometry
  auto row_it = warehouse_rows_.find(row_key);
  if (row_it == warehouse_rows_.end()) {
    RCLCPP_WARN(node_->get_logger(),
      "SolveIkBtNode: row '%s' not in warehouse config, using static poses",
      row_key.c_str());
    return cfg.poses;
  }

  const RowData & row = row_it->second;

  // Find a reference bin position on the row face. Yaw is taken from the
  // bay's owning segment (stored on BayData), since a row may now span
  // multiple segments at slightly different facing angles.
  BinMapPosition ref_bin{};
  bool found_bin = false;
  double row_yaw = 0.0;

  auto bay_map_it = row.bays.find(bay_key);
  if (bay_map_it != row.bays.end()) {
    row_yaw = bay_map_it->second.yaw_rad;
    auto shelf_map_it = bay_map_it->second.bins.find(shelf_key);
    if (shelf_map_it != bay_map_it->second.bins.end()) {
      auto bin_map_it = shelf_map_it->second.find(bin_num);
      if (bin_map_it != shelf_map_it->second.end()) {
        ref_bin = bin_map_it->second;
        found_bin = true;
      } else if (!shelf_map_it->second.empty()) {
        ref_bin = shelf_map_it->second.begin()->second;
        found_bin = true;
      }
    }
  }

  if (!found_bin) {
    RCLCPP_WARN(node_->get_logger(),
      "SolveIkBtNode: bin %s/%s/%s/%d not found, using static poses",
      row_key.c_str(), bay_key.c_str(), shelf_key.c_str(), bin_num);
    return cfg.poses;
  }

  // TF lookup: get robot pose in map frame
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "SolveIkBtNode: TF lookup failed (%s), using static poses", ex.what());
    return cfg.poses;
  }

  const double robot_x = tf_stamped.transform.translation.x;
  const double robot_y = tf_stamped.transform.translation.y;

  tf2::Quaternion robot_q;
  tf2::fromMsg(tf_stamped.transform.rotation, robot_q);
  const double robot_yaw = tf2::getYaw(robot_q);

  // Perpendicular distance from robot to row face line (yaw set above from bay)
  const double d = (ref_bin.x - robot_x) * std::cos(row_yaw)
                 + (ref_bin.y - robot_y) * std::sin(row_yaw);

  // Yaw error: angular misalignment between robot heading and row normal
  double yaw_error = row_yaw - robot_yaw;
  while (yaw_error > M_PI) { yaw_error -= 2.0 * M_PI; }
  while (yaw_error < -M_PI) { yaw_error += 2.0 * M_PI; }

  // Forward reach in base_link frame to place gripper at standoff from row
  double x_adjusted = (d - gripper_standoff_) * std::cos(yaw_error);

  static constexpr double MIN_REACH = 0.1;
  if (x_adjusted < MIN_REACH) {
    RCLCPP_WARN(node_->get_logger(),
      "SolveIkBtNode: x_adjusted=%.3f too small (d=%.3f), clamping to %.2f",
      x_adjusted, d, MIN_REACH);
    x_adjusted = MIN_REACH;
  }

  // Yaw correction quaternion for gripper orientation
  tf2::Quaternion q_yaw_correction;
  q_yaw_correction.setRPY(0.0, 0.0, yaw_error);

  // Build adjusted poses: override x and orientation, keep y and z
  std::vector<geometry_msgs::msg::Pose> adjusted;
  adjusted.reserve(cfg.poses.size());

  for (const auto & static_pose : cfg.poses) {
    geometry_msgs::msg::Pose pose = static_pose;

    pose.position.x = x_adjusted;

    tf2::Quaternion q_static;
    tf2::fromMsg(static_pose.orientation, q_static);
    tf2::Quaternion q_adjusted = q_yaw_correction * q_static;
    q_adjusted.normalize();
    pose.orientation = tf2::toMsg(q_adjusted);

    adjusted.push_back(pose);
  }

  RCLCPP_INFO(node_->get_logger(),
    "SolveIkBtNode: adjusted %zu poses for row=%s: "
    "robot=(%.3f,%.3f,%.1f deg), bin_ref=(%.3f,%.3f), row_yaw=%.1f deg, "
    "d=%.3f, yaw_err=%.1f deg, x_adj=%.3f",
    adjusted.size(), row_key.c_str(),
    robot_x, robot_y, robot_yaw * 180.0 / M_PI,
    ref_bin.x, ref_bin.y, row_yaw * 180.0 / M_PI,
    d, yaw_error * 180.0 / M_PI, x_adjusted);

  return adjusted;
}

BT::PortsList SolveIkBtNode::providedPorts()
{
  return {
    BT::InputPort<double>("server_timeout", 30.0,
      "Timeout waiting for action server (s)"),
  };
}

void SolveIkBtNode::resetInternal()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_goal_handle_.reset();
  goal_handle_future_ = std::shared_future<std::shared_ptr<GoalHandle>>{};
  result_future_ = std::shared_future<GoalHandle::WrappedResult>{};
  phase_ = Phase::IDLE;
}

BT::NodeStatus SolveIkBtNode::onStart()
{
  halt_requested_.store(false);
  resetInternal();

  rclcpp::spin_some(node_);

  std::string shelf;
  r1_interfaces::msg::PickTicket order_copy;
  {
    std::lock_guard<std::mutex> lock(order_mutex_);
    if (!current_order_) {
      RCLCPP_ERROR(node_->get_logger(), "SolveIkBtNode: no active order");
      return BT::NodeStatus::FAILURE;
    }
    shelf = current_order_->bin.shelf;
    order_copy = *current_order_;
  }

  auto it = shelf_configs_.find(shelf);
  if (it == shelf_configs_.end()) {
    RCLCPP_ERROR(node_->get_logger(),
      "SolveIkBtNode: shelf '%s' not found in IK pose config", shelf.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const ShelfIkConfig & cfg = it->second;
  if (cfg.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
      "SolveIkBtNode: no poses configured for shelf '%s'", shelf.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(),
      "SolveIkBtNode: /solve_ik action server not available");
    return BT::NodeStatus::FAILURE;
  }

  SolveIK::Goal goal;
  if (warehouse_loaded_) {
    goal.target_ee_poses = computeAdjustedPoses(cfg, order_copy);
  } else {
    goal.target_ee_poses = cfg.poses;
    RCLCPP_WARN(node_->get_logger(),
      "SolveIkBtNode: warehouse config not loaded, using static poses");
  }
  goal.target_link = cfg.target_link;

  RCLCPP_INFO(node_->get_logger(),
    "SolveIkBtNode: sending %zu poses for shelf '%s' (link: %s)",
    goal.target_ee_poses.size(), shelf.c_str(), goal.target_link.c_str());

  rclcpp_action::Client<SolveIK>::SendGoalOptions opts;

  opts.feedback_callback =
    [this](GoalHandle::SharedPtr /*gh*/,
           const std::shared_ptr<const SolveIK::Feedback> feedback)
    {
      if (feedback) {
        RCLCPP_INFO(node_->get_logger(),
          "SolveIkBtNode: pose %u/%u, group=%s, status=%s, ready=%s",
          feedback->pose_index, feedback->total_poses,
          feedback->current_group.c_str(), feedback->status.c_str(),
          feedback->trajectory_ready ? "true" : "false");
      }
    };

  opts.goal_response_callback =
    [this](std::shared_ptr<GoalHandle> gh)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_goal_handle_ = gh;
    };

  goal_handle_future_ = client_->async_send_goal(goal, opts);
  phase_ = Phase::WAIT_GOAL_HANDLE;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SolveIkBtNode::onRunning()
{
  rclcpp::spin_some(node_);

  if (halt_requested_.load()) {
    return BT::NodeStatus::FAILURE;
  }

  if (phase_ == Phase::WAIT_GOAL_HANDLE) {
    if (!goal_handle_future_.valid()) {
      RCLCPP_ERROR(node_->get_logger(), "SolveIkBtNode: goal handle future invalid");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    if (goal_handle_future_.wait_for(std::chrono::milliseconds(100))
        != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }

    auto gh = goal_handle_future_.get();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_goal_handle_ = gh;
    }

    if (!gh) {
      RCLCPP_ERROR(node_->get_logger(), "SolveIkBtNode: goal rejected by server");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    result_future_ = client_->async_get_result(gh);
    phase_ = Phase::WAIT_RESULT;

    RCLCPP_INFO(node_->get_logger(), "SolveIkBtNode: goal accepted, waiting for result");
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::WAIT_RESULT) {
    if (!result_future_.valid()) {
      RCLCPP_ERROR(node_->get_logger(), "SolveIkBtNode: result future invalid");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    if (result_future_.wait_for(std::chrono::milliseconds(0))
        != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }

    auto wrapped = result_future_.get();
    resetInternal();

    switch (wrapped.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: {
        const bool ok = wrapped.result && wrapped.result->success;
        if (ok) {
          RCLCPP_INFO(node_->get_logger(),
            "SolveIkBtNode: IK solved successfully (%zu groups used)",
            wrapped.result->groups_used.size());
        } else {
          RCLCPP_WARN(node_->get_logger(),
            "SolveIkBtNode: IK returned success=false: %s",
            wrapped.result ? wrapped.result->error_message.c_str() : "null result");
        }
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(node_->get_logger(), "SolveIkBtNode: IK action aborted");
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "SolveIkBtNode: IK action canceled");
        return BT::NodeStatus::FAILURE;

      default:
        RCLCPP_WARN(node_->get_logger(), "SolveIkBtNode: unknown result code");
        return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void SolveIkBtNode::onHalted()
{
  halt_requested_.store(true);

  std::shared_ptr<GoalHandle> gh;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gh = active_goal_handle_;
  }

  if (!gh && goal_handle_future_.valid()) {
    RCLCPP_INFO(node_->get_logger(),
      "SolveIkBtNode::onHalted: waiting for goal handle future...");

    auto start = std::chrono::steady_clock::now();
    constexpr auto timeout = std::chrono::seconds(2);

    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(node_);

      if (goal_handle_future_.wait_for(std::chrono::milliseconds(10))
          == std::future_status::ready)
      {
        try {
          gh = goal_handle_future_.get();
        } catch (const std::exception & e) {
          RCLCPP_INFO(node_->get_logger(),
            "SolveIkBtNode::onHalted: exception getting goal handle: %s",
            e.what());
        }
        break;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_goal_handle_) {
          gh = active_goal_handle_;
          break;
        }
      }
    }
  }

  if (gh && client_) {
    RCLCPP_INFO(node_->get_logger(),
      "SolveIkBtNode::onHalted: sending cancel request to /solve_ik");
    auto cancel_future = client_->async_cancel_goal(gh);

    auto start = std::chrono::steady_clock::now();
    constexpr auto timeout = std::chrono::seconds(2);

    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(node_);
      if (cancel_future.wait_for(std::chrono::milliseconds(10))
          == std::future_status::ready)
      {
        RCLCPP_INFO(node_->get_logger(),
          "SolveIkBtNode::onHalted: cancel acknowledged");
        break;
      }
    }
  }

  resetInternal();
}

}  // namespace r1_bt_nodes
