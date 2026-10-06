#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <cmath>
#include <limits>
#include <cstdlib>

#include "r1_nav_cpp/bt_nodes/servo_to_target_action.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include "rclcpp/qos.hpp"
#include "rclcpp/logging.hpp"

namespace r1_bt_nodes
{

ServoToTargetAction::ServoToTargetAction(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf),
  timeout_(10.0)
{
  if (!config().blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "ServoToTargetAction: 'node' not found on blackboard. "
      "This BT node must be used inside Nav2 bt_navigator.");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "ServoToTargetAction: using shared Nav2 node '%s'",
    node_->get_name());

  action_client_ = rclcpp_action::create_client<ServoHeight>(node_, "servo_height");

  rclcpp::QoS order_qos(1);
  order_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  order_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  order_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  order_sub_ = node_->create_subscription<r1_interfaces::msg::PickTicket>(
    "/teleop/order",
    order_qos,
    std::bind(&ServoToTargetAction::orderCallback, this, std::placeholders::_1));

  loadConfig();
}

// ---------------------------------------------------------------------
// Config loading
// ---------------------------------------------------------------------
void ServoToTargetAction::loadConfig()
{
  if (!shelf_height_map_.empty() || !tote_height_map_.empty()) {
    return;
  }

  std::string pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("r1_nav_cpp");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ServoToTargetAction: failed to get package share directory: %s",
      e.what());
    throw;
  }

  const char * loc_env = std::getenv("LOCATION");
  std::string location = (loc_env && loc_env[0] != '\0') ? loc_env : "yondu";
  std::transform(location.begin(), location.end(), location.begin(), ::tolower);

  const char * lidar_env = std::getenv("LIDAR");
  std::string lidar = (lidar_env && lidar_env[0] != '\0') ? lidar_env : "livox";
  std::transform(lidar.begin(), lidar.end(), lidar.begin(), ::tolower);

  const std::string location_config_path = 
    pkg_share + "/config/warehouse_configs/" + location + "/" + lidar + "/warehouse.yaml";

  YAML::Node location_root;
  try {
    location_root = YAML::LoadFile(location_config_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ServoToTargetAction: failed to load YAML file '%s': %s",
      location_config_path.c_str(), e.what());
    throw;
  }

  YAML::Node rows = location_root["warehouse"]["rows"];
  if (rows && rows.IsMap()) {
    for (auto row_it = rows.begin(); row_it != rows.end(); ++row_it) {
      const std::string row_id = row_it->first.as<std::string>();
      YAML::Node row_node = row_it->second;

      YAML::Node shelf_heights = row_node["shelf_heights"];
      if (shelf_heights && shelf_heights.IsMap()) {
        for (auto it = shelf_heights.begin(); it != shelf_heights.end(); ++it) {
          std::string key = it->first.as<std::string>();
          int val = it->second.as<int>();
          if (shelf_height_map_.find(key) == shelf_height_map_.end()) {
            shelf_height_map_[key] = val;
          }
        }
      }

      // New schema: a single per-row search_height (servo 0..100) replaces the
      // old per-shelf shelf_search_heights map.
      if (row_node["search_height"]) {
        row_search_height_map_[row_id] = row_node["search_height"].as<int>();
      }
    }
  }

  if (shelf_height_map_.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ServoToTargetAction: no shelf_heights found in '%s'",
      location_config_path.c_str());
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "ServoToTargetAction: loaded %zu shelf heights from '%s'",
      shelf_height_map_.size(), location_config_path.c_str());
  }

  YAML::Node warehouse_node = location_root["warehouse"];

  YAML::Node tote_map = warehouse_node["tote_row_heights"];
  if (tote_map && tote_map.IsMap()) {
    for (auto it = tote_map.begin(); it != tote_map.end(); ++it) {
      tote_height_map_[it->first.as<int>()] = it->second.as<int>();
    }
  }

  if (tote_height_map_.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ServoToTargetAction: no tote_row_heights found in '%s'",
      location_config_path.c_str());
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "ServoToTargetAction: loaded %zu tote heights from '%s'",
      tote_height_map_.size(), location_config_path.c_str());
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "ServoToTargetAction: config loaded from '%s' (%zu rows w/ search_height)",
    location_config_path.c_str(), row_search_height_map_.size());
}

// ---------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------
void ServoToTargetAction::orderCallback(
  const r1_interfaces::msg::PickTicket::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (msg->item_name == "NONE") {
    current_order_.reset();
  } else {
    current_order_ = msg;
  }
}

// ---------------------------------------------------------------------
// Height lookup
// ---------------------------------------------------------------------

double ServoToTargetAction::getTargetHeight()
{
  r1_interfaces::msg::PickTicket::SharedPtr current_order;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_order = current_order_;
  }

  if (target_type_ == "BIN") {
    std::string shelf;
    if (has_explicit_bin_) {
      shelf = explicit_bin_;
    } else if (current_order) {
      shelf = current_order->bin.shelf;
    } else {
      return std::numeric_limits<double>::quiet_NaN();
    }

    auto it = shelf_height_map_.find(shelf);
    if (it == shelf_height_map_.end()) {
      RCLCPP_WARN(node_->get_logger(),
        "ServoToTargetAction: shelf '%s' not found in shelf_height_mapping", shelf.c_str());
      return std::numeric_limits<double>::quiet_NaN();
    }
    RCLCPP_INFO(node_->get_logger(),
      "ServoToTargetAction: shelf '%s' height=%.1f", shelf.c_str(), static_cast<double>(it->second));
    return static_cast<double>(it->second);

  } else if (target_type_ == "TOTE") {
    int row = 0;
    if (has_explicit_tote_) {
      row = explicit_tote_;
    } else if (current_order) {
      row = current_order->tote.row;
    } else {
      return std::numeric_limits<double>::quiet_NaN();
    }

    auto it = tote_height_map_.find(row);
    if (it == tote_height_map_.end()) {
      RCLCPP_WARN(node_->get_logger(),
        "ServoToTargetAction: row '%d' not found in tote_row_height_mapping", row);
      return std::numeric_limits<double>::quiet_NaN();
    }
    RCLCPP_INFO(node_->get_logger(),
      "ServoToTargetAction: tote row '%d' height=%.1f", row, static_cast<double>(it->second));
    return static_cast<double>(it->second);
    
  } else if (target_type_ == "BIN_SEARCH") {
    if (!current_order) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const std::string row_id =
      current_order->bin.side + std::to_string(current_order->bin.row);
    auto it = row_search_height_map_.find(row_id);
    if (it == row_search_height_map_.end()) {
      RCLCPP_WARN(node_->get_logger(),
        "ServoToTargetAction: row '%s' not found in row_search_height_mapping",
        row_id.c_str());
      return std::numeric_limits<double>::quiet_NaN();
    }
    RCLCPP_INFO(node_->get_logger(),
      "ServoToTargetAction: row '%s' search height=%.1f",
      row_id.c_str(), static_cast<double>(it->second));
    return static_cast<double>(it->second);

  } else if (target_type_ == "TOTE_SEARCH") {
    if (!current_order) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    int row = current_order->tote.row;
    auto it = tote_search_height_map_.find(row);
    if (it == tote_search_height_map_.end()) {
      RCLCPP_WARN(node_->get_logger(),
        "ServoToTargetAction: row '%d' not found in tote_row_search_height_mapping", row);
      return std::numeric_limits<double>::quiet_NaN();
    }
    RCLCPP_INFO(node_->get_logger(),
      "ServoToTargetAction: tote row '%d' search height=%.1f",
      row, static_cast<double>(it->second));
    return static_cast<double>(it->second);
  }

  RCLCPP_WARN(node_->get_logger(),
    "ServoToTargetAction: unknown target_type '%s'", target_type_.c_str());
  return std::numeric_limits<double>::quiet_NaN();
}

// ---------------------------------------------------------------------
// BT lifecycle
// ---------------------------------------------------------------------

BT::NodeStatus ServoToTargetAction::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "ServoToTargetAction: onStart()");

  if (!getInput("target_type", target_type_)) {
    target_type_ = "BIN";
  }
  has_explicit_bin_ = getInput("bin", explicit_bin_).has_value();
  has_explicit_tote_ = getInput("tote", explicit_tote_).has_value();

  if (has_explicit_bin_ &&
      explicit_bin_ != "A" && explicit_bin_ != "B" && explicit_bin_ != "C")
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "ServoToTargetAction: invalid explicit bin '%s'; defaulting to C",
      explicit_bin_.c_str());
    explicit_bin_ = "C";
  }
  if (has_explicit_tote_ && (explicit_tote_ < 0 || explicit_tote_ > 3)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ServoToTargetAction: invalid explicit tote '%d'; defaulting to 3",
      explicit_tote_);
    explicit_tote_ = 3;
  }

  if (!getInput("server_timeout", timeout_)) {
    timeout_ = 10.0;
  }

  start_time_ = node_->get_clock()->now();
  goal_done_ = false;
  goal_succeeded_ = false;
  goal_handle_ = nullptr;

  double height = getTargetHeight();
  if (std::isnan(height)) {
    RCLCPP_ERROR(node_->get_logger(),
      "ServoToTargetAction: no valid height for target_type=%s", target_type_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(),
      "ServoToTargetAction: servo_height action server not available");
    return BT::NodeStatus::FAILURE;
  }

  auto goal = ServoHeight::Goal();
  goal.height = height;

  auto send_goal_options = rclcpp_action::Client<ServoHeight>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandleServoHeight::SharedPtr & gh) {
      if (!gh) {
        RCLCPP_ERROR(node_->get_logger(), "ServoToTargetAction: goal rejected by server");
        goal_done_ = true;
        goal_succeeded_ = false;
      } else {
        RCLCPP_INFO(node_->get_logger(), "ServoToTargetAction: goal accepted");
        goal_handle_ = gh;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleServoHeight::WrappedResult & result) {
      goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                         result.result->success);
      goal_done_ = true;
      RCLCPP_INFO(node_->get_logger(), "ServoToTargetAction: result=%s",
                  goal_succeeded_ ? "SUCCESS" : "FAILED");
    };

  send_goal_options.feedback_callback =
    [this](GoalHandleServoHeight::SharedPtr,
           const std::shared_ptr<const ServoHeight::Feedback> feedback) {
      RCLCPP_DEBUG(node_->get_logger(), "ServoToTargetAction: phase=%s",
                   feedback->phase.c_str());
    };

  RCLCPP_INFO(node_->get_logger(),
    "ServoToTargetAction: sending servo goal height=%.1f for [%s]",
    height, target_type_.c_str());

  action_client_->async_send_goal(goal, send_goal_options);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ServoToTargetAction::onRunning()
{
  rclcpp::spin_some(node_);

  if (goal_done_) {
    return goal_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  const double elapsed = (node_->get_clock()->now() - start_time_).seconds();
  if (elapsed > timeout_) {
    RCLCPP_ERROR(node_->get_logger(),
      "ServoToTargetAction: timeout (%.2fs > %.2fs)", elapsed, timeout_);
    if (goal_handle_) {
      action_client_->async_cancel_goal(goal_handle_);
    }
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void ServoToTargetAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "ServoToTargetAction: onHalted() - canceling goal");
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
}

}  // namespace r1_bt_nodes
