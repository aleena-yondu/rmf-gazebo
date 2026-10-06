#include "r1_nav_cpp/bt_nodes/set_tolerance.hpp"
#include "behaviortree_cpp/blackboard.h"

using namespace std::chrono_literals;

namespace r1_bt_nodes
{

SetTolerance::SetTolerance(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  // Get shared Nav2 node from blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError(
      "SetTolerance: blackboard is null. This BT node must run inside Nav2 bt_navigator.");
  }

  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "SetTolerance: 'node' not found on blackboard. "
      "This BT node must be used inside Nav2 bt_navigator.");
  }

  // Create parameter clients for both servers
  planner_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(node_, PLANNER_NODE);
  controller_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(node_, CONTROLLER_NODE);

  // Wait briefly for parameter services
  if (!planner_client_->wait_for_service(1s)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SetTolerance: Parameter service for %s not available",
      PLANNER_NODE);
  }
  if (!controller_client_->wait_for_service(1s)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SetTolerance: Parameter service for %s not available",
      CONTROLLER_NODE);
  }
}

BT::PortsList SetTolerance::providedPorts()
{
  return {
    // Planner tolerance (GridBased.tolerance)
    BT::InputPort<double>("planner_set", "Set GridBased.tolerance to this value"),
    BT::InputPort<double>("planner_step", "Increment GridBased.tolerance by this value"),
    // XY goal tolerance (goal_checker.xy_goal_tolerance)
    BT::InputPort<double>("xy_set", "Set goal_checker.xy_goal_tolerance to this value"),
    BT::InputPort<double>("xy_step", "Increment goal_checker.xy_goal_tolerance by this value"),
    // Yaw goal tolerance (goal_checker.yaw_goal_tolerance)
    BT::InputPort<double>("yaw_set", "Set goal_checker.yaw_goal_tolerance to this value"),
    BT::InputPort<double>("yaw_step", "Increment goal_checker.yaw_goal_tolerance by this value")
  };
}

bool SetTolerance::setOrIncrementParam(
  std::shared_ptr<rclcpp::AsyncParametersClient> & client,
  const std::string & param_name,
  const BT::Expected<double> & set_value,
  const BT::Expected<double> & step_value,
  const std::string & target_node)
{
  // If neither port is provided, do nothing
  if (!set_value && !step_value) {
    return true;
  }

  double new_value = 0.0;

  // Case 1: Explicit set (takes precedence)
  if (set_value) {
    new_value = set_value.value();
  }
  // Case 2: Increment current value
  else if (step_value) {
    // Get current parameter value
    auto future = client->get_parameters({param_name});

    if (rclcpp::spin_until_future_complete(
          node_, future, 500ms) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "SetTolerance: Failed to read parameter %s from %s",
        param_name.c_str(),
        target_node.c_str());
      return false;
    }

    const auto & params = future.get();
    if (params.empty()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "SetTolerance: Parameter %s not found on %s",
        param_name.c_str(),
        target_node.c_str());
      return false;
    }

    new_value = params[0].as_double() + step_value.value();
  }

  // Clamp to reasonable range (avoid negative tolerances)
  if (new_value < 0.0) {
    new_value = 0.0;
  }

  // Set parameter
  rclcpp::Parameter param(param_name, new_value);
  auto set_future = client->set_parameters({param});

  if (rclcpp::spin_until_future_complete(
        node_, set_future, 500ms) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetTolerance: Failed to set parameter %s on %s",
      param_name.c_str(),
      target_node.c_str());
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "SetTolerance: Set %s.%s to %.3f",
    target_node.c_str(),
    param_name.c_str(),
    new_value);

  return true;
}

BT::NodeStatus SetTolerance::tick()
{
  // Read all input ports
  auto planner_set = getInput<double>("planner_set");
  auto planner_step = getInput<double>("planner_step");
  auto xy_set = getInput<double>("xy_set");
  auto xy_step = getInput<double>("xy_step");
  auto yaw_set = getInput<double>("yaw_set");
  auto yaw_step = getInput<double>("yaw_step");

  bool success = true;

  // Set/increment planner tolerance
  if (!setOrIncrementParam(
        planner_client_, PLANNER_TOLERANCE_PARAM,
        planner_set, planner_step, PLANNER_NODE))
  {
    success = false;
  }

  // Set/increment xy goal tolerance
  if (!setOrIncrementParam(
        controller_client_, XY_TOLERANCE_PARAM,
        xy_set, xy_step, CONTROLLER_NODE))
  {
    success = false;
  }

  // Set/increment yaw goal tolerance
  if (!setOrIncrementParam(
        controller_client_, YAW_TOLERANCE_PARAM,
        yaw_set, yaw_step, CONTROLLER_NODE))
  {
    success = false;
  }

  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace r1_bt_nodes
