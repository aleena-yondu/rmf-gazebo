#ifndef R1_NAV_CPP__BT_NODES__SET_TOLERANCE_HPP_
#define R1_NAV_CPP__BT_NODES__SET_TOLERANCE_HPP_

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

namespace r1_bt_nodes
{

/**
 * @brief BT node to set/increment tolerances on planner_server and controller_server
 *
 * Parameters controlled:
 *   - GridBased.tolerance on /planner_server
 *   - goal_checker.xy_goal_tolerance on /controller_server
 *   - goal_checker.yaw_goal_tolerance on /controller_server
 *
 * Each parameter has a "set" port (absolute value) and "step" port (increment).
 * If "set" is provided, it takes precedence over "step".
 */
class SetTolerance : public BT::SyncActionNode
{
public:
  SetTolerance(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  /**
   * @brief Set or increment a single parameter
   * @param client Parameter client for the target node
   * @param param_name Full parameter name
   * @param set_value Optional absolute value to set
   * @param step_value Optional increment value
   * @param target_node Name of target node (for logging)
   * @return true if successful or no operation needed, false on error
   */
  bool setOrIncrementParam(
    std::shared_ptr<rclcpp::AsyncParametersClient> & client,
    const std::string & param_name,
    const BT::Expected<double> & set_value,
    const BT::Expected<double> & step_value,
    const std::string & target_node);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> planner_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> controller_client_;

  static constexpr const char * PLANNER_NODE = "/planner_server";
  static constexpr const char * CONTROLLER_NODE = "/controller_server";
  static constexpr const char * PLANNER_TOLERANCE_PARAM = "GridBased.tolerance";
  static constexpr const char * XY_TOLERANCE_PARAM = "goal_checker.xy_goal_tolerance";
  static constexpr const char * YAW_TOLERANCE_PARAM = "goal_checker.yaw_goal_tolerance";
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__SET_TOLERANCE_HPP_
