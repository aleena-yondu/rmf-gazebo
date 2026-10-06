#ifndef R1_NAV_CPP__BT_NODES__NOTIFY_OPERATOR_READY_HPP_
#define R1_NAV_CPP__BT_NODES__NOTIFY_OPERATOR_READY_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "r1_interfaces/msg/operator_alert.hpp"
#include "r1_interfaces/msg/pick_ticket.hpp"

namespace r1_bt_nodes
{

/**
 * @brief BT node that notifies the remote operator that the robot is staged
 *        and ready to pick/place.
 *
 * Placed immediately after the nav+servo Parallel in the Bin/Tote sequences,
 * this node publishes an r1_interfaces::msg::OperatorAlert on /operator_alert.
 * The webrtc_ros_bridge forwards it robot->teleop (latched) so an operator
 * monitoring several robots is told which one needs attention.
 *
 * The node subscribes to /teleop/order to enrich the alert with the active
 * PickTicket's context (item, sku, barcodes). It always returns SUCCESS so it
 * never disrupts the enclosing sequence.
 */
class NotifyOperatorReady : public BT::SyncActionNode
{
public:
  NotifyOperatorReady(const std::string & xml_tag_name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<r1_interfaces::msg::OperatorAlert>::SharedPtr pub_;
  rclcpp::Subscription<r1_interfaces::msg::PickTicket>::SharedPtr order_sub_;

  std::mutex data_mutex_;
  r1_interfaces::msg::PickTicket::SharedPtr current_order_;

  std::string topic_name_{"/operator_alert"};
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__NOTIFY_OPERATOR_READY_HPP_
