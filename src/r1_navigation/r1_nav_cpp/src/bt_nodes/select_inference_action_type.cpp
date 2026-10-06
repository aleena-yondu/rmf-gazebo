#include "r1_nav_cpp/bt_nodes/select_inference_action_type.hpp"

#include <functional>

#include "behaviortree_cpp/blackboard.h"

namespace r1_bt_nodes
{

namespace
{

constexpr char kServoBin[] = "servo_bin";
constexpr char kServoTote[] = "servo_tote";
constexpr char kInferencePick[] = "inference_pick";
constexpr char kInferencePlace[] = "inference_place";

bool isValidState(const r1_interfaces::msg::BTState & state)
{
  const bool valid_mode = state.pick != state.place;
  const bool valid_bin = state.bin == "A" || state.bin == "B" || state.bin == "C";
  const bool valid_tote = state.tote >= 0 && state.tote <= 3;
  return valid_mode && valid_bin && valid_tote;
}

r1_interfaces::msg::BTState defaultState()
{
  r1_interfaces::msg::BTState state;
  state.pick = true;
  state.place = false;
  state.bin = "C";
  state.tote = 3;
  return state;
}

}  // namespace

SelectInferenceActionType::SelectInferenceActionType(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard) {
    throw BT::RuntimeError(
      "SelectInferenceActionType: blackboard is null. "
      "This BT node must run inside Nav2 bt_navigator.");
  }

  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "SelectInferenceActionType: 'node' not found on blackboard. "
      "This BT node must run inside Nav2 bt_navigator.");
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .reliable()
    .transient_local();

  state_sub_ = node_->create_subscription<r1_interfaces::msg::BTState>(
    "/teleop_bt_config",
    qos,
    std::bind(&SelectInferenceActionType::stateCallback, this, std::placeholders::_1));
}

BT::PortsList SelectInferenceActionType::providedPorts()
{
  return {
    BT::OutputPort<std::string>(
      "command",
      "Selected command: servo_bin, servo_tote, inference_pick, or inference_place"),
    BT::OutputPort<std::string>(
      "bin",
      "Normalized bin shelf: A, B, or C"),
    BT::OutputPort<int>(
      "tote",
      "Normalized tote row: 0, 1, 2, or 3")
  };
}

void SelectInferenceActionType::stateCallback(
  const r1_interfaces::msg::BTState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = *msg;
  has_received_state_ = true;
}

BT::NodeStatus SelectInferenceActionType::tick()
{
  rclcpp::spin_some(node_);

  r1_interfaces::msg::BTState state;
  bool has_received_state = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
    has_received_state = has_received_state_;
  }

  const bool valid_received_state = has_received_state && isValidState(state);
  if (!valid_received_state) {
    state = defaultState();
  }

  const bool pick = state.pick;
  const bool place = state.place;
  const std::string & bin = state.bin;
  const int tote = state.tote;

  std::string command;
  if (first_tick_) {
    command = pick ? kServoBin : kServoTote;
  } else if (pick) {
    command = previous_place_ ? kServoBin : kInferencePick;
  } else {
    command = previous_pick_ ? kServoTote : kInferencePlace;
  }

  const bool was_first_tick = first_tick_;
  first_tick_ = false;
  previous_pick_ = pick;
  previous_place_ = place;

  setOutput("command", command);
  setOutput("bin", bin);
  setOutput("tote", tote);
  RCLCPP_INFO(
    node_->get_logger(),
    "SelectInferenceActionType: received=%s, valid=%s, first_tick=%s, "
    "pick=%s, place=%s, bin=%s, tote=%d, command=%s",
    has_received_state ? "true" : "false",
    valid_received_state ? "true" : "false",
    was_first_tick ? "true" : "false",
    pick ? "true" : "false",
    place ? "true" : "false",
    bin.c_str(),
    tote,
    command.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace r1_bt_nodes
