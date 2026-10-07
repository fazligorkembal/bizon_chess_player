#include "bizon_behavior_clients/plugins/action/wait_action_client_node.hpp"

namespace bizon_behavior_clients
{
WaitActionClientNode::WaitActionClientNode(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionClientNode<bizon_msgs::action::Wait>(xml_tag_name, action_name, conf)
{
  RCLCPP_INFO(
    rclcpp::get_logger("WaitActionClientNode"),
    "WaitActionClientNode created for action: %s",
    action_name.c_str());
}

void WaitActionClientNode::on_tick()
{
  unsigned int sec, nanosec;

  if (!getInput("sec", sec)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("WaitActionClientNode"),
      "Missing required input port [sec]");
    throw BT::RuntimeError("missing required input port [sec]");
  }
  if (!getInput("nanosec", nanosec)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("WaitActionClientNode"),
      "Missing required input port [nanosec]");
    throw BT::RuntimeError("missing required input port [nanosec]");
  }

  goal_.time.sec = sec;
  goal_.time.nanosec = nanosec;
}

BT::NodeStatus WaitActionClientNode::on_success()
{
  RCLCPP_INFO(
    rclcpp::get_logger("WaitActionClientNode"),
    "Wait action succeeded");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitActionClientNode::on_aborted()
{
  RCLCPP_ERROR(
    rclcpp::get_logger("WaitActionClientNode"),
    "Wait action aborted");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus WaitActionClientNode::on_cancelled()
{
  RCLCPP_WARN(
    rclcpp::get_logger("WaitActionClientNode"),
    "Wait action canceled");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus WaitActionClientNode::onResultReceived(
  const typename rclcpp_action::ClientGoalHandle<bizon_msgs::action::Wait>::WrappedResult & result)
{
  return BT::NodeStatus::SUCCESS;
}
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<bizon_behavior_clients::WaitActionClientNode>(
        name, "wait_action", config);
    };
  factory.registerBuilder<bizon_behavior_clients::WaitActionClientNode>(
    "WaitActionClient", builder);
}
