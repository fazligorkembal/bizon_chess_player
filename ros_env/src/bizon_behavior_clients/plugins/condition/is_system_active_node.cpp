#include "bizon_behavior_clients/plugins/condition/is_system_active_node.hpp"

namespace bizon_behavior_clients
{

IsSystemActiveNode::IsSystemActiveNode(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string manager_name = "lifecycle_manager";
  getInput("lifecycle_manager_name", manager_name);

  client_ = std::make_shared<bizon_lifecycle_manager::LifecycleManagerClient>(
    manager_name, node_);
}

BT::NodeStatus IsSystemActiveNode::tick()
{
  // Only make the real service call (and log) when a fresh poll is due;
  // ticks in between reuse last_status_. See poll_throttle_'s declaration
  // for why this node is ticked far more often than it needs to actually
  // ask the lifecycle manager anything.
  if (poll_throttle_.shouldPollNow(std::chrono::steady_clock::now())) {
    last_status_ = client_->is_active(std::chrono::seconds(1));
    if (last_status_ != bizon_lifecycle_manager::SystemStatus::ACTIVE) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Lifecycle manager reports the system is not active; refusing to command the arm");
    }
  }

  return last_status_ == bizon_lifecycle_manager::SystemStatus::ACTIVE ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace bizon_behavior_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bizon_behavior_clients::IsSystemActiveNode>("IsSystemActive");
}
