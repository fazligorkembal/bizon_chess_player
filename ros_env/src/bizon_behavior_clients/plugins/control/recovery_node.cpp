#include "bizon_behavior_clients/plugins/control/recovery_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bizon_behavior_clients
{

RecoveryNode::RecoveryNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus RecoveryNode::tick()
{
  if (children_nodes_.size() != 2) {
    // BT::RuntimeError is what the rest of this package throws (see foreach_node.cpp:20).
    throw BT::RuntimeError("RecoveryNode '" + name() + "' must have exactly 2 children");
  }

  int max_retries = 3;
  getInput("number_of_retries", max_retries);

  setStatus(BT::NodeStatus::RUNNING);

  while (true) {
    if (current_child_idx_ == 0) {
      const BT::NodeStatus child_status = children_nodes_[0]->executeTick();

      if (child_status == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }

      if (child_status == BT::NodeStatus::SUCCESS) {
        halt();
        return BT::NodeStatus::SUCCESS;
      }

      // FAILURE
      haltChild(0);
      if (retry_count_ >= max_retries) {
        RCLCPP_ERROR(
          rclcpp::get_logger("RecoveryNode"),
          "[%s] work branch failed and %d retries are exhausted, giving up",
          name().c_str(), max_retries);
        halt();
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_WARN(
        rclcpp::get_logger("RecoveryNode"),
        "[%s] work branch failed, running recovery (attempt %d/%d)",
        name().c_str(), retry_count_ + 1, max_retries);
      current_child_idx_ = 1;
      continue;
    }

    // current_child_idx_ == 1: recovery branch
    const BT::NodeStatus recovery_status = children_nodes_[1]->executeTick();

    if (recovery_status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }

    haltChild(1);

    if (recovery_status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RecoveryNode"),
        "[%s] recovery branch itself failed, giving up", name().c_str());
      halt();
      return BT::NodeStatus::FAILURE;
    }

    // Recovery succeeded: count the attempt and let the work branch run again.
    retry_count_++;
    current_child_idx_ = 0;
  }
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

}  // namespace bizon_behavior_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bizon_behavior_clients::RecoveryNode>("RecoveryNode");
}
