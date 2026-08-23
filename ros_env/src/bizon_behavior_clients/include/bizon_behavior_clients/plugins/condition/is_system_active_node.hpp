#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__IS_SYSTEM_ACTIVE_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__IS_SYSTEM_ACTIVE_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "bizon_lifecycle_manager/lifecycle_manager_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bizon_behavior_clients
{
/// Queries bizon_lifecycle_manager for whether the managed stack
/// (behavior_server, which now hosts wait/board/arm/decision) is ACTIVE.
/// This is the E-stop path: PAUSE-ing the lifecycle manager deactivates
/// behavior_server, and this node is what makes the tree notice and stop
/// commanding the arm rather than continuing blind. See chess_game.xml's
/// header comment for how the tree is wired around this node -- it must not
/// sit inside RecoveryNode's work branch, or a paused system reads as a
/// fault RecoveryNode cannot recover from (recovery needs the same
/// deactivated arm).
class IsSystemActiveNode : public BT::ConditionNode
{
public:
  IsSystemActiveNode(
    const std::string & name, const BT::NodeConfiguration & config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(
        "lifecycle_manager_name", "lifecycle_manager",
        "Name of the lifecycle manager to query")};
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<bizon_lifecycle_manager::LifecycleManagerClient> client_;
};
}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__IS_SYSTEM_ACTIVE_NODE_HPP_
