#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__RECOVERY_NODE_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"

namespace bizon_behavior_clients
{
/// Two-child control node. Child 0 does the work, child 1 recovers from a
/// failure of child 0. The work branch is never retried until the recovery
/// branch has completed successfully.
class RecoveryNode : public BT::ControlNode
{
public:
  RecoveryNode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("number_of_retries", 3, "Recovery attempts before giving up")};
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  int retry_count_{0};
  unsigned current_child_idx_{0};
};
}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
