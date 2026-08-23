#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__IS_SYSTEM_ACTIVE_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__IS_SYSTEM_ACTIVE_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "bizon_behavior_clients/plugins/condition/poll_throttle.hpp"
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
/// deactivated arm), and it must be reached through a ReactiveFallback so a
/// pause mid-move halts the arm immediately instead of only at the next
/// move boundary.
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
  // The ReactiveFallback above this node ticks it on every BT tick
  // (~10ms) for as long as the system stays paused, so an unthrottled real
  // service call here would drive roughly 100 is_active round-trips (and
  // the log lines each one produces) per second for as long as the E-stop
  // is held. poll_throttle_ bounds real polling to once every
  // kPollInterval_; ticks in between reuse last_status_. See
  // poll_throttle.hpp for why this is safe for the E-stop's own
  // responsiveness.
  static constexpr std::chrono::milliseconds kPollInterval_{200};

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<bizon_lifecycle_manager::LifecycleManagerClient> client_;
  PollThrottle poll_throttle_{kPollInterval_};
  // Fails closed: until the first real poll runs (which happens
  // immediately -- PollThrottle always allows the first call), this reads
  // as "not active", so no tick between construction and that first poll
  // can slip through as SUCCESS.
  bizon_lifecycle_manager::SystemStatus last_status_{
    bizon_lifecycle_manager::SystemStatus::INACTIVE};
};
}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__IS_SYSTEM_ACTIVE_NODE_HPP_
