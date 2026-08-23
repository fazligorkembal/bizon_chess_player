#ifndef BIZON_BEHAVIOR_SERVERS__PLUGINS__ARM_PLUGIN_HPP_
#define BIZON_BEHAVIOR_SERVERS__PLUGINS__ARM_PLUGIN_HPP_

#include <future>
#include <memory>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>

#include "bizon_behavior_servers/timed_behavior.hpp"
#include "bizon_msgs/action/arm.hpp"
#include "bizon_util/node_thread.hpp"

namespace bizon_behaviors
{
/// Owns the MoveGroupInterface for one robot. Arm motion runs to completion
/// before the gripper is commanded, so a settling correction can never occur
/// while the fingers are closing.
///
/// When the goal's `hand_only` field is set, the arm is left completely
/// stationary: onRun skips straight to commanding the gripper and enters
/// Phase::HAND_MOVING directly, so future_arm_ is never populated for that
/// goal. This exists so the recovery subtree can open the gripper to release
/// a held piece without dragging it across the board first.
class ArmPlugin : public TimedBehavior<bizon_msgs::action::Arm>
{
public:
  using ArmAction = bizon_msgs::action::Arm;

  ArmPlugin();
  ~ArmPlugin() override;

  void onConfigure() override;
  void onCleanup() override;

  ResultStatus onRun(const std::shared_ptr<const ArmAction::Goal> command) override;
  ResultStatus onCycleUpdate() override;
  void onActionCompletion(std::shared_ptr<ArmAction::Result> result) override;

private:
  enum class Phase
  {
    ARM_MOVING,
    HAND_MOVING,
  };

  // MoveGroupInterface requires a plain rclcpp::Node::SharedPtr, but
  // TimedBehavior only exposes the behavior_server's LifecycleNode (which
  // does not derive from rclcpp::Node, so it cannot be handed to
  // MoveGroupInterface directly). moveit_node_ is a small helper node,
  // spun on its own background thread by moveit_node_thread_, that exists
  // solely to host the two MoveGroupInterface instances below.
  rclcpp::Node::SharedPtr moveit_node_;
  std::unique_ptr<bizon_util::NodeThread> moveit_node_thread_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_hand_;

  std::future<moveit::core::MoveItErrorCode> future_arm_;
  std::future<moveit::core::MoveItErrorCode> future_hand_;

  Phase phase_{Phase::ARM_MOVING};
  std::vector<double> target_hand_position_;
  double planning_time_{15.0};
  // Set in onRun(), read back in onActionCompletion() for events.log --
  // the goal itself is gone by the time onActionCompletion() runs (see
  // TimedBehavior::execute()).
  bool hand_only_{false};
};
}  // namespace bizon_behaviors

#endif  // BIZON_BEHAVIOR_SERVERS__PLUGINS__ARM_PLUGIN_HPP_
