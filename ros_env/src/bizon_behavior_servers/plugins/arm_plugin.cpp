#include "bizon_behavior_servers/plugins/arm_plugin.hpp"

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace bizon_behaviors
{

ArmPlugin::ArmPlugin() = default;
ArmPlugin::~ArmPlugin() = default;

void ArmPlugin::onConfigure()
{
  auto node = node_.lock();

  if (!node->has_parameter(behavior_name_ + ".planning_time")) {
    node->declare_parameter(behavior_name_ + ".planning_time", 15.0);
  }
  node->get_parameter(behavior_name_ + ".planning_time", planning_time_);

  // See the header comment on moveit_node_: MoveGroupInterface needs a
  // plain rclcpp::Node, which the LifecycleNode we were configured with is
  // not, so we spin a dedicated helper node for it instead.
  rclcpp::NodeOptions moveit_node_options;
  moveit_node_options.automatically_declare_parameters_from_overrides(true);
  moveit_node_ = std::make_shared<rclcpp::Node>(
    behavior_name_ + "_moveit_client", node->get_namespace(), moveit_node_options);
  moveit_node_thread_ = std::make_unique<bizon_util::NodeThread>(moveit_node_);

  moveit::planning_interface::MoveGroupInterface::Options arm_options(
    "arm_group", "robot_description", node->get_namespace());
  moveit::planning_interface::MoveGroupInterface::Options hand_options(
    "hand_group", "robot_description", node->get_namespace());

  move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    moveit_node_, arm_options);
  move_group_hand_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    moveit_node_, hand_options);

  move_group_arm_->setPlanningTime(planning_time_);
  move_group_hand_->setPlanningTime(planning_time_);

  RCLCPP_INFO(node->get_logger(), "[%s] ArmPlugin configured", behavior_name_.c_str());
}

void ArmPlugin::onCleanup()
{
  move_group_arm_.reset();
  move_group_hand_.reset();
  moveit_node_thread_.reset();
  moveit_node_.reset();
}

ResultStatus ArmPlugin::onRun(const std::shared_ptr<const ArmAction::Goal> command)
{
  auto node = node_.lock();

  // target_hand_position is always required: even a hand_only goal has to
  // say where the gripper should go.
  if (command->target_hand_position.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] empty hand target in goal", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 1};
  }

  target_hand_position_ = command->target_hand_position;
  hand_only_ = command->hand_only;

  if (command->hand_only) {
    // The recovery subtree's first step must open the gripper to release a
    // held piece without moving the arm -- moving first would drag the
    // piece across the board. So when hand_only is set, skip the arm
    // entirely: do not validate or use target_joint_positions, do not call
    // setJointValueTarget on the arm group, do not launch an arm move.
    // future_arm_ is left untouched (default-constructed) and must never be
    // waited on or get()-ed while phase_ == HAND_MOVING was entered this way.
    RCLCPP_INFO(
      node->get_logger(), "[%s] hand_only goal: arm left stationary", behavior_name_.c_str());

    move_group_hand_->setJointValueTarget(target_hand_position_);
    future_hand_ = std::async(std::launch::async, [this]() { return move_group_hand_->move(); });
    phase_ = Phase::HAND_MOVING;

    return ResultStatus{Status::SUCCEEDED, 0};
  }

  if (command->target_joint_positions.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] empty joint target in goal", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 1};
  }

  if (move_group_arm_->getCurrentJointValues().empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] no joint state available", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 2};
  }

  move_group_arm_->setJointValueTarget(command->target_joint_positions);
  future_arm_ = std::async(std::launch::async, [this]() { return move_group_arm_->move(); });
  phase_ = Phase::ARM_MOVING;

  return ResultStatus{Status::SUCCEEDED, 0};
}

ResultStatus ArmPlugin::onCycleUpdate()
{
  auto node = node_.lock();

  if (phase_ == Phase::ARM_MOVING) {
    if (future_arm_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
      return ResultStatus{Status::RUNNING, 0};
    }
    const auto result = future_arm_.get();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "[%s] arm move failed: %d", behavior_name_.c_str(), result.val);
      return ResultStatus{Status::FAILED, static_cast<uint16_t>(-result.val)};
    }

    move_group_hand_->setJointValueTarget(target_hand_position_);
    future_hand_ = std::async(std::launch::async, [this]() { return move_group_hand_->move(); });
    phase_ = Phase::HAND_MOVING;
    return ResultStatus{Status::RUNNING, 0};
  }

  // phase_ == Phase::HAND_MOVING. This branch is reached either after a
  // completed arm move (future_hand_ launched just above) or directly from a
  // hand_only goal (future_hand_ launched in onRun). Either way future_arm_
  // is never touched here.
  if (future_hand_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    return ResultStatus{Status::RUNNING, 0};
  }
  const auto result = future_hand_.get();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[%s] hand move failed: %d", behavior_name_.c_str(), result.val);
    return ResultStatus{Status::FAILED, static_cast<uint16_t>(-result.val)};
  }

  return ResultStatus{Status::SUCCEEDED, 0};
}

void ArmPlugin::onActionCompletion(std::shared_ptr<ArmAction::Result> result)
{
  DebugSession::instance().logEvent(
    "arm", "hand_only=" + std::string(hand_only_ ? "true" : "false") +
    " error_code=" + std::to_string(result->error_code));
}

}  // namespace bizon_behaviors

PLUGINLIB_EXPORT_CLASS(bizon_behaviors::ArmPlugin, bizon_core::Behavior)
