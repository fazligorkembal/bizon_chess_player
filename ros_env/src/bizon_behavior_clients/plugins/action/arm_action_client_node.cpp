#include "bizon_behavior_clients/plugins/action/arm_action_client_node.hpp"

namespace bizon_behavior_clients
{

ArmActionClientNode::ArmActionClientNode(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionClientNode<bizon_msgs::action::Arm>(xml_tag_name, action_name, conf)
{
}

void ArmActionClientNode::on_tick()
{
  std::vector<double> hand;
  std::string player_side;
  bool hand_only = false;
  std::string board_fen;
  std::string target_square;

  getInput("target_hand_position", hand);
  getInput("player_side", player_side);
  getInput("hand_only", hand_only);
  getInput("board_fen", board_fen);
  getInput("target_square", target_square);

  goal_.target_hand_position = hand;
  goal_.player_side = player_side;
  goal_.hand_only = hand_only;
  goal_.board_fen = board_fen;
  goal_.target_square = target_square;

  // hand_only goals (e.g. the recovery subtree's first step) never read or
  // send target_joint_positions: moving the arm first would drag a held
  // piece across the board before the gripper releases it.
  if (hand_only) {
    goal_.target_joint_positions.clear();
  } else {
    std::vector<double> joints;
    getInput("target_joint_positions", joints);
    goal_.target_joint_positions = joints;
  }
}

BT::NodeStatus ArmActionClientNode::onResultReceived(
  const rclcpp_action::ClientGoalHandle<bizon_msgs::action::Arm>::WrappedResult & result)
{
  if (result.result->error_code != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArmActionClientNode"),
      "Arm action reported error_code %u", result.result->error_code);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ArmActionClientNode::on_aborted()
{
  RCLCPP_ERROR(rclcpp::get_logger("ArmActionClientNode"), "Arm action aborted");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ArmActionClientNode::on_cancelled()
{
  RCLCPP_WARN(rclcpp::get_logger("ArmActionClientNode"), "Arm action cancelled");
  return BT::NodeStatus::FAILURE;
}

}  // namespace bizon_behavior_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<bizon_behavior_clients::ArmActionClientNode>(
      name, "arm_action", config);
  };
  factory.registerBuilder<bizon_behavior_clients::ArmActionClientNode>("ArmActionClient", builder);
}
