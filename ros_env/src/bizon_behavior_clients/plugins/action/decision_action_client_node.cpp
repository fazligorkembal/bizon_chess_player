#include "bizon_behavior_clients/plugins/action/decision_action_client_node.hpp"

namespace bizon_behavior_clients
{

DecisionActionClientNode::DecisionActionClientNode(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionClientNode<bizon_msgs::action::Decision>(xml_tag_name, action_name, conf)
{
}

void DecisionActionClientNode::on_tick()
{
  std::string player_side;
  std::string fen;

  getInput("player_side", player_side);
  getInput("fen", fen);

  goal_.player_side = player_side;
  goal_.fen = fen;
}

BT::NodeStatus DecisionActionClientNode::onResultReceived(
  const rclcpp_action::ClientGoalHandle<bizon_msgs::action::Decision>::WrappedResult & result)
{
  if (result.result->error_code != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DecisionActionClientNode"),
      "Decision action reported error_code %u", result.result->error_code);
    return BT::NodeStatus::FAILURE;
  }

  // move_type "wait" is itself a SUCCEEDED goal, not a failure -- the tree's
  // MoveOrWaitForOpponent fallback is what decides whether to skip the move
  // subtree, by reading this same output port. Nothing here treats "wait"
  // specially.
  setOutput("move_type", result.result->move_type);
  setOutput("move_count", static_cast<int>(result.result->move_count));
  setOutput("hand_open_position", result.result->hand_open_position);
  setOutput("hand_close_position", result.result->hand_close_position);
  setOutput("move_from1", result.result->move_from1);
  setOutput("move_from_down1", result.result->move_from_down1);
  setOutput("move_to1", result.result->move_to1);
  setOutput("move_to_down1", result.result->move_to_down1);
  setOutput("move_from2", result.result->move_from2);
  setOutput("move_from_down2", result.result->move_from_down2);
  setOutput("move_to2", result.result->move_to2);
  setOutput("move_to_down2", result.result->move_to_down2);
  setOutput("move_from3", result.result->move_from3);
  setOutput("move_from_down3", result.result->move_from_down3);
  setOutput("move_to3", result.result->move_to3);
  setOutput("move_to_down3", result.result->move_to_down3);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DecisionActionClientNode::on_aborted()
{
  RCLCPP_ERROR(rclcpp::get_logger("DecisionActionClientNode"), "Decision action aborted");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DecisionActionClientNode::on_cancelled()
{
  RCLCPP_WARN(rclcpp::get_logger("DecisionActionClientNode"), "Decision action cancelled");
  return BT::NodeStatus::FAILURE;
}

}  // namespace bizon_behavior_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<bizon_behavior_clients::DecisionActionClientNode>(
        name, "decision_action", config);
    };
  factory.registerBuilder<bizon_behavior_clients::DecisionActionClientNode>(
    "MakeDecisionClient", builder);
}
