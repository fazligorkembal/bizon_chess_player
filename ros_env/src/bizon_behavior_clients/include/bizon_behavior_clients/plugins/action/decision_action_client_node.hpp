#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__DECISION_ACTION_CLIENT_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__DECISION_ACTION_CLIENT_NODE_HPP_

#include <string>
#include <vector>

#include "bizon_behavior_clients/bt_action_client_node.hpp"
#include "bizon_msgs/action/decision.hpp"

namespace bizon_behavior_clients
{
/// Thin BT client for decision_action (bizon_behaviors::DecisionPlugin,
/// bizon_behavior_servers). Replaces the old MakeDecisionNode
/// (make_decision_client_node.*, deleted in the same change that added this
/// file), which ran the whole decision -- including forking and driving
/// Stockfish directly -- inline in a SyncActionNode::tick(), blocking the
/// tree for as long as the engine took (unbounded; see F4). The XML tag
/// name is unchanged (MakeDecisionClient) so chess_game.xml needs no edit.
class DecisionActionClientNode : public BtActionClientNode<bizon_msgs::action::Decision>
{
public:
  DecisionActionClientNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus onResultReceived(
    const rclcpp_action::ClientGoalHandle<bizon_msgs::action::Decision>::WrappedResult & result)
  override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("player_side", "white or black"),
      BT::InputPort<std::string>("fen", "Current board FEN string"),
      BT::OutputPort<std::string>(
        "move_type", "Type of the move: straight, capture, en_passant, castle, promotion, "
        "promotion_capture, save, killking, wait"),
      BT::OutputPort<int>("move_count", "Number of moves made"),
      BT::OutputPort<std::vector<double>>("hand_open_position", "Hand open position"),
      BT::OutputPort<std::vector<double>>("hand_close_position", "Hand close position"),
      BT::OutputPort<std::vector<double>>("move_from1", "The move's source joint angles"),
      BT::OutputPort<std::vector<double>>(
        "move_from_down1", "The move's source with downward offset joint angles"),
      BT::OutputPort<std::vector<double>>("move_to1", "The move's destination joint angles"),
      BT::OutputPort<std::vector<double>>(
        "move_to_down1", "The move's destination with downward offset joint angles"),
      BT::OutputPort<std::vector<double>>(
        "move_from2", "The move's source joint angles for second piece in case of castling move"),
      BT::OutputPort<std::vector<double>>(
        "move_from_down2",
        "The move's source with downward offset joint angles for second piece in case of "
        "castling move"),
      BT::OutputPort<std::vector<double>>(
        "move_to2", "The move's destination joint angles for second piece in case of castling move"),
      BT::OutputPort<std::vector<double>>(
        "move_to_down2",
        "The move's destination with downward offset joint angles for second piece in case of "
        "castling move"),
      BT::OutputPort<std::vector<double>>(
        "move_from3", "The move's source joint angles for third piece in case of promotion capture"),
      BT::OutputPort<std::vector<double>>(
        "move_from_down3",
        "The move's source with downward offset joint angles for third piece in case of "
        "promotion capture"),
      BT::OutputPort<std::vector<double>>(
        "move_to3", "The move's destination joint angles for third piece in case of promotion capture"),
      BT::OutputPort<std::vector<double>>(
        "move_to_down3",
        "The move's destination with downward offset joint angles for third piece in case of "
        "promotion capture"),
    });
  }
};
}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__DECISION_ACTION_CLIENT_NODE_HPP_
