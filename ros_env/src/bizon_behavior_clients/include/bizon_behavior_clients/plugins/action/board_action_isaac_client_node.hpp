#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__BOARD_ACTION_ISAAC_CLIENT_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__BOARD_ACTION_ISAAC_CLIENT_NODE_HPP_

#include <string>
#include <bizon_behavior_clients/bt_action_client_node.hpp>
#include "bizon_msgs/action/board.hpp"
#include "builtin_interfaces/msg/duration.hpp"

namespace bizon_behavior_clients
{
class BoardActionIsaacClientNode : public BtActionClientNode<bizon_msgs::action::Board>
{
public:
  BoardActionIsaacClientNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus onResultReceived(
    const typename rclcpp_action::ClientGoalHandle<bizon_msgs::action::Board>::WrappedResult & result)
  override;

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<unsigned int>("sec", 0, "seconds for timeout"),
        BT::InputPort<unsigned int>("nanosec", 0, "nanoseconds for timeout"),
        BT::InputPort<std::string>("player_side", "white", "player side (white or black)"),
        BT::OutputPort<std::string>("fen", "resulting FEN string"),
      });
  }
};
}

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__BOARD_ACTION_ISAAC_CLIENT_NODE_HPP_
