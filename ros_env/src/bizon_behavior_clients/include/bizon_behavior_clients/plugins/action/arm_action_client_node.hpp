#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_

#include <string>
#include <vector>

#include "bizon_behavior_clients/bt_action_client_node.hpp"
#include "bizon_msgs/action/arm.hpp"

namespace bizon_behavior_clients
{
class ArmActionClientNode : public BtActionClientNode<bizon_msgs::action::Arm>
{
public:
  ArmActionClientNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus onResultReceived(
    const rclcpp_action::ClientGoalHandle<bizon_msgs::action::Arm>::WrappedResult & result) override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("player_side", "white or black"),
      BT::InputPort<std::vector<double>>("target_joint_positions", "Arm joint targets"),
      BT::InputPort<std::vector<double>>("target_hand_position", "Gripper finger targets"),
      // hand_only lets a caller (e.g. the recovery subtree) command only the
      // gripper, without ever reading or acting on target_joint_positions.
      // This is the only way to open the gripper without first completing an
      // arm move: moving first would drag a held piece across the board.
      BT::InputPort<bool>("hand_only", false,
        "Command only the gripper, leaving the arm stationary"),
      // Task 7 (F6): refresh the planning scene from the board before
      // planning this goal, and open a path to the square being descended
      // onto. Both default to empty, meaning "leave the scene alone" --
      // e.g. the home-position and RecoverArm goals, which carry no board
      // context and would otherwise clear the whole scene on every tick.
      BT::InputPort<std::string>(
        "board_fen", "", "Current board FEN; refreshes planning-scene collision objects"),
      BT::InputPort<std::string>(
        "target_square", "", "Square this goal descends onto; its collision object is removed"),
    });
  }
};
}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_
