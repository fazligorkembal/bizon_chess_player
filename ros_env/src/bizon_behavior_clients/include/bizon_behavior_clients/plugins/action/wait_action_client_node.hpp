#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_

#include <string>
#include <bizon_behavior_clients/bt_action_client_node.hpp>
#include "bizon_msgs/action/wait.hpp"
#include "builtin_interfaces/msg/duration.hpp"

namespace bizon_behavior_clients
{
class WaitActionClientNode : public BtActionClientNode<bizon_msgs::action::Wait>
{
public:
  WaitActionClientNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus onResultReceived(
    const typename rclcpp_action::ClientGoalHandle<bizon_msgs::action::Wait>::WrappedResult & result)
  override;

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<unsigned int>("sec", "seconds to wait"),
        BT::InputPort<unsigned int>("nanosec", "nanoseconds to wait"),
      });
  }
};
}

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__WAIT_ACTION_CLIENT_NODE_HPP_
