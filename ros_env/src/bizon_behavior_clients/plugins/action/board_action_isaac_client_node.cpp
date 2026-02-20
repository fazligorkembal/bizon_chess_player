#include "bizon_behavior_clients/plugins/action/board_action_isaac_client_node.hpp"

namespace bizon_behavior_clients
{
    BoardActionIsaacClientNode::BoardActionIsaacClientNode(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BtActionClientNode<bizon_msgs::action::Board>(xml_tag_name, action_name, conf)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("BoardActionIsaacClientNode"),
            "BoardActionIsaacClientNode created for action: %s",
            action_name.c_str());
    }

    void BoardActionIsaacClientNode::on_tick()
    {
        unsigned int sec = 0, nanosec = 0;
        getInput("sec", sec);
        getInput("nanosec", nanosec);

        std::string player_side;
        getInput("player_side", player_side);

        goal_.time.sec = sec;
        goal_.time.nanosec = nanosec;
        goal_.player_side = player_side;
    }

    BT::NodeStatus BoardActionIsaacClientNode::on_success()
    {
        RCLCPP_INFO(
            rclcpp::get_logger("BoardActionIsaacClientNode"),
            "Board action succeeded");
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus BoardActionIsaacClientNode::on_aborted()
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("BoardActionIsaacClientNode"),
            "Board action aborted");
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus BoardActionIsaacClientNode::on_cancelled()
    {
        RCLCPP_WARN(
            rclcpp::get_logger("BoardActionIsaacClientNode"),
            "Board action canceled");
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus BoardActionIsaacClientNode::onResultReceived(
        const typename rclcpp_action::ClientGoalHandle<bizon_msgs::action::Board>::WrappedResult &result)
    {
        setOutput("fen", result.result->fen);
        RCLCPP_INFO(
            rclcpp::get_logger("BoardActionIsaacClientNode"),
            "fen-camera-detected: '%s'",
            result.result->fen.c_str());
        return BT::NodeStatus::SUCCESS;
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<bizon_behavior_clients::BoardActionIsaacClientNode>(
            name, "board_action", config);
    };
    factory.registerBuilder<bizon_behavior_clients::BoardActionIsaacClientNode>(
        "BoardActionIsaacClient", builder);
}
