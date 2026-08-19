#include "bizon_behavior_clients/plugins/control/condition_node.hpp"
#include "rclcpp/rclcpp.hpp"
namespace bizon_behavior_clients
{
    ConditionNode::ConditionNode(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("ConditionNode"),
            "ConditionNode created with name: %s",
            name.c_str());
    }

    BT::NodeStatus ConditionNode::tick()
    {
        if (!getInput("param1", param1))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("ConditionNode"),
                "Missing required input port [param1]");
            throw BT::RuntimeError("missing required input port [param1]");
        }
        if (!getInput("param2", param2))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("ConditionNode"),
                "Missing required input port [param2]");
            throw BT::RuntimeError("missing required input port [param2]");
        }

        // Implement your condition logic here using param1 and param2
        // For demonstration, let's assume the condition is true if param1 equals "check" and param2 is greater than 10

        if (param1 == param2)
        {
            return BT::NodeStatus::SUCCESS;
        }else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bizon_behavior_clients::ConditionNode>("ConditionNode");
}