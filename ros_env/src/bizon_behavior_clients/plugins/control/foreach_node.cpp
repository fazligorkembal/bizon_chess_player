#include "bizon_behavior_clients/plugins/control/foreach_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bizon_behavior_clients
{
    ForeachNode::ForeachNode(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::ControlNode(name, config)
    {
    }

    BT::NodeStatus ForeachNode::tick()
    {
        if (!getInput("move_count", count))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("ForeachNode"),
                "Missing required input port [move_count]");
            throw BT::RuntimeError("missing required input port [move_count]");
        }

        if (count <= 0)
        {
            return BT::NodeStatus::FAILURE;
        }

        // Set up the current iteration's outputs only when the child is not already running
        if (children_nodes_[0]->status() != BT::NodeStatus::RUNNING)
        {
            // Reset child if it completed a previous iteration
            if (children_nodes_[0]->status() != BT::NodeStatus::IDLE)
            {
                haltChild(0);
            }

            const int i = current_index_ + 1;
            std::vector<double> from_val, from_down_val, to_val, to_down_val;
            std::string box_from_val, box_to_val;
            const std::string idx = std::to_string(i);

            if (!getInput("move_from" + idx, from_val))
                throw BT::RuntimeError("missing required input port [move_from" + idx + "]");
            if (!getInput("move_from_down" + idx, from_down_val))
                throw BT::RuntimeError("missing required input port [move_from_down" + idx + "]");
            if (!getInput("move_to" + idx, to_val))
                throw BT::RuntimeError("missing required input port [move_to" + idx + "]");
            if (!getInput("move_to_down" + idx, to_down_val))
                throw BT::RuntimeError("missing required input port [move_to_down" + idx + "]");
            if (!getInput("box_from" + idx, box_from_val))
                throw BT::RuntimeError("missing required input port [box_from" + idx + "]");
            if (!getInput("box_to" + idx, box_to_val))
                throw BT::RuntimeError("missing required input port [box_to" + idx + "]");

            setOutput("move_from", from_val);
            setOutput("move_from_down", from_down_val);
            setOutput("move_to", to_val);
            setOutput("move_to_down", to_down_val);
            setOutput("box_from", box_from_val);
            setOutput("box_to", box_to_val);
        }

        auto child_status = children_nodes_[0]->executeTick();

        if (child_status == BT::NodeStatus::RUNNING)
        {
            return BT::NodeStatus::RUNNING;
        }

        if (child_status == BT::NodeStatus::FAILURE)
        {
            current_index_ = 0;
            return BT::NodeStatus::FAILURE;
        }

        // Child succeeded — advance to next iteration
        current_index_++;
        if (current_index_ >= count)
        {
            current_index_ = 0;
            return BT::NodeStatus::SUCCESS;
        }

        // More iterations remain; return RUNNING so we are ticked again
        return BT::NodeStatus::RUNNING;
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bizon_behavior_clients::ForeachNode>("Foreach");
}