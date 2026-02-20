#include "bizon_behavior_clients/plugins/control/run_until_success_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bizon_behavior_tree
{
    RunUntilSuccessNode::RunUntilSuccessNode(
        const std::string &name,
        const BT::NodeConfiguration &conf)
        : BT::ControlNode::ControlNode(name, conf),
          current_child_idx_(0)
    {
    }

    BT::NodeStatus RunUntilSuccessNode::tick()
    {
        const unsigned children_count = children_nodes_.size();
        setStatus(BT::NodeStatus::RUNNING);

        while (current_child_idx_ < children_count)
        {
            TreeNode *child_node = children_nodes_[current_child_idx_];
            const BT::NodeStatus child_status = child_node->executeTick();

            if (current_child_idx_ == children_nodes_.size() - 1)
            {
                if (child_status == BT::NodeStatus::SUCCESS)
                {
                    // bizon_util::message_debug(rclcpp::get_logger("run_until_success_node"), {"RunUntilSuccessNode: Last child success"});
                    halt();
                    return BT::NodeStatus::SUCCESS;
                }
                else if (child_status == BT::NodeStatus::RUNNING)
                {
                    // bizon_util::message_debug(rclcpp::get_logger("run_until_success_node"), {"RunUntilSuccessNode: Last child running"});
                    return BT::NodeStatus::RUNNING;
                }
                else
                {
                    // bizon_util::message_debug(rclcpp::get_logger("run_until_success_node"), {"RunUntilSuccessNode: Last child failed"});
                    current_child_idx_ = 0;
                    halt();
                }
            }
            else
            {
                // bizon_util::message_debug(rclcpp::get_logger("run_until_success_node"), {"RunUntilSuccessNode: Not last child, child idx: ", std::to_string(current_child_idx_)});

                if (child_status == BT::NodeStatus::FAILURE)
                {
                    // bizon_util::message_debug(rclcpp::get_logger("run_until_success_node"), {"RunUntilSuccessNode: Child failed"});
                    // halt();
                    // return BT::NodeStatus::FAILURE;
                    current_child_idx_ = 0;
                    halt();
                    return BT::NodeStatus::RUNNING;
                }

                else if (child_status == BT::NodeStatus::SUCCESS)
                {
                    // bizon_util::message_debug(rclcpp::get_logger("run_until_success_node"), {"RunUntilSuccessNode: Child success"});
                    current_child_idx_++;
                }
                else
                {
                    return BT::NodeStatus::RUNNING;
                }
            }
        }

        halt();
        return BT::NodeStatus::FAILURE;
    }

    void RunUntilSuccessNode::halt()
    {
        // bizon_util::message_debug(rclcpp::get_logger("run_until_success_node"), {"RunUntilSuccessNode: halt"});
        ControlNode::halt();
        current_child_idx_ = 0;
    }

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bizon_behavior_tree::RunUntilSuccessNode>("RunUntilSuccess");
}