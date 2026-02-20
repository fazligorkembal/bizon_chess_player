#include "bizon_behavior_clients/plugins/action/arm_action_client_node.hpp"

namespace bizon_behavior_clients
{
    ArmActionClientNode::ArmActionClientNode(
        const std::string &name,
        const BT::NodeConfiguration &conf)
        : BT::StatefulActionNode(name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        options_arm_ = new moveit::planning_interface::MoveGroupInterface::Options(
            "arm_group",
            "robot_description",
            node_->get_namespace());

        options_hand_ = new moveit::planning_interface::MoveGroupInterface::Options(
            "hand_group",
            "robot_description",
            node_->get_namespace());

        move_group_arm_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, *options_arm_);
        move_group_hand_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, *options_hand_);

        move_group_arm_ptr_->setPlanningTime(15.0);
        move_group_hand_ptr_->setPlanningTime(15.0);

        RCLCPP_INFO(node_->get_logger(), "ArmActionClientNode created");

        auto arm_joint_names = move_group_arm_ptr_->getJointNames();
        RCLCPP_INFO(node_->get_logger(), "Arm joints:");
        for (const auto &joint_name : arm_joint_names)
        {
            RCLCPP_INFO(node_->get_logger(), "  - %s", joint_name.c_str());
        }

        auto hand_joint_names = move_group_hand_ptr_->getJointNames();
        RCLCPP_INFO(node_->get_logger(), "Hand joints:");
        for (const auto &joint_name : hand_joint_names)
        {
            RCLCPP_INFO(node_->get_logger(), "  - %s", joint_name.c_str());
        }
    }

    BT::NodeStatus ArmActionClientNode::onStart()
    {

        target_joint_positions_.clear();
        if (!getInput<std::string>("player_side", player_side_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input port [player_side]");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput<std::vector<double>>("target_joint_positions", target_joint_positions_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input port [target_joint_positions]");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput<std::vector<double>>("target_hand_position", target_hand_position_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input port [target_hand_position]");
            return BT::NodeStatus::FAILURE;
        }

        auto measured_joint_values = move_group_arm_ptr_->getCurrentJointValues();
        if (measured_joint_values.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get current joint values");
            return BT::NodeStatus::FAILURE;
        }

        move_group_arm_ptr_->setJointValueTarget(target_joint_positions_);
        move_group_hand_ptr_->setJointValueTarget(target_hand_position_);

        // Start async movement
        move_future_arm_ = std::async(std::launch::async, [this]()
        {
            return move_group_arm_ptr_->move();
        });

        move_future_hand_ = std::async(std::launch::async, [this]()
        {
            return move_group_hand_ptr_->move();
        });

        move_started_ = true;
        RCLCPP_INFO(node_->get_logger(), "Arm movement started (async)");

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ArmActionClientNode::onRunning()
    {
        if (!move_started_)
        {
            return BT::NodeStatus::FAILURE;
        }

        // Check if async move is complete
        if (move_future_arm_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready &&
            move_future_hand_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
        {
            auto result_arm = move_future_arm_.get();
            auto result_hand = move_future_hand_.get();
            move_started_ = false;

            if (result_arm == moveit::core::MoveItErrorCode::SUCCESS && result_hand == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(node_->get_logger(), "Arm movement completed successfully");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Arm movement failed with error code: %d", result_arm.val);
                RCLCPP_ERROR(node_->get_logger(), "Hand movement failed with error code: %d", result_hand.val);
                return BT::NodeStatus::FAILURE;
            }
        }

        // Still running
        RCLCPP_DEBUG(node_->get_logger(), "Arm movement in progress...");
        return BT::NodeStatus::RUNNING;
    }

    void ArmActionClientNode::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(), "ArmActionClient halted - stopping movement");
        if (move_started_)
        {
            move_group_arm_ptr_->stop();
            move_started_ = false;
        }
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bizon_behavior_clients::ArmActionClientNode>("ArmActionClient");
}