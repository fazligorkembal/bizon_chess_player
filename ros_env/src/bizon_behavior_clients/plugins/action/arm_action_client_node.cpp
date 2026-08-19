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

        // hand_only lets a caller (e.g. the recovery subtree) command only the
        // gripper, without ever reading or acting on target_joint_positions.
        // This is the only way to open the gripper without first completing an
        // arm move: see the "Arm first, gripper afterwards" note below.
        bool hand_only = false;
        getInput<bool>("hand_only", hand_only);

        if (!getInput<std::vector<double>>("target_hand_position", target_hand_position_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input port [target_hand_position]");
            return BT::NodeStatus::FAILURE;
        }

        if (hand_only)
        {
            move_group_hand_ptr_->setJointValueTarget(target_hand_position_);
            move_future_hand_ = std::async(std::launch::async, [this]()
            {
                return move_group_hand_ptr_->move();
            });

            phase_ = Phase::HAND_MOVING;
            RCLCPP_INFO(node_->get_logger(), "Hand-only movement started (arm left stationary)");

            return BT::NodeStatus::RUNNING;
        }

        if (!getInput<std::vector<double>>("target_joint_positions", target_joint_positions_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input port [target_joint_positions]");
            return BT::NodeStatus::FAILURE;
        }

        auto measured_joint_values = move_group_arm_ptr_->getCurrentJointValues();
        if (measured_joint_values.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get current joint values");
            return BT::NodeStatus::FAILURE;
        }

        move_group_arm_ptr_->setJointValueTarget(target_joint_positions_);

        // Arm first, gripper afterwards. Running them concurrently means a
        // settling correction on the arm can execute while the fingers are
        // closing, which knocks the piece over on real hardware.
        move_future_arm_ = std::async(std::launch::async, [this]()
        {
            return move_group_arm_ptr_->move();
        });

        phase_ = Phase::ARM_MOVING;
        RCLCPP_INFO(node_->get_logger(), "Arm movement started");

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ArmActionClientNode::onRunning()
    {
        if (phase_ == Phase::ARM_MOVING)
        {
            if (move_future_arm_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
            {
                return BT::NodeStatus::RUNNING;
            }

            const auto result_arm = move_future_arm_.get();
            if (result_arm != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Arm movement failed with error code: %d", result_arm.val);
                phase_ = Phase::IDLE;
                return BT::NodeStatus::FAILURE;
            }

            // Arm has settled; only now command the gripper.
            move_group_hand_ptr_->setJointValueTarget(target_hand_position_);
            move_future_hand_ = std::async(std::launch::async, [this]()
            {
                return move_group_hand_ptr_->move();
            });
            phase_ = Phase::HAND_MOVING;
            return BT::NodeStatus::RUNNING;
        }

        if (phase_ == Phase::HAND_MOVING)
        {
            if (move_future_hand_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
            {
                return BT::NodeStatus::RUNNING;
            }

            const auto result_hand = move_future_hand_.get();
            phase_ = Phase::IDLE;
            if (result_hand != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Hand movement failed with error code: %d", result_hand.val);
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_->get_logger(), "Arm and hand movement completed successfully");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

    void ArmActionClientNode::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(), "ArmActionClient halted - stopping movement");
        if (phase_ == Phase::ARM_MOVING)
        {
            move_group_arm_ptr_->stop();
        }
        else if (phase_ == Phase::HAND_MOVING)
        {
            move_group_hand_ptr_->stop();
        }
        phase_ = Phase::IDLE;
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bizon_behavior_clients::ArmActionClientNode>("ArmActionClient");
}