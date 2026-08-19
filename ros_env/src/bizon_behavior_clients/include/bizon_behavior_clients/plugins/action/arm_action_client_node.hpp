#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_

#include <string>
#include <future>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include <moveit/move_group_interface/move_group_interface.h>

namespace bizon_behavior_clients
{
    class ArmActionClientNode : public BT::StatefulActionNode
    {
    public:
        ArmActionClientNode(const std::string &name, const BT::NodeConfiguration &conf);

        // StatefulActionNode interface
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("player_side", "Player side, white or black"),
                BT::InputPort<std::vector<double>>("target_joint_positions", "Target joint positions for the arm"),
                BT::InputPort<std::vector<double>>("target_hand_position", "Target joint positions for the hand"),
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        moveit::planning_interface::MoveGroupInterface::Options *options_arm_ = nullptr;
        moveit::planning_interface::MoveGroupInterface::Options *options_hand_ = nullptr;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_ptr_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_hand_ptr_;

        // Async movement tracking
        std::future<moveit::core::MoveItErrorCode> move_future_arm_;
        std::future<moveit::core::MoveItErrorCode> move_future_hand_;
        enum class Phase
        {
          IDLE,
          ARM_MOVING,
          HAND_MOVING,
        };
        Phase phase_{Phase::IDLE};

        std::string player_side_;
        std::vector<double> target_joint_positions_;
        std::vector<double> target_hand_position_;
    };
}

#endif // BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_