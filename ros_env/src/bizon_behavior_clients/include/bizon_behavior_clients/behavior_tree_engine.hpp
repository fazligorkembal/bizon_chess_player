#ifndef BIZON_BEHAVIOR_TREE_BEHAVIOR_TREE_ENGINE_HPP
#define BIZON_BEHAVIOR_TREE_BEHAVIOR_TREE_ENGINE_HPP

#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "rclcpp/rclcpp.hpp"

namespace bizon_behavior_clients
{
    /**
     * @enum bizon_behavior_tree::BtStatus
     * @brief An enum class representing BT execution status
     */
    enum class BtStatus
    {
        SUCCEEDED,
        FAILED,
        CANCELED
    };

    /**
     * @class bizon_behavior_tree::BehaviorTreeEngine
     * @brief A class to create and handle behavior trees
     */
    class BehaviorTreeEngine
    {
        /**
         * @brief A constructor for bizon_behavior_tree::BehaviorTreeEngine
         * @param plugin_libraries vector of BT plugin library names to load
         */
        explicit BehaviorTreeEngine(const std::vector<std::string> &plugin_libraries);
        virtual ~BehaviorTreeEngine() {}

        /**
         * @brief Function to execute a BT at a specific rate
         * @param tree BT to execute
         * @param onLoop Function to execute on each iteration of BT execution
         * @param cancelRequested Function to check if cancel was requested during BT execution
         * @param loopTimeout Time period for each iteration of BT execution
         * @return bizon_behavior_tree::BtStatus Status of BT execution
         */
        BtStatus run(
            BT::Tree *tree,
            std::function<void()> onLoop,
            std::function<bool()> cancelRequested,
            std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

        /**
         * @brief Function to create a BT from a XML string
         * @param xml_string XML string representing BT
         * @param blackboard Blackboard for BT
         * @return BT::Tree Created behavior tree
         */
        BT::Tree createTreeFromText(
            const std::string &xml_string,
            BT::Blackboard::Ptr blackboard);

        /**
         * @brief Function to create a BT from an XML file
         * @param file_path Path to BT XML file
         * @param blackboard Blackboard for BT
         * @return BT::Tree Created behavior tree
         */
        BT::Tree createTreeFromFile(
            const std::string &file_path,
            BT::Blackboard::Ptr blackboard);

        /**
         * @brief Function to explicitly reset all BT nodes to initial state
         * @param tree Tree to halt
         */
        void haltAllActions(BT::Tree &tree);

    protected:
        // The factory that will be used to dynamically construct the behavior tree
        BT::BehaviorTreeFactory factory_;
    };
}

#endif // BIZON_BEHAVIOR_TREE_BEHAVIOR_TREE_ENGINE_HPP