#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <utility>
#include <boost/filesystem.hpp>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "bizon_behavior_clients/behavior_tree_engine.hpp"

namespace fs = boost::filesystem;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto nh = std::make_shared<rclcpp::Node>("bizon_behavior_tree", node_options);

    BT::BehaviorTreeFactory factory_;
    std::vector<std::string> plugin_lib_names_ = {
        "wait_action_client_node",
        "arm_action_client_node",
        "run_until_success_node",
        "board_action_isaac_client_node",
        "make_decision_client_node",
        "condition_node",
        "foreach_node",
        "recovery_node"
    };

    RCLCPP_INFO(rclcpp::get_logger("main"), "Loading BT plugin libraries...");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Namespace: %s", nh->get_namespace());
    RCLCPP_INFO(rclcpp::get_logger("main"), "Number of plugins to load: %zu", plugin_lib_names_.size());

    fs::path bt_file = fs::path(ament_index_cpp::get_package_share_directory("bizon_behavior_clients")) / "behavior_trees" / "chess_game.xml";
    BT::Blackboard::Ptr blackboard;
    BT::Tree tree;

    RCLCPP_INFO(rclcpp::get_logger("main"), "Behavior Tree XML file: %s", bt_file.c_str());

    for (const auto &lib_name : plugin_lib_names_)
    {
        factory_.registerFromPlugin(BT::SharedLibrary::getOSName(lib_name));
        RCLCPP_INFO(rclcpp::get_logger("main"), "Registered plugin: %s", lib_name.c_str());
    }

    std::ifstream xml_file(bt_file.string());
    if (!xml_file.good())
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to open BT XML file: %s", bt_file.c_str());
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Creating Behavior Tree...");
    }

    auto xml_string = std::string(
        std::istreambuf_iterator<char>(xml_file),
        std::istreambuf_iterator<char>());

    blackboard = BT::Blackboard::create();

    std_msgs::msg::Int64 input_order;
    input_order.data = 10;

    blackboard->set<std_msgs::msg::Int64>("input_order", input_order);
    blackboard->set<rclcpp::Node::SharedPtr>("node", nh);
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(1000));

    std::string ns = nh->get_namespace();
    std::string player_side = (ns == "/bizon3" || ns == "bizon3") ? "black" : "white";
    blackboard->set<std::string>("player_side", player_side);
    RCLCPP_INFO(nh->get_logger(), "Namespace: %s, Player side: %s", ns.c_str(), player_side.c_str());

    // Recovery waypoint. The piece is released in place first (RecoverArm's
    // hand_only step, no joint target involved), then the arm lifts clear of
    // the board on this pose before homing. Values match the joint layout
    // used throughout the trees: {rev1, pris1, rev2, rev3}, with pris1 = 0.0
    // meaning fully retracted.
    blackboard->set<std::string>("recovery_lift_position", "-1.5807;0.0;1.5807;0.0");
    try
    {
        factory_.registerBehaviorTreeFromText(xml_string);
        tree = factory_.createTree("MainTree", blackboard);
    }
    catch (BT::RuntimeError &e)
    {
        std::cout << "Failed to create tree: " << e.what() << std::endl;
    }

    std::unique_ptr<BT::Groot2Publisher> publisher;
    if (player_side == "white")
    {
        publisher = std::make_unique<BT::Groot2Publisher>(tree, 1667);
    }

    std::unique_ptr<std::thread> thread_;
    rclcpp::Executor::SharedPtr executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    thread_ = std::make_unique<std::thread>(
        [&]()
        {
            executor_->add_node(nh);
            executor_->spin();
            executor_->remove_node(nh);
        });

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
        status = tree.tickWhileRunning();
    }
    std::cout << "Tree finished with: " << status << std::endl;

    std::cout << "Shutting down..." << std::endl;

    executor_->cancel();
    thread_->join();

    rclcpp::shutdown();

    return 0;
}