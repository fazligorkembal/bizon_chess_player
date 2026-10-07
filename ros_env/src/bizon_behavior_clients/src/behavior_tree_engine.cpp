#include "bizon_behavior_clients/behavior_tree_engine.hpp"
#include "behaviortree_cpp/utils/shared_library.h"

namespace bizon_behavior_clients
{
BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
    RCLCPP_INFO(
      rclcpp::get_logger("BehaviorTreeEngine"), "Loaded BT plugin library: %s",
      p.c_str());
  }
}

BtStatus BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      if (cancelRequested()) {
        tree->rootNode()->haltNode();
        return BtStatus::CANCELED;
      }
      result = tree->tickOnce();
      onLoop();
      if (!loopRate.sleep()) {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "BehaviorTreeEngine"), "Sleep interrupted in BT execution loop");
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "BehaviorTreeEngine"), "Exception during BT execution: %s", e.what());
    return BtStatus::FAILED;
  }
  return result == BT::NodeStatus::SUCCESS ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree BehaviorTreeEngine::createTreeFromText(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(file_path, blackboard);
}

void BehaviorTreeEngine::haltAllActions(BT::Tree & tree)
{
  tree.haltTree();
}


}
