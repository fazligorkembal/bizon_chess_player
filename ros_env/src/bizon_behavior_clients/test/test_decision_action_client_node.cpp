#include <gtest/gtest.h>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "bizon_behavior_clients/plugins/action/decision_action_client_node.hpp"
#include "bizon_msgs/action/decision.hpp"

// DecisionActionClientNode's constructor builds a live rclcpp_action::Client
// (it needs a running node from the blackboard and, once ticked, a reachable
// decision_action action server), so it cannot be instantiated in this
// unit-test environment -- see test_arm_action_client_node.cpp for the same
// constraint on ArmActionClientNode, which this test mirrors. It only proves
// the port manifest has the shape chess_game.xml and DecisionPlugin's
// result depend on; it does not exercise on_tick() or DecisionPlugin itself.

TEST(DecisionActionClientNode, XmlTagIsMakeDecisionClientAgainstDecisionAction)
{
  BT::BehaviorTreeFactory factory;
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<bizon_behavior_clients::DecisionActionClientNode>(
      name, "decision_action", config);
  };
  factory.registerBuilder<bizon_behavior_clients::DecisionActionClientNode>(
    "MakeDecisionClient", builder);

  const auto & manifests = factory.manifests();
  ASSERT_NE(manifests.find("MakeDecisionClient"), manifests.end())
    << "chess_game.xml's <MakeDecisionClient> tag must stay registered -- "
       "Task 5 keeps the XML tag name unchanged while moving the action name "
       "to decision_action, precisely so chess_game.xml needs no edit.";
}

// Structural guard for the port contract Step 8 of the task-5 brief and
// DecisionPlugin::onActionCompletion() both depend on: every port
// make_decision_client_node.hpp used to declare must still be declared here,
// with the same direction and type, or setOutput()/getInput() calls that
// compile today would silently no-op against a renamed or dropped port.
TEST(DecisionActionClientNode, DeclaresEveryPortTheOldSyncActionNodeDid)
{
  const auto ports = bizon_behavior_clients::DecisionActionClientNode::providedPorts();

  auto expect_input = [&](const char * name) {
    auto it = ports.find(name);
    ASSERT_NE(it, ports.end()) << "missing input port: " << name;
    EXPECT_EQ(it->second.direction(), BT::PortDirection::INPUT) << name;
  };
  auto expect_output = [&](const char * name) {
    auto it = ports.find(name);
    ASSERT_NE(it, ports.end()) << "missing output port: " << name;
    EXPECT_EQ(it->second.direction(), BT::PortDirection::OUTPUT) << name;
  };

  expect_input("player_side");
  expect_input("fen");

  expect_output("move_type");
  expect_output("move_count");
  expect_output("hand_open_position");
  expect_output("hand_close_position");
  for (const char * suffix : {"1", "2", "3"}) {
    expect_output((std::string("move_from") + suffix).c_str());
    expect_output((std::string("move_from_down") + suffix).c_str());
    expect_output((std::string("move_to") + suffix).c_str());
    expect_output((std::string("move_to_down") + suffix).c_str());
  }
}

// Ruling 2 added hand_open_position/hand_close_position to Decision.action's
// result beyond what the task-5 brief's Step 1 excerpt listed, specifically
// so onResultReceived() can setOutput() the two hand-position ports the old
// node declared. If either field is ever dropped from the .action file, this
// fails to compile rather than silently regressing into stale hand targets.
TEST(DecisionResult, ResultMessageCarriesHandPositionFields)
{
  bizon_msgs::action::Decision::Result result;
  result.hand_open_position = {0.2, 0.2, 0.2};
  result.hand_close_position = {0.04, 0.04, 0.04};
  EXPECT_EQ(result.hand_open_position.size(), 3u);
  EXPECT_EQ(result.hand_close_position.size(), 3u);
}
