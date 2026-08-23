#include <gtest/gtest.h>

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "bizon_behavior_clients/plugins/control/condition_node.hpp"

using bizon_behavior_clients::ConditionNode;

namespace
{

// Mirrors the shape of PlayUntilGameOver in chess_game.xml: a guard that
// can succeed without any work being done (standing in for
// PlayMoveOrWaitForSystemActive short-circuiting on a paused system, before
// MakeDecisionClient ever writes move_type), followed by
// Inverter(ConditionNode CheckGameOver param1="{move_type}"
// param2="killking") the same way MainTree wires it.
BT::Tree buildTree(BT::Blackboard::Ptr blackboard)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ConditionNode>("ConditionNode");

  const std::string xml =
    R"(<root BTCPP_format="4"><BehaviorTree ID="T">)"
    R"(<Sequence>)"
    R"(<AlwaysSuccess/>)"
    R"(<Inverter><ConditionNode name="CheckGameOver" )"
    R"(param1="{move_type}" param2="killking"/></Inverter>)"
    R"(</Sequence></BehaviorTree></root>)";

  return factory.createTreeFromText(xml, blackboard);
}

}  // namespace

// Task 6 fix round 2 (Critical 1): PlayMoveOrWaitForSystemActive can
// short-circuit past RecoveryNode entirely -- on a PAUSE arriving before
// the first move, or simply because autostart has not yet activated
// behavior_server when the tree starts ticking -- so CheckGameOver can be
// reached with move_type never written by MakeDecisionClient at all.
// ConditionNode::tick() throws BT::RuntimeError when a required input port
// has no blackboard entry (see condition_node.cpp), and main.cpp ticks the
// tree with tickWhileRunning() outside any try/catch, so an unseeded
// move_type crashes the whole process on exactly the path the E-stop guard
// exists to make safe. main.cpp now seeds move_type to "" up front; these
// two tests document the crash this fixes and prove the fix actually
// prevents it.

TEST(MoveTypeSeededBeforeFirstDecision, ThrowsWhenMoveTypeWasNeverSeeded)
{
  auto blackboard = BT::Blackboard::create();
  // Deliberately not setting "move_type", reproducing main.cpp's state
  // before this fix, on the guard-skip path.
  auto tree = buildTree(blackboard);

  EXPECT_THROW(tree.tickWhileRunning(), BT::RuntimeError)
    << "this reproduces the crash: CheckGameOver's param1 port has nothing "
    "to bind to when move_type was never written";
}

TEST(MoveTypeSeededBeforeFirstDecision, SucceedsAsNotGameOverWhenSeededEmpty)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::string>("move_type", "");
  auto tree = buildTree(blackboard);

  BT::NodeStatus status;
  EXPECT_NO_THROW(status = tree.tickWhileRunning());
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS)
    << "\"\" must never equal CheckGameOver's param2 (\"killking\"), so "
    "Inverter reads it as \"game not over\" and the Sequence completes";
}
