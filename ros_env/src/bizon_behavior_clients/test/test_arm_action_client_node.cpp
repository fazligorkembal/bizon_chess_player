#include <gtest/gtest.h>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "bizon_behavior_clients/plugins/action/arm_action_client_node.hpp"
#include "bizon_msgs/action/arm.hpp"

// ArmActionClientNode's constructor builds a live rclcpp_action::Client (it
// needs a running node from the blackboard and, once ticked, a reachable
// arm_action action server), so it cannot be instantiated in this unit-test
// environment. It also no longer has a (name, config) constructor -- as a
// BtActionClientNode<bizon_msgs::action::Arm> subclass it takes
// (xml_tag_name, action_name, conf), so it must be registered with a
// BT::NodeBuilder rather than BehaviorTreeFactory::registerNodeType<T>(),
// exactly as production code does in arm_action_client_node.cpp's
// BT_REGISTER_NODES block. registerBuilder<T>() only calls
// T::providedPorts() to build a manifest and stores the builder closure for
// later -- it never invokes the builder -- so the port contract itself is
// testable in isolation. This is the "port/state-machine level" coverage for
// the hand_only safety port: it guards against the port being renamed,
// dropped, or given the wrong type/default, none of which the compiler would
// catch (BT ports are resolved by string name at tree-parse time).
//
// It does NOT exercise on_tick() or the ArmPlugin server-side behavior --
// i.e. it cannot prove that hand_only=true actually skips the arm. That
// requires a live MoveGroupInterface (a real node, robot_description, and
// move_group action server), which is unavailable here.

TEST(ArmActionClientNode, HandOnlyPortIsDeclaredAsOptionalBoolDefaultingFalse)
{
  BT::BehaviorTreeFactory factory;
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<bizon_behavior_clients::ArmActionClientNode>(
      name, "arm_action", config);
  };
  factory.registerBuilder<bizon_behavior_clients::ArmActionClientNode>("ArmActionClient", builder);

  const auto & manifests = factory.manifests();
  auto it = manifests.find("ArmActionClient");
  ASSERT_NE(it, manifests.end());

  const auto & ports = it->second.ports;
  auto port_it = ports.find("hand_only");
  ASSERT_NE(port_it, ports.end()) << "hand_only port must exist on ArmActionClient";

  const BT::PortInfo & port = port_it->second;
  EXPECT_EQ(port.direction(), BT::PortDirection::INPUT);
  EXPECT_EQ(port.type(), typeid(bool));
  EXPECT_EQ(port.defaultValueString(), "false");
}

// Structural guard for the highest-risk part of the arm_action refactor: if
// hand_only is ever dropped from Arm.action's goal definition, this fails to
// compile rather than silently regressing into the arm dragging a held piece
// across the board on the next recovery. See ArmPlugin::onRun and
// ArmActionClientNode::on_tick for the two places that read/write it.
TEST(ArmGoal, GoalMessageCarriesHandOnlyField)
{
  bizon_msgs::action::Arm::Goal goal;
  goal.hand_only = true;
  EXPECT_TRUE(goal.hand_only);
  goal.hand_only = false;
  EXPECT_FALSE(goal.hand_only);
}
