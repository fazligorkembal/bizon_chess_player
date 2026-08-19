#include <gtest/gtest.h>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "bizon_behavior_clients/plugins/action/arm_action_client_node.hpp"

// ArmActionClientNode's constructor builds a live moveit::MoveGroupInterface
// (it needs a running node with a resolved robot_description parameter and a
// reachable move_group action server), so it cannot be instantiated in this
// unit-test environment. BT::BehaviorTreeFactory::registerNodeType<T>() only
// calls T::providedPorts() to build a manifest and stores a builder closure
// -- it never constructs T -- so the port contract itself is testable in
// isolation. This is the "port/state-machine level" coverage for the
// hand_only safety port: it guards against the port being renamed, dropped,
// or given the wrong type/default, none of which the compiler would catch
// (BT ports are resolved by string name at tree-parse time).
//
// It does NOT exercise onStart()/onRunning() -- i.e. it cannot prove that
// hand_only=true actually skips move_group_arm_ptr_->move(). That requires a
// live MoveGroupInterface (a real node, robot_description, and move_group
// action server), which is unavailable here.

TEST(ArmActionClientNode, HandOnlyPortIsDeclaredAsOptionalBoolDefaultingFalse)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bizon_behavior_clients::ArmActionClientNode>("ArmActionClient");

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
