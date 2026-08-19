#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "bizon_behavior_clients/plugins/control/recovery_node.hpp"

using bizon_behavior_clients::RecoveryNode;

namespace {

/// Records every tick so the test can assert on ordering, and returns a
/// scripted sequence of results.
class ScriptedAction : public BT::SyncActionNode
{
public:
  ScriptedAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    trace->push_back(name());
    if (call_index_ < results.size()) {
      return results[call_index_++];
    }
    return results.empty() ? BT::NodeStatus::SUCCESS : results.back();
  }

  std::vector<BT::NodeStatus> results;
  std::vector<std::string> * trace{nullptr};

private:
  size_t call_index_{0};
};

struct Fixture
{
  BT::BehaviorTreeFactory factory;
  std::vector<std::string> trace;
  ScriptedAction * work{nullptr};
  ScriptedAction * recovery{nullptr};
  BT::Tree tree;

  void build(const std::vector<BT::NodeStatus> & work_results,
             const std::vector<BT::NodeStatus> & recovery_results,
             int retries)
  {
    factory.registerNodeType<RecoveryNode>("RecoveryNode");
    factory.registerNodeType<ScriptedAction>("Work");
    factory.registerNodeType<ScriptedAction>("Recovery");

    const std::string xml =
      R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">)"
      R"(<RecoveryNode number_of_retries=")" + std::to_string(retries) + R"(">)"
      R"(<Work name="work"/><Recovery name="recovery"/>)"
      R"(</RecoveryNode></BehaviorTree></root>)";

    tree = factory.createTreeFromText(xml);
    for (auto & subtree : tree.subtrees) {
      for (auto & node : subtree->nodes) {
        if (auto * a = dynamic_cast<ScriptedAction *>(node.get())) {
          a->trace = &trace;
          if (a->name() == "work") { a->results = work_results; work = a; }
          else { a->results = recovery_results; recovery = a; }
        }
      }
    }
  }
};

}  // namespace

TEST(RecoveryNode, ReturnsSuccessWithoutRunningRecovery)
{
  Fixture f;
  f.build({BT::NodeStatus::SUCCESS}, {BT::NodeStatus::SUCCESS}, 3);
  EXPECT_EQ(f.tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.trace, (std::vector<std::string>{"work"}));
}

// The core contract: the work branch is never re-entered until recovery ran.
TEST(RecoveryNode, RunsRecoveryBetweenWorkAttempts)
{
  Fixture f;
  f.build({BT::NodeStatus::FAILURE, BT::NodeStatus::SUCCESS}, {BT::NodeStatus::SUCCESS}, 3);
  EXPECT_EQ(f.tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.trace, (std::vector<std::string>{"work", "recovery", "work"}));
}

TEST(RecoveryNode, GivesUpAfterRetriesExhausted)
{
  Fixture f;
  f.build({BT::NodeStatus::FAILURE}, {BT::NodeStatus::SUCCESS}, 2);
  EXPECT_EQ(f.tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.trace,
    (std::vector<std::string>{"work", "recovery", "work", "recovery", "work"}));
}

// A failed recovery must stop the tree, not loop forever.
TEST(RecoveryNode, FailsImmediatelyWhenRecoveryFails)
{
  Fixture f;
  f.build({BT::NodeStatus::FAILURE}, {BT::NodeStatus::FAILURE}, 3);
  EXPECT_EQ(f.tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.trace, (std::vector<std::string>{"work", "recovery"}));
}
