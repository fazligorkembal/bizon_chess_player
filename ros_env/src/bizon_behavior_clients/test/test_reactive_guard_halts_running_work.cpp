#include <gtest/gtest.h>

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"

namespace
{

// Mirrors IsSystemActiveNode's polarity (SUCCESS while active, FAILURE
// while not) without needing a live lifecycle manager: reads a plain bool
// off the blackboard so the test can flip it between ticks.
class ActiveCondition : public BT::ConditionNode
{
public:
  ActiveCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("active")};
  }

  BT::NodeStatus tick() override
  {
    bool active = true;
    getInput("active", active);
    return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// Stands in for RecoveryNode mid-move: returns RUNNING on every tick until
// halted, and records whether halt() was ever called and how many times it
// was ticked. halt() is the only way a control node can interrupt an
// in-flight async child in BT.CPP (see ControlNode::haltChild, which only
// calls it while the child's status is RUNNING).
class LongRunningAction : public BT::ActionNodeBase
{
public:
  LongRunningAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    ++tick_count;
    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    halted = true;
  }

  int tick_count{0};
  bool halted{false};
};

// Builds <control_tag name="Guard"><Inverter><ActiveCondition active="{active}"/>
// </Inverter><LongRunningAction name="work"/></control_tag>, the same shape
// as PlayMoveOrWaitForSystemActive wrapping RecoveryNode in chess_game.xml,
// parametrized so the exact same scenario can be run against both
// "Fallback" and "ReactiveFallback".
struct GuardFixture
{
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  BT::Blackboard::Ptr bb;
  LongRunningAction * work{nullptr};

  void build(const std::string & control_tag)
  {
    factory.registerNodeType<ActiveCondition>("ActiveCondition");
    factory.registerNodeType<LongRunningAction>("LongRunningAction");

    const std::string xml =
      R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
      ("<" + control_tag + R"( name="Guard">)") +
      R"(<Inverter><ActiveCondition active="{active}"/></Inverter>)"
      R"(<LongRunningAction name="work"/>)" +
      ("</" + control_tag + ">") +
      R"(</BehaviorTree></root>)";

    bb = BT::Blackboard::create();
    bb->set<bool>("active", true);
    tree = factory.createTreeFromText(xml, bb);

    for (auto & subtree : tree.subtrees) {
      for (auto & node : subtree->nodes) {
        if (auto * w = dynamic_cast<LongRunningAction *>(node.get())) {
          work = w;
        }
      }
    }
  }
};

}  // namespace

// Task 6 fix round 2 (Critical 2): a PAUSE arriving mid-move must halt the
// arm immediately, not only once the current move finishes on its own. A
// plain <Fallback> remembers which child last returned RUNNING and never
// re-ticks an earlier sibling while that child keeps returning RUNNING
// (FallbackNode::tick() only advances current_child_idx_ on FAILURE or
// SUCCESS), so once the work branch starts a move, the guard is never
// consulted again until that cycle ends -- the stop guard is deaf for the
// entire duration of a move. <ReactiveFallback> re-ticks every child from
// the top on every single tick, so it notices the guard tripping and halts
// the running child immediately.
//
// The two tests below run the identical scenario against both wirings and
// assert opposite outcomes, so together they are a genuine discriminator:
// PlainFallbackNeverNoticesGuardTrippingMidRun is what proves
// ReactiveFallbackHaltsRunningWorkAsSoonAsGuardTrips isn't a test that would
// also pass against chess_game.xml wired with a plain Fallback.

TEST(ReactiveGuard, ReactiveFallbackHaltsRunningWorkAsSoonAsGuardTrips)
{
  GuardFixture f;
  f.build("ReactiveFallback");
  ASSERT_NE(f.work, nullptr);

  // Tick 1: active, so Inverter(ActiveCondition) FAILs and the fallback
  // falls through to the work branch, which starts running (mirrors
  // starting a move while the system is active).
  EXPECT_EQ(f.tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(f.work->tick_count, 1);
  EXPECT_FALSE(f.work->halted);

  // The system is deactivated mid-move (a PAUSE arriving while the arm is
  // moving).
  f.bb->set<bool>("active", false);

  // Tick 2: the guard now succeeds and must halt the still-running work
  // branch immediately, without waiting for it to finish on its own.
  EXPECT_EQ(f.tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(f.work->halted);
  EXPECT_EQ(f.work->tick_count, 1) << "work must not be ticked again once paused";
}

TEST(ReactiveGuard, PlainFallbackNeverNoticesGuardTrippingMidRun)
{
  GuardFixture f;
  f.build("Fallback");
  ASSERT_NE(f.work, nullptr);

  EXPECT_EQ(f.tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(f.work->tick_count, 1);

  f.bb->set<bool>("active", false);

  // This is the defect a ReactiveFallback fixes: a plain Fallback is locked
  // onto the running child and never re-ticks the guard, so the work branch
  // keeps running -- and keeps sending arm goals -- straight through the
  // pause.
  EXPECT_EQ(f.tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_FALSE(f.work->halted);
  EXPECT_EQ(f.work->tick_count, 2);
}
