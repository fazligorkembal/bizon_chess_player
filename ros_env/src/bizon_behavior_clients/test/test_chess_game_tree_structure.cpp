#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "tinyxml2.h"

// Structural guard for the fix-round-2 regression: RecoveryNode's work branch
// must never contain CheckGameOver again. If it does, an ordinary successful
// move fails the work branch (CheckGameOver only SUCCEEDs when the game is
// actually over), which makes RecoveryNode spend a retry and run the
// recovery subtree after every single move, and after `number_of_retries`
// moves the whole tree gives up mid-game. See the "Fix round 2" section of
// task-3-report.md for the full story.
//
// This deliberately does NOT build a BT::Tree: doing so would construct a
// live ArmActionClientNode (which needs a running node, a resolved
// robot_description parameter, and a reachable move_group action server)
// plus every other plugin's runtime dependencies, none of which exist in
// this unit-test environment. Instead it parses chess_game.xml directly with
// tinyxml2 and walks the DOM, so it only proves the shipped XML is
// well-formed and has the right shape -- it cannot prove the tree behaves
// correctly when actually ticked. That behavioral check is deferred to the
// human partner's Isaac Sim run (see task-3-report.md, "Sim step deferred").

namespace
{

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

/// Depth-first search for the first descendant (not including `root` itself)
/// whose tag name is `tag_name`. Returns nullptr if none is found.
const XMLElement * findDescendantByTag(const XMLElement * root, const char * tag_name)
{
  for (const XMLElement * child = root->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement())
  {
    if (std::string(child->Name()) == tag_name) {
      return child;
    }
    if (const XMLElement * found = findDescendantByTag(child, tag_name)) {
      return found;
    }
  }
  return nullptr;
}

/// Collects the `name` attribute of every element in the subtree rooted at
/// `root` (root included) into `out`.
void collectNameAttributes(const XMLElement * root, std::vector<std::string> & out)
{
  if (const char * name = root->Attribute("name")) {
    out.emplace_back(name);
  }
  for (const XMLElement * child = root->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement())
  {
    collectNameAttributes(child, out);
  }
}

bool contains(const std::vector<std::string> & haystack, const std::string & needle)
{
  for (const auto & s : haystack) {
    if (s == needle) {
      return true;
    }
  }
  return false;
}

/// Counts every element in the subtree rooted at `root` (root included)
/// whose tag name is `tag_name`.
int countDescendantsByTag(const XMLElement * root, const char * tag_name)
{
  int count = (std::string(root->Name()) == tag_name) ? 1 : 0;
  for (const XMLElement * child = root->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement())
  {
    count += countDescendantsByTag(child, tag_name);
  }
  return count;
}

/// Depth-first search for the first descendant of `root` (root included) whose
/// tag is `tag_name` AND whose `name` attribute equals `name_attr`.
const XMLElement * findByTagAndName(
  const XMLElement * root, const char * tag_name, const char * name_attr)
{
  const char * name = root->Attribute("name");
  if (std::string(root->Name()) == tag_name && name != nullptr &&
    std::string(name) == name_attr)
  {
    return root;
  }
  for (const XMLElement * child = root->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement())
  {
    if (const XMLElement * found = findByTagAndName(child, tag_name, name_attr)) {
      return found;
    }
  }
  return nullptr;
}

}  // namespace

TEST(ChessGameTreeStructure, XmlParsesCleanly)
{
  XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(CHESS_GAME_XML_PATH), tinyxml2::XML_SUCCESS)
    << "chess_game.xml failed to parse as well-formed XML: " << doc.ErrorStr();
}

TEST(ChessGameTreeStructure, CheckGameOverIsNotInsideRecoveryNodeWorkBranch)
{
  XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(CHESS_GAME_XML_PATH), tinyxml2::XML_SUCCESS);

  const XMLElement * root = doc.RootElement();
  ASSERT_NE(root, nullptr);

  // Find <BehaviorTree ID="MainTree">.
  const XMLElement * main_tree = nullptr;
  for (const XMLElement * bt = root->FirstChildElement("BehaviorTree"); bt != nullptr;
       bt = bt->NextSiblingElement("BehaviorTree"))
  {
    const char * id = bt->Attribute("ID");
    if (id != nullptr && std::string(id) == "MainTree") {
      main_tree = bt;
      break;
    }
  }
  ASSERT_NE(main_tree, nullptr) << "chess_game.xml has no <BehaviorTree ID=\"MainTree\">";

  const XMLElement * recovery_node = findDescendantByTag(main_tree, "RecoveryNode");
  ASSERT_NE(recovery_node, nullptr) << "MainTree has no RecoveryNode";

  // RecoveryNode's first child element is the work branch (child 0, per
  // RecoveryNode's documented contract); its second child is the recovery
  // branch.
  const XMLElement * work_branch = recovery_node->FirstChildElement();
  ASSERT_NE(work_branch, nullptr) << "RecoveryNode has no children";

  std::vector<std::string> work_branch_names;
  collectNameAttributes(work_branch, work_branch_names);
  EXPECT_FALSE(contains(work_branch_names, "CheckGameOver"))
    << "CheckGameOver must not be a descendant of RecoveryNode's work branch: "
       "a successful move would then fail the work branch, spend a retry, "
       "and run the recovery subtree after every move.";

  // Guard against the check disappearing entirely rather than moving out:
  // CheckGameOver must still be present somewhere in MainTree.
  std::vector<std::string> main_tree_names;
  collectNameAttributes(main_tree, main_tree_names);
  EXPECT_TRUE(contains(main_tree_names, "CheckGameOver"))
    << "CheckGameOver is missing from MainTree entirely";
}

// Waiting for the opponent is the normal steady state, not a fault. It used to
// be reported as FAILURE from inside RecoveryNode's work branch, so three
// consecutive "not my turn" ticks exhausted the retries and the tree gave up
// mid-game -- fatal when the opponent is a human who takes their time.
// MakeDecisionClient now reports waiting as SUCCESS with move_type "wait", and
// the move subtree must sit behind a guard that skips it in that case;
// otherwise Foreach would run with move_count 0 and FAIL (foreach_node.cpp
// returns FAILURE for count <= 0), putting the fault back.
TEST(ChessGameTreeStructure, MoveSubtreeIsSkippedWhileWaitingForTheOpponent)
{
  XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(CHESS_GAME_XML_PATH), tinyxml2::XML_SUCCESS);

  const XMLElement * root = doc.RootElement();
  ASSERT_NE(root, nullptr);

  const XMLElement * main_tree = nullptr;
  for (const XMLElement * bt = root->FirstChildElement("BehaviorTree"); bt != nullptr;
       bt = bt->NextSiblingElement("BehaviorTree"))
  {
    const char * id = bt->Attribute("ID");
    if (id != nullptr && std::string(id) == "MainTree") {
      main_tree = bt;
      break;
    }
  }
  ASSERT_NE(main_tree, nullptr);

  const XMLElement * guard = findByTagAndName(main_tree, "Fallback", "MoveOrWaitForOpponent");
  ASSERT_NE(guard, nullptr)
    << "MainTree has no <Fallback name=\"MoveOrWaitForOpponent\">: the move subtree "
       "is unguarded, so a waiting tick would run Foreach with move_count 0 and fail.";

  // First child must be the wait check, so a waiting tick short-circuits before
  // any arm motion is planned.
  const XMLElement * wait_check = guard->FirstChildElement();
  ASSERT_NE(wait_check, nullptr) << "the guard has no children";
  EXPECT_EQ(std::string(wait_check->Name()), "ConditionNode");
  ASSERT_NE(wait_check->Attribute("param1"), nullptr);
  ASSERT_NE(wait_check->Attribute("param2"), nullptr);
  EXPECT_EQ(std::string(wait_check->Attribute("param1")), "{move_type}");
  EXPECT_EQ(std::string(wait_check->Attribute("param2")), "wait")
    << "the guard must compare move_type against the literal \"wait\" that "
       "MakeDecisionClient writes when it is the opponent's turn";

  // The move loop must live inside that guard, not beside it.
  const XMLElement * move_loop = findByTagAndName(guard, "Foreach", "MoveLoop");
  EXPECT_NE(move_loop, nullptr)
    << "Foreach MoveLoop must be a descendant of the MoveOrWaitForOpponent guard";
}

// Task 6 (F5, the software stop path): a paused system (lifecycle_manager PAUSE
// deactivates behavior_server) is a third instance of the same shape as
// waiting for the opponent above -- a normal, expected non-progress state,
// not a fault. The task brief's own Step 4 places IsSystemActive as the
// first element of RecoveryNode's work branch (Sequence "PlayOneMove").
// That placement is wrong: it makes a pause read as a fault, RecoveryNode
// spends a retry and runs its recovery subtree, which also needs the now-
// deactivated behavior_server and also fails, and after 3 retries the whole
// tree ends -- leaving nothing for a later RESUME command to act on. These
// two tests prove the guard instead gates *entry to RecoveryNode itself*,
// via a ReactiveFallback that sits outside RecoveryNode's work branch (it
// must be reactive, not a plain Fallback, so a PAUSE arriving mid-move is
// seen immediately rather than only at the next move boundary -- see
// test_reactive_guard_halts_running_work.cpp and chess_game.xml's header
// comment for that half of the story; this file only checks the XML has
// the right shape). Both tests fail against the brief's literal placement
// (confirmed by hand against that placement before this fix): the first
// because IsSystemActive would be found inside the work branch, and the
// second because no PlayMoveOrWaitForSystemActive fallback would exist at
// all -- RecoveryNode would be reachable unconditionally, paused or not.

TEST(ChessGameTreeStructure, IsSystemActiveIsNotInsideRecoveryNodeWorkBranch)
{
  XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(CHESS_GAME_XML_PATH), tinyxml2::XML_SUCCESS);

  const XMLElement * root = doc.RootElement();
  ASSERT_NE(root, nullptr);

  const XMLElement * main_tree = nullptr;
  for (const XMLElement * bt = root->FirstChildElement("BehaviorTree"); bt != nullptr;
       bt = bt->NextSiblingElement("BehaviorTree"))
  {
    const char * id = bt->Attribute("ID");
    if (id != nullptr && std::string(id) == "MainTree") {
      main_tree = bt;
      break;
    }
  }
  ASSERT_NE(main_tree, nullptr);

  ASSERT_NE(findDescendantByTag(main_tree, "IsSystemActive"), nullptr)
    << "MainTree has no IsSystemActive node: the stop guard is missing entirely";

  const XMLElement * recovery_node = findDescendantByTag(main_tree, "RecoveryNode");
  ASSERT_NE(recovery_node, nullptr) << "MainTree has no RecoveryNode";

  const XMLElement * work_branch = recovery_node->FirstChildElement();
  ASSERT_NE(work_branch, nullptr) << "RecoveryNode has no children";

  EXPECT_EQ(findDescendantByTag(work_branch, "IsSystemActive"), nullptr)
    << "IsSystemActive must not be inside RecoveryNode's work branch: a paused "
       "system would then read as a fault, and recovery -- which also needs the "
       "deactivated behavior_server -- would fail too, burning all retries and "
       "ending the tree with nothing left for a RESUME to act on.";
}

TEST(ChessGameTreeStructure, PausedSystemNeverTicksRecoveryNodeOrAnArmGoal)
{
  XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(CHESS_GAME_XML_PATH), tinyxml2::XML_SUCCESS);

  const XMLElement * root = doc.RootElement();
  ASSERT_NE(root, nullptr);

  const XMLElement * main_tree = nullptr;
  for (const XMLElement * bt = root->FirstChildElement("BehaviorTree"); bt != nullptr;
       bt = bt->NextSiblingElement("BehaviorTree"))
  {
    const char * id = bt->Attribute("ID");
    if (id != nullptr && std::string(id) == "MainTree") {
      main_tree = bt;
      break;
    }
  }
  ASSERT_NE(main_tree, nullptr);

  const XMLElement * guard =
    findByTagAndName(main_tree, "ReactiveFallback", "PlayMoveOrWaitForSystemActive");
  ASSERT_NE(guard, nullptr)
    << "MainTree has no <ReactiveFallback name=\"PlayMoveOrWaitForSystemActive\">: "
       "without it RecoveryNode is reachable unconditionally, regardless of system "
       "state. It must specifically be a ReactiveFallback, not a plain Fallback: a "
       "plain Fallback never re-ticks this guard once RecoveryNode starts running, "
       "so a PAUSE arriving mid-move would go unnoticed until the move finishes on "
       "its own (see test_reactive_guard_halts_running_work.cpp).";

  // First branch: the paused check. It must succeed exactly while the system
  // is inactive (Inverter of IsSystemActive), so the fallback short-circuits
  // before ever reaching RecoveryNode.
  const XMLElement * guard_branch = guard->FirstChildElement();
  ASSERT_NE(guard_branch, nullptr) << "the guard fallback has no children";
  EXPECT_EQ(std::string(guard_branch->Name()), "Inverter")
    << "the guard fallback's first child must invert IsSystemActive, so it "
       "SUCCEEDs -- short-circuiting the fallback -- exactly while the system "
       "is paused";
  EXPECT_NE(findDescendantByTag(guard_branch, "IsSystemActive"), nullptr)
    << "the guard fallback's first child must wrap IsSystemActive";
  EXPECT_EQ(findDescendantByTag(guard_branch, "RecoveryNode"), nullptr)
    << "RecoveryNode must not be reachable from the guard branch itself: a "
       "paused tick that satisfies the guard must never reach RecoveryNode, "
       "let alone its recovery subtree.";

  // Second branch: the real work, reached only when the first branch fails
  // (system active). RecoveryNode -- and everything it protects -- must live
  // here.
  const XMLElement * work_branch_wrapper = guard->LastChildElement();
  ASSERT_NE(work_branch_wrapper, nullptr) << "the guard fallback has only one child";
  ASSERT_NE(work_branch_wrapper, guard_branch)
    << "the guard fallback needs a second, distinct branch for the real work";
  const XMLElement * recovery_node_in_guard =
    (std::string(work_branch_wrapper->Name()) == "RecoveryNode") ?
    work_branch_wrapper : findDescendantByTag(work_branch_wrapper, "RecoveryNode");
  ASSERT_NE(recovery_node_in_guard, nullptr)
    << "the guard fallback's second branch must contain RecoveryNode";

  // No arm goal may be reachable outside that guarded RecoveryNode. Every
  // ArmActionClient element literally present in MainTree's XML (the
  // MoveSequence and RecoverArm subtrees are separate <BehaviorTree>
  // definitions reached via <SubTree>, so they don't appear here directly,
  // but they are only ever reached BY ticking RecoveryNode) must be nested
  // under it, so a paused tick -- which never reaches RecoveryNode -- can
  // never reach an arm goal either.
  const int total_arm_goals = countDescendantsByTag(main_tree, "ArmActionClient");
  const int guarded_arm_goals = countDescendantsByTag(recovery_node_in_guard, "ArmActionClient");
  ASSERT_GT(total_arm_goals, 0) << "sanity check: MainTree issues no ArmActionClient "
    "goals at all -- the fixture no longer matches this test's assumptions";
  EXPECT_EQ(total_arm_goals, guarded_arm_goals)
    << total_arm_goals - guarded_arm_goals << " of " << total_arm_goals
    << " ArmActionClient node(s) in MainTree sit outside the guarded RecoveryNode "
       "subtree, and so would still be reachable while the system is paused.";
}
