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
