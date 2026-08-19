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
