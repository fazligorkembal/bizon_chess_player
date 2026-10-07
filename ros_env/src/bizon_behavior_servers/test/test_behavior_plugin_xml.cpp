#include <gtest/gtest.h>

#include <string>

#include "tinyxml2.h"

// Structural guard: ArmPlugin cannot be unit-tested directly (it needs a
// live move_group, a resolved robot_description, and a reachable action
// server -- see task-4-report.md). This instead parses behavior_plugin.xml
// directly with tinyxml2, the same approach bizon_behavior_clients's
// test_chess_game_tree_structure.cpp uses for chess_game.xml, so it only
// proves the plugin export file is well-formed and has the right shape. It
// cannot prove pluginlib can actually load the class at runtime.

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

TEST(BehaviorPluginXml, XmlParsesCleanly)
{
  XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(BEHAVIOR_PLUGIN_XML_PATH), tinyxml2::XML_SUCCESS)
    << "behavior_plugin.xml failed to parse as well-formed XML: " << doc.ErrorStr();
}

TEST(BehaviorPluginXml, ArmPluginIsRegistered)
{
  XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(BEHAVIOR_PLUGIN_XML_PATH), tinyxml2::XML_SUCCESS);

  const XMLElement * root = doc.RootElement();
  ASSERT_NE(root, nullptr);

  const XMLElement * found = nullptr;
  for (const XMLElement * library = root->FirstChildElement("library"); library != nullptr;
    library = library->NextSiblingElement("library"))
  {
    for (const XMLElement * cls = library->FirstChildElement("class"); cls != nullptr;
      cls = cls->NextSiblingElement("class"))
    {
      const char * name = cls->Attribute("name");
      if (name != nullptr && std::string(name) == "bizon_behaviors/ArmPlugin") {
        found = cls;
        break;
      }
    }
    if (found != nullptr) {
      break;
    }
  }

  ASSERT_NE(found, nullptr)
    << "behavior_plugin.xml has no <class name=\"bizon_behaviors/ArmPlugin\">";

  const char * type = found->Attribute("type");
  ASSERT_NE(type, nullptr);
  EXPECT_EQ(std::string(type), "bizon_behaviors::ArmPlugin");

  const char * base_class_type = found->Attribute("base_class_type");
  ASSERT_NE(base_class_type, nullptr);
  EXPECT_EQ(std::string(base_class_type), "bizon_core::Behavior");
}
