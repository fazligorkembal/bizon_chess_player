#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "bizon_util/node_utils.hpp"
#include "bizon_behavior_servers/behavior_server.hpp"

namespace behavior_server
{
BehaviorServer::BehaviorServer(const rclcpp::NodeOptions & options)
: LifecycleNode("behavior_server", "", options),
  plugin_loader_("bizon_core", "bizon_core::Behavior")
{
  declare_parameter("cycle_frequency", rclcpp::ParameterValue(10.0));
  declare_parameter("behavior_plugins", default_ids_);

  get_parameter("behavior_plugins", behavior_ids_);
  if (behavior_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }
  for (auto id : behavior_ids_) {
    RCLCPP_INFO(
      get_logger(), "Found behavior plugin %s", id.c_str());
  }
}

BehaviorServer::~BehaviorServer()
{
  RCLCPP_INFO(get_logger(), "BehaviorServer shutting down.");
  behaviors_.clear();
}


bizon_util::CallbackReturn BehaviorServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring behavior server");

  behavior_types_.resize(behavior_ids_.size());
  if (!loadBehaviorPlugins()) {
    return bizon_util::CallbackReturn::FAILURE;
  }
  configureBehaviorPlugins();
  return bizon_util::CallbackReturn::SUCCESS;
}

bool BehaviorServer::loadBehaviorPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != behavior_ids_.size(); i++) {
    try {
      behavior_types_[i] = bizon_util::get_plugin_type_param(node, behavior_ids_[i]);
      RCLCPP_INFO(
        get_logger(), "Creating behavior plugin %s of type %s",
        behavior_ids_[i].c_str(), behavior_types_[i].c_str());
      behaviors_.push_back(plugin_loader_.createUniqueInstance(behavior_types_[i]));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to create behavior %s of type %s",
        behavior_ids_[i].c_str(), behavior_types_[i].c_str());
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      return false;
    }
  }
  return true;
}

void BehaviorServer::configureBehaviorPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != behavior_ids_.size(); i++) {
    behaviors_[i]->configure(
      node,
      behavior_ids_[i]
    );
  }
}

bizon_util::CallbackReturn BehaviorServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating behavior server");
  std::vector<pluginlib::UniquePtr<bizon_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->activate();
    RCLCPP_INFO(get_logger(), "Activating behavior %s", (*iter)->getName().c_str());
  }

  // create bond connection
  createBond();

  return bizon_util::CallbackReturn::SUCCESS;
}

bizon_util::CallbackReturn BehaviorServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating behavior server");

  std::vector<pluginlib::UniquePtr<bizon_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->deactivate();
  }

  // destroy bond connection
  destroyBond();

  return bizon_util::CallbackReturn::SUCCESS;
}

bizon_util::CallbackReturn
BehaviorServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up behavior server");

  std::vector<pluginlib::UniquePtr<bizon_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->cleanup();
  }

  behaviors_.clear();

  return bizon_util::CallbackReturn::SUCCESS;
}

bizon_util::CallbackReturn BehaviorServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down behavior server");
  return bizon_util::CallbackReturn::SUCCESS;
}

}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_server::BehaviorServer)
