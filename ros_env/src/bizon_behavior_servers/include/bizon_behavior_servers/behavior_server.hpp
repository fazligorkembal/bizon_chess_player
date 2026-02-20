#ifndef BIZON_BEHAVIOR_SERVERS_BEHAVIOR_SERVER_HPP
#define BIZON_BEHAVIOR_SERVERS_BEHAVIOR_SERVER_HPP

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "bizon_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "bizon_core/behavior.hpp"

namespace behavior_server
{
/**
 * @class bizon_behavior_server::BehaviorServer
 * @brief An server hosting a map of behavior plugins
 */
class BehaviorServer : public bizon_util::LifecycleNode
{
public:
/**
 * @brief A constructor for bizon_behavior_server::BehaviorServer
 * @param options Additional options to control creation of the node.
*/
explicit BehaviorServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
~BehaviorServer();

protected:
/**
 * @brief Loads behavior plugins from parameter file
 * @return bool if successfully loaded the plugins
*/
bool loadBehaviorPlugins();

/**
 * @brief configures behavior plugins
*/
void configureBehaviorPlugins();

/**
 * @brief configures behavior plugins
*/
void setupResourcesForBehaviorPlugins();

/**
 * @brief Configure lifecycle server
*/
bizon_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

/**
 * @brief Activate lifecycle server
*/
bizon_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

/**
 * @brief Deactivate lifecycle server
*/
bizon_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

/**
 * @brief Cleanup lifecycle server
*/
bizon_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

/**
 * @brief Shutdown lifecycle server
*/
bizon_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

pluginlib::ClassLoader<bizon_core::Behavior> plugin_loader_;
std::vector<pluginlib::UniquePtr<bizon_core::Behavior>> behaviors_;
std::vector<std::string> default_ids_;
std::vector<std::string> default_types_;
std::vector<std::string> behavior_ids_;
std::vector<std::string> behavior_types_;

};

}

#endif // behavior_server_BEHAVIOR_SERVER_HPP