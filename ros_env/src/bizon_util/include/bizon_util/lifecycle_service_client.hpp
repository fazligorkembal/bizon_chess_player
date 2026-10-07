#ifndef BIZON_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
#define BIZON_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "bizon_util/service_client.hpp"
#include "bizon_util/node_utils.hpp"

namespace bizon_util
{
class LifecycleServiceClient
{
public:
  explicit LifecycleServiceClient(const std::string & lifecycle_node_name);
  LifecycleServiceClient(
    const std::string & lifecycle_node_name,
    rclcpp::Node::SharedPtr node);

  /// Trigger a state change
  /**
       * Throws std::runtime_error on failure
       */
  bool change_state(
    const uint8_t transition,         // takes a lifecycle_msgs::msg::Transition id
    const std::chrono::seconds timeout);

  /// Trigger a state change, returning result
  bool change_state(std::uint8_t transition);

  /// Get the current state as a lifecycle_msgs::msg::State id value
  /**
       * Throws std::runtime_error on failure
       */
  uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2));

protected:
  rclcpp::Node::SharedPtr node_;
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};
} // namespace bizon_util
#endif // BIZON_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
