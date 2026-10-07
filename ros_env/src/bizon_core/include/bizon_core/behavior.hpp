#ifndef BIZON_CORE__BEHAVIOR_HPP_
#define BIZON_CORE__BEHAVIOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bizon_util/lifecycle_node.hpp"

namespace bizon_core
{
class Behavior
{
public:
  using Ptr = std::shared_ptr<Behavior>;
  /**
   * @brief Virtual destructor
  */
  virtual ~Behavior() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
  */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
  */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active Behavior and any threads involved in execution.
  */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
  */
  virtual void deactivate() = 0;

  virtual std::string getName() const = 0;

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
  */
};

}  // namespace bizon_core

#endif  // BIZON_CORE__BEHAVIOR_HPP_
