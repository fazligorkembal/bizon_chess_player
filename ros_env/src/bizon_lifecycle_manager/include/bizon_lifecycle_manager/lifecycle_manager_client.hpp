#ifndef BIZON_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
#define BIZON_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "bizon_msgs/srv/manage_lifecycle_nodes.hpp"
#include "bizon_util/service_client.hpp"

namespace bizon_lifecycle_manager
{
    /**
     * @enum bizon_lifecycle_manager::SystemStatus
     * @brief Enum class representing the status of the system.
     */
    enum class SystemStatus
    {
        ACTIVE,
        INACTIVE,
        TIMEOUT
    };
    /**
     * @class bizon_lifecycle_manager::LifeCycleMangerClient
     * @brief The LifecycleManagerClient sends requests to the LifecycleManager to
     * control the lifecycle state of the modules.
     */
    class LifecycleManagerClient
    {
    public:
        /**
         * @brief A constructor for bizon_lifecycle_manager::LifecycleManagerClient
         * @param name Managed node name
         * @param parent_node Node that executes the service calls
         */
        explicit LifecycleManagerClient(
            const std::string &name,
            std::shared_ptr<rclcpp::Node> parent_node);
    

    // Client-side interface to the bizon lifecycle manager
    /**
     * @brief Make start up service call
     * @return true or false
     */
    bool startup(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    /**
     * @brief Make shutdown service call
     * @return true or false
     */
    bool shutdown(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    /**
     * @brief Make pause service call
     * @return true or false
     */
    bool pause(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    /**
     * @brief Make resume service call
     * @return true or false
     */
    bool resume(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    /**
     * @brief Make reset service call
     * @return true or false
     */
    bool reset(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    /**
     * @brief Check if lifecycle node manager server is active
     * @return ACTIVE or INACTIVE or TIMEOUT
     */
    SystemStatus is_active(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

    // // A couple convenience methods to facilitate scripting tests
    // /**
    //  * @brief Set initial pose with covariance
    //  * @param x X position
    //  * @param y Y position
    //  * @param theta orientation
    //  */
    // void set_initial_pose(double x, double y, double theta);
    // /**
    //  * @brief Send goal pose to NavigationToPose action server
    //  * @param x X position
    //  * @param y Y position
    //  * @param theta orientation
    //  * @return true or false
    //  */
    // bool navigate_to_pose(double x, double y, double theta);

protected:
  using ManageLifecycleNodes = bizon_msgs::srv::ManageLifecycleNodes;

  /**
   * @brief A generic method used to call startup, shutdown, etc.
   * @param command
   */
  bool callService(
    uint8_t command,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // The node to use for the service call
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<bizon_util::ServiceClient<ManageLifecycleNodes>> manager_client_;
  std::shared_ptr<bizon_util::ServiceClient<std_srvs::srv::Trigger>> is_active_client_;
  std::string manage_service_name_;
  std::string active_service_name_;
};

}  // namespace bizon_lifecycle_manager

#endif  // BIZON_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_