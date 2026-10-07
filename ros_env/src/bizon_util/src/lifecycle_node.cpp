#include "bizon_util/lifecycle_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace bizon_util
{

LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
{
  // server side never times out from lifecycle manager
  this->declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true);
  this->set_parameter(
    rclcpp::Parameter(
      bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

  printLifecycleNodeNotification();

  register_rcl_preshutdown_callback();
}

LifecycleNode::~LifecycleNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");

  runCleanups();

  if (rcl_preshutdown_cb_handle_) {
    rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();
    context->remove_pre_shutdown_callback(*(rcl_preshutdown_cb_handle_.get()));
    rcl_preshutdown_cb_handle_.reset();
  }
}

void LifecycleNode::createBond()
{
  RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", this->get_name());

  bond_ = std::make_unique<bond::Bond>(
    std::string("bond"),
    this->get_name(),
    shared_from_this());

  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();
}

void LifecycleNode::runCleanups()
{
  /*
       * In case this lifecycle node wasn't properly shut down, do it here.
       * We will give the user some ability to clean up properly here, but it's
       * best effort; i.e. we aren't trying to account for all possible states.
       */
  if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    this->deactivate();
  }

  if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    this->cleanup();
  }
}

void LifecycleNode::on_rcl_preshutdown()
{
  RCLCPP_INFO(
    get_logger(), "Running Bizon LifecycleNode rcl preshutdown (%s)",
    this->get_name());

  runCleanups();

  destroyBond();
}

void LifecycleNode::register_rcl_preshutdown_callback()
{
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  rcl_preshutdown_cb_handle_ = std::make_unique<rclcpp::PreShutdownCallbackHandle>(
    context->add_pre_shutdown_callback(
      std::bind(&LifecycleNode::on_rcl_preshutdown, this)));
}

void LifecycleNode::destroyBond()
{
  RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", this->get_name());

  if (bond_) {
    bond_.reset();
  }
}

void LifecycleNode::printLifecycleNodeNotification()
{
  RCLCPP_INFO(
    get_logger(),
    "\n\t%s lifecycle node launched. \n"
    "\tWaiting on external lifecycle transitions to activate\n"
    "\tSee https://design.ros2.org/articles/node_lifecycle.html for more information.",
    get_name());
}

} // namespace bizon_util
