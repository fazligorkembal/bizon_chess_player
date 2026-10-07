#ifndef BIZON_BEHAVIOR_SERVERS__TIMED_BEHAVIOR_HPP
#define BIZON_BEHAVIOR_SERVERS__TIMED_BEHAVIOR_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "bizon_util/simple_action_server.hpp"
#include "bizon_core/behavior.hpp"
#include "bizon_behavior_servers/debug_session.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic pop

namespace bizon_behaviors
{
enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

struct ResultStatus
{
  Status status;
  uint16_t error_code{0};
};

using namespace std::chrono_literals;  //NOLINT

template<typename ActionT>
class TimedBehavior : public bizon_core::Behavior
{
public:
  using ActionServer = bizon_util::SimpleActionServer<ActionT>;

/**
 * @brief A TimedBehavior constructor
*/
  TimedBehavior()
  : action_server_(nullptr),
    cycle_frequency_(20.0),
    enabled_(false),
    transform_tolerance_(0.0)
  {}

  virtual ~TimedBehavior() = default;

// Derived classes can override this method to catch the command and perform some checks
// before getting into the main loop. The method will only be called
// once and should return SUCCEEDED otherwise behavior will return FAILED.
  virtual ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> command) = 0;

// This is the method derived classes should mainly implement
// and will be called cyclically while it returns RUNNING.
// Implement the behavior such that it runs some unit of work on each call
// and provides a status. The Behavior will finish once SUCCEEDED is returned
// It's up to the derived class to define the final commanded velocity.
  virtual ResultStatus onCycleUpdate() = 0;

// an opportunity for derived classes to do something on configuration
// if they chose
  virtual void onConfigure()
  {
  }

// an opportunity for derived classes to do something on cleanup
// if they chose
  virtual void onCleanup()
  {
  }

// an opportunity for a derived class to do something on action completion
  virtual void onActionCompletion(std::shared_ptr<typename ActionT::Result>/*result*/)
  {
  }

  virtual std::string getName() const
  {
    return behavior_name_;
  }

// configure the server on lifecycle setup
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name) override
  {
    node_ = parent;
    auto node = node_.lock();
    clock_ = node->get_clock();

    RCLCPP_INFO(
      node->get_logger(),
      "[%s] [TimedBehavior] Configuring behavior",
      name.c_str());

    behavior_name_ = name;

    node->get_parameter("cycle_frequency", cycle_frequency_);

    if (!node->has_parameter("action_server_result_timeout")) {
      node->declare_parameter("action_server_result_timeout", 10.0);
    }

    double action_server_result_timeout;
    node->get_parameter("action_server_result_timeout", action_server_result_timeout);
    rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
    server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

    action_server_ = std::make_shared<ActionServer>(
      node,
      behavior_name_,
      std::bind(&TimedBehavior::execute, this), nullptr, std::chrono::milliseconds(500),
      false,
      server_options
    );


    onConfigure();
  }

// Cleanup server on lifecycle transition
  void cleanup() override
  {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "[%s] [TimedBehavior] Cleaning up behavior",
      behavior_name_.c_str());

    action_server_.reset();
    onCleanup();
  }

// Activate server on lifecycle transition
  void activate() override
  {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "[%s] [TimedBehavior] Activating behavior",
      behavior_name_.c_str());
    action_server_->activate();
    enabled_ = true;
  }

// Deactivate server on lifecycle transition
  void deactivate() override
  {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "[%s] [TimedBehavior] Deactivating behavior",
      behavior_name_.c_str());
    action_server_->deactivate();
    enabled_ = false;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string behavior_name_;
  std::shared_ptr<ActionServer> action_server_;

  double cycle_frequency_;
  double enabled_;
  std::string local_frame_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  rclcpp::Duration elasped_time_{0, 0};

// Clock
  rclcpp::Clock::SharedPtr clock_;


  void execute()
  {
    //bizon_util::message_info(rclcpp::get_logger("timed_behavior"), {"Running ", behavior_name_, " behavior"});

    if (!enabled_) {
      RCLCPP_WARN(
        node_.lock()->get_logger(),
        "[%s] [TimedBehavior] Behavior is not enabled, cannot execute",
        behavior_name_.c_str());
      return;
    }

    // Initialize the ActionT result
    auto result = std::make_shared<typename ActionT::Result>();

    ResultStatus on_run_result = onRun(action_server_->get_current_goal());
    if (on_run_result.status != Status::SUCCEEDED) {
      RCLCPP_INFO(
        node_.lock()->get_logger(),
        "[%s] [TimedBehavior] onRun failed, aborting behavior",
        behavior_name_.c_str());
      result->error_code = on_run_result.error_code;
      // Generic abort logging lives here rather than in each plugin: this
      // is the only exit point onRun failures take, and onActionCompletion
      // (where BoardPlugin/DecisionPlugin/ArmPlugin log their own
      // per-behavior detail) is never called for an onRun failure -- see
      // the loop below for the onCycleUpdate FAILED case.
      DebugSession::instance().logEvent(
        "abort", "behavior=" + behavior_name_ +
        " error_code=" + std::to_string(on_run_result.error_code));
      action_server_->terminate_current(result);
      return;
    }

    auto start_time = clock_->now();
    rclcpp::WallRate loop_rate(cycle_frequency_);

    while (rclcpp::ok()) {
      elasped_time_ = clock_->now() - start_time;
      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(
          node_.lock()->get_logger(),
          "[%s] [TimedBehavior] Cancel requested, stopping behavior",
          behavior_name_.c_str());
        result->total_elapsed_time = elasped_time_;
        onActionCompletion(result);
        action_server_->terminate_all(result);
        return;
      }

      // TODO(orduno) #868 Enable preempting a Behavior on-the-fly without stopping
      if (action_server_->is_preempt_requested()) {
        RCLCPP_ERROR(
          node_.lock()->get_logger(),
          "[%s] [TimedBehavior] Received a preemption request, however feature is currently not implemented. Aborting and stopping.",
          behavior_name_.c_str());
        result->total_elapsed_time = clock_->now() - start_time;
        onActionCompletion(result);
        action_server_->terminate_current(result);
        return;
      }

      ResultStatus on_cycle_update_result = onCycleUpdate();
      switch (on_cycle_update_result.status) {
        case Status::SUCCEEDED:
          result->total_elapsed_time = clock_->now() - start_time;
          onActionCompletion(result);
          action_server_->succeeded_current(result);
          return;

        case Status::FAILED:
          RCLCPP_WARN(
            node_.lock()->get_logger(),
            "[%s] [TimedBehavior] onCycleUpdate returned FAILED, aborting behavior",
            behavior_name_.c_str());
          result->total_elapsed_time = clock_->now() - start_time;
          result->error_code = on_cycle_update_result.error_code;
          DebugSession::instance().logEvent(
            "abort", "behavior=" + behavior_name_ +
            " error_code=" + std::to_string(on_cycle_update_result.error_code));
          onActionCompletion(result);
          action_server_->terminate_current(result);
          return;

        case Status::RUNNING:

        default:
          loop_rate.sleep();
          break;
      }
    }
  }

};

} // namespace behavior_server
#endif  // behavior_server_TIMED_BEHAVIOR_HPP
