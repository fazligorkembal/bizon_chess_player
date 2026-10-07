#ifndef BIZON_BEHAVIOR_CLIENTS__BT_ACTION_CLIENT_NODE_HPP
#define BIZON_BEHAVIOR_CLIENTS__BT_ACTION_CLIENT_NODE_HPP

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp/action_node.h"
#include "bizon_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bizon_behavior_clients/bt_conversions.hpp"

namespace bizon_behavior_clients
{
using namespace std::chrono_literals;     // NOLINT
/**
     * @brief Abstract class representing an action based BT node
     * @tparam ActionT Type of action
     */
template<class ActionT>
class BtActionClientNode : public BT::ActionNodeBase
{
public:
  /**
       * @brief A bizon_behavior_tree::BtActionClientNode constructor
       * @param xml_tag_name Name for the XML tag for this node
       * @param action_name Action name this node creates a client for
       * @param conf BT node configuration
       */
  BtActionClientNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name), should_send_goal_(true)
  {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "BtActionClientNode"), "Creating action client for action: %s", action_name_.c_str());
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get The required items from the blackboard
    bt_loop_duration_ = config().blackboard->template get<std::chrono::milliseconds>(
      "bt_loop_duration");
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");

    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
      RCLCPP_WARN(
        rclcpp::get_logger(
          "BtActionClientNode"), "Remapped action name: %s", action_name_.c_str());
    }
    createActionClient(action_name_);
  }

  BtActionClientNode() = delete;

  virtual ~BtActionClientNode()
  {
    RCLCPP_INFO(rclcpp::get_logger("BtActionClientNode"), "Shutting down callback group executor");
    RCLCPP_INFO(
      rclcpp::get_logger(
        "BtActionClientNode"), "Destroying action client for action: %s", action_name_.c_str());
  }

  /**
       * @brief Function to create the action client
       * @param action_name Name of the action to create a client for
       */
  void createActionClient(const std::string & action_name)
  {
    // Now that we have the ROS node to use, create the action client for this BT action
    RCLCPP_INFO(
      rclcpp::get_logger(
        "BtActionClientNode"), "Creating action client for action: %s", action_name.c_str());
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    // Make sure the server is actually there before continuing
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "BtActionClientNode"), "Waiting for action server: %s", action_name.c_str());
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "BtActionClientNode"), "Action server %s not available after waiting for 5s",
        action_name.c_str());
      throw std::runtime_error(
              std::string("Action server ") + action_name +
              std::string(" not available"));
    }
    RCLCPP_INFO(
      rclcpp::get_logger(
        "BtActionClientNode"), "Action client for action: %s created", action_name.c_str());
  }

  /**
       * @brief Any subclass of BtActionClientNode that accepts parameters must provide a
       * providedPorts method and call providedBasicPorts in it.
       * @param addition Additional ports to add to BT port list
       * @return BT::PortsList Containing basic ports along with node-specific ports
       */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
       * @brief Creates list of BT ports
       * @return BT::PortsList Containing basic ports along with node-specific ports
       */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  /**
       * @brief Function to perform some user-defined operation on tick
       * Could do dynamic checks, such as getting updates to values on the blackboard
       */
  virtual void on_tick()
  {
  }

  virtual BT::NodeStatus onResultReceived(
    const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) = 0;

  /**
       * @brief Function to perform some user-defined operation after a timeout
       * waiting for a result that hasn't been received yet. Also provides access to
       * the latest feedback message from the action server. Feedback will be nullptr
       * in subsequent calls to this function if no new feedback is received while waiting for a result.
       * @param feedback shared_ptr to latest feedback message, nullptr if no new feedback was received
       */
  virtual void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback>/*feedback*/)
  {
  }

  /**
       * @brief Function to perform some user-defined operation upon successful
       * completion of the action. Could put a value on the blackboard.
       * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
       */
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
       * @brief Function to perform some user-defined operation whe the action is aborted.
       * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
       */
  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  /**
       * @brief Function to perform some user-defined operation when the action is cancelled.
       * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
       */
  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
       * @brief The main override required by a BT action
       * @return BT::NodeStatus Status of tick execution
       */
  BT::NodeStatus tick() override
  {
    // rist step to be done only at the beggining of the action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      // reset the flag to send the goal or not, allowing the user the option to set it in on_tick
      should_send_goal_ = true;

      // user defined callback, may modify should_send_goal_
      on_tick();

      if (!should_send_goal_) {
        // user decided not to send the goal, so return SUCCESS
        return BT::NodeStatus::SUCCESS;
      }
      send_new_goal();
    }

    try {
      // if new goal was sent and action server has not yet responded
      // check the future goal handle
      if (future_goal_handle_) {
        auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
        if (!is_future_goal_handle_complete(elapsed)) {
          if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;
          }

          RCLCPP_WARN(
            rclcpp::get_logger(
              "BtActionClientNode"), "Timed out while waiting for action server to acknowledge goal request for %s",
            action_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }

      // The following code corresponds to the "RUNNING" loop
      if (rclcpp::ok() && !goal_result_available_) {
        // user defined callback, May modify the value of "goal_updated_"
        on_wait_for_result(feedback_);

        // reset feedback to avoid state information
        feedback_.reset();

        auto goal_status = goal_handle_->get_status();
        if (goal_updated_ &&
          (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
        {
          goal_updated_ = false;
          send_new_goal();
          auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
          if (!is_future_goal_handle_complete(elapsed)) {
            if (elapsed < server_timeout_) {
              return BT::NodeStatus::RUNNING;
            }

            RCLCPP_WARN(
              rclcpp::get_logger(
                "BtActionClientNode"), "Timed out while waiting for action server to acknowledge goal request for %s",
              action_name_.c_str());
            future_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
          }
        }

        callback_group_executor_.spin_some();

        // check if, after invoking spin_some(), we finally received the result
        if (!goal_result_available_) {
          // Yield this Action, returning RUNNING
          return BT::NodeStatus::RUNNING;
        }
      }
    } catch (const std::runtime_error & e) {
      if (e.what() == std::string("send_goal failed") ||
        e.what() == std::string("Goal was rejected by the action server"))
      {
        // Action related failure that should not fail the tree, but the node
        return BT::NodeStatus::FAILURE;
      } else {
        // Internal exception to propogate to the tree
        throw e;
      }
    }

    BT::NodeStatus status;
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status = on_success();
        onResultReceived(result_);
        break;

      case rclcpp_action::ResultCode::ABORTED:
        status = on_aborted();
        break;

      case rclcpp_action::ResultCode::CANCELED:
        status = on_cancelled();
        break;

      default:
        throw std::logic_error("BtActionClientNode::Tick: invalid status value");
    }

    goal_handle_.reset();
    return status;
  }

  /**
       * @brief The other (optional) override required by a BT action. In this case, we
       * make sure to cancel the ROS2 action if it is still running.
       */
  void halt() override
  {
    if (should_cancel_goal()) {
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      if (callback_group_executor_.spin_until_future_complete(
          future_cancel,
          server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "BtActionClientNode"), "Failed to cancel goal for %s", action_name_.c_str());
      }
    }
    resetStatus();
  }

protected:
  /**
       * @brief Function to check if current goal should be cancelled
       * @return bool True if current goal should be cancelled, false otherwise
       */
  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    // No need to cancel the goal if goal handle is invalid
    if (!goal_handle_) {
      return false;
    }

    callback_group_executor_.spin_some();
    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
           status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED;
  }

  /**
       * @brief Function to send new goal to the action server
       */
  void send_new_goal()
  {
    goal_result_available_ = false;
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)
      {
        if (future_goal_handle_) {
          RCLCPP_DEBUG(
            rclcpp::get_logger(
              "BtActionClientNode"), "Goal result for %s available, but it hasn't received the goal response yet. It's probably a goal result for the last goal request",
            action_name_.c_str());
          return;
        }

        // TODO(#1652): a work around until rcl_action interface is updated
        // if goal ids are not matched, the older goal call this callback so ignore the result
        // if matched, it must be processed (including aborted)
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
        }
      };

    send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const typename ActionT::Feedback> feedback)
      {
        feedback_ = feedback;
      };

    future_goal_handle_ =
      std::make_shared<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));
    time_goal_sent_ = node_->now();
  }

  /**
       * @brief Function to check if the action server acknowledged a new goal
       * @param elapsed Duration since the last goal was sent and future goal handle has not completed.
       * After waiting for the future to complete, this value is incremented with the timeout value.
       * @return boolean True if future_goal_handle_ returns SUCCESS, False otherwise
       */
  bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed)
  {
    auto remaining = server_timeout_ - elapsed;

    // server has already timed out, no need to sleep
    if (remaining <= std::chrono::milliseconds(0)) {
      return false;
    }

    auto timedout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
    auto result =
      callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timedout);
    elapsed += timedout;

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_->get();
      future_goal_handle_.reset();
      if (!goal_handle_) {
        throw std::runtime_error("Goal was rejected by the action server");
      }
      return true;
    }
    return false;
  }

  /***
       * @brief Function to increment recovery count on blackboard if this node wraps a recovery
       */
  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);
    recovery_count++;
    config().blackboard->template set<int>("number_recoveries", recovery_count);
  }

  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // To handle feedback from action server
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // The node that will be used for any ROS2 operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while waiting for response from a server when a new action goal is sent or cancelled
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // To track the action server acknowledgement of a new goal is sent
  std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>
  future_goal_handle_;
  rclcpp::Time time_goal_sent_;

  // Can be set in on_tick or on_wait_for_result to indicate if a goal should be sent
  bool should_send_goal_;
};
}

#endif // BIZON_BEHAVIOR_CLIENTS__BT_ACTION_CLIENT_NODE_HPP
