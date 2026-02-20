#include <chrono>
#include <memory>

#include "bizon_behavior_servers/plugins/wait_plugin.hpp"

namespace bizon_behaviors
{
WaitPlugin::WaitPlugin() : TimedBehavior<WaitAction>(),
    feedback_(std::make_shared<WaitAction::Feedback>())
{
}

WaitPlugin::~WaitPlugin() = default;
ResultStatus WaitPlugin::onRun(const std::shared_ptr<const WaitAction::Goal> command)
{
  wait_end_ = node_.lock()->now() + rclcpp::Duration(command->time);
  return ResultStatus{Status::SUCCEEDED};
}

ResultStatus WaitPlugin::onCycleUpdate()
{
    auto current_point = node_.lock()->now();
    auto time_left = wait_end_ - current_point;

    feedback_->time_left = time_left;
    action_server_->publish_feedback(feedback_);

    if (time_left.nanoseconds() > 0) {
        return ResultStatus{Status::RUNNING};
    } else {
        return ResultStatus{Status::SUCCEEDED};
    }
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bizon_behaviors::WaitPlugin, bizon_core::Behavior)