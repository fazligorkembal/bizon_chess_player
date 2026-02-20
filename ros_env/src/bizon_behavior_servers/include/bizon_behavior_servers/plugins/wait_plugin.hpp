#ifndef BIZON_BEHAVIOR_SERVERS_PLUGINS_WAIT_HPP
#define BIZON_BEHAVIOR_SERVERS_PLUGINS_WAIT_HPP

#include <chrono>
#include <string>
#include <memory>

#include "bizon_behavior_servers/timed_behavior.hpp"
#include "bizon_msgs/action/wait.hpp"

namespace bizon_behaviors
{
    using WaitAction = bizon_msgs::action::Wait;

    class WaitPlugin : public TimedBehavior<WaitAction>
    {
    public:
        using WaitActionGoal = WaitAction::Goal;

        /**
         * @brief A constructor for behavior_server::Wait
         */
        WaitPlugin();
        ~WaitPlugin();

        /**
         * @brief Initialization to run behavior
         * @param command Goal to execute
         * @return Status of behavior
         */
        ResultStatus onRun(const std::shared_ptr<const WaitActionGoal> command) override;

        /**
         * @brief Loop function to run behavior
         * @return Status of behavior
         */
        ResultStatus onCycleUpdate() override;

    protected:
        rclcpp::Time wait_end_;
        WaitAction::Feedback::SharedPtr feedback_;
    };
} // namespace behavior_server

#endif // behavior_server_PLUGINS_WAIT_HPP