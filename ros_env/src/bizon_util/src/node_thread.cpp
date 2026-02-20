#include "bizon_util/node_thread.hpp"

namespace bizon_util
{
    NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
        : node_(node_base)
    {
        RCLCPP_INFO(rclcpp::get_logger("NodeThread"), "Starting NodeThread %s", node_->get_fully_qualified_name());
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        thread_ = std::make_unique<std::thread>([this](){
            executor_->add_node(node_);
            executor_->spin();
            executor_->remove_node(node_);
        });
    }

    NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) : executor_(executor)
    {
        RCLCPP_INFO(rclcpp::get_logger("NodeThread"), "Starting NodeThread with existing executor");

        thread_ = std::make_unique<std::thread>([this](){
            executor_->spin();
        });
    }

    NodeThread::~NodeThread()
    {
        executor_->cancel();
        thread_->join();
    }
} // namespace bizon_util