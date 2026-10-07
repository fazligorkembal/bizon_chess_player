#ifndef BIZON_UTIL__NODE_THREAD_HPP_
#define BIZON_UTIL__NODE_THREAD_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace bizon_util
{
/**
     * @class bizon_util::NodeThread
     * @brief Runs a ROS 2 node in its own background thread with a dedicated executor,
     *        allowing the node to process callbacks independently without blocking other threads
     */

class NodeThread
{
public:
  /**
       * @brief A background thread to process node callbacks constructor
       * @param node_base Interface to Node to spin in thread
       */
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);

  /**
       * @brief Constructs a NodeThread from an existing SingleThreadedExecutor and starts spinning it in a background thread
       * @param executor Interface to executor to spin in thread
       */
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  /**
       * @brief Template constructor that accepts any node type (Node, LifecycleNode, etc.) and starts spinning it in a background thread
       * @param node Node pointer to spin in thread
       */
  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface()) {}

  /**
       * @brief Destructor to stop the background thread and join it
       */
  ~NodeThread();

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;

};

} // namespace bizon_util

#endif // BIZON_UTIL__NODE_THREAD_HPP_
