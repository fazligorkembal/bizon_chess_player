// Subscribes /rosout and appends WARN-and-above messages whose logger name
// belongs to this robot's namespace to <debug_session_dir>/rosout.log. This
// is what puts MoveIt, OMPL and controller failures in the same folder as
// the rest of a game's debug output -- they otherwise only exist on a
// console nobody is reading (or a ~/.ros/log file nobody thinks to check).
//
// Deliberately a standalone rclcpp node, not a bizon_core::Behavior plugin:
// it has no goal to serve, nothing ever commands it, and it has no failure
// mode that should abort a game -- lifecycle-managing it alongside
// behavior_server would only add ceremony for no benefit. See
// bizon_lifecycle_dev.launch.py for how it is started.
#include <chrono>
#include <cstdint>
#include <string>

#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bizon_behavior_servers/debug_session.hpp"
#include "bizon_behavior_servers/debug_session_format.hpp"

namespace
{

// Converts a builtin_interfaces/Time (the /rosout message's own stamp, not
// this process's clock -- two robots' rosout.log files should each read the
// time the message was actually logged) into a system_clock::time_point for
// debug_format::formatTimestamp().
std::chrono::system_clock::time_point toTimePoint(const builtin_interfaces::msg::Time & stamp)
{
  return std::chrono::system_clock::time_point{
    std::chrono::seconds{stamp.sec} + std::chrono::nanoseconds{stamp.nanosec}};
}

std::string levelName(uint8_t level)
{
  switch (level) {
    case rcl_interfaces::msg::Log::DEBUG:
      return "DEBUG";
    case rcl_interfaces::msg::Log::INFO:
      return "INFO";
    case rcl_interfaces::msg::Log::WARN:
      return "WARN";
    case rcl_interfaces::msg::Log::ERROR:
      return "ERROR";
    case rcl_interfaces::msg::Log::FATAL:
      return "FATAL";
    default:
      return "LEVEL" + std::to_string(level);
  }
}

}  // namespace

class RosoutLoggerNode : public rclcpp::Node
{
public:
  RosoutLoggerNode()
  : rclcpp::Node("rosout_logger")
  {
    declare_parameter("debug_session_dir", std::string(""));
    declare_parameter("namespace_prefix", std::string(""));

    const std::string debug_session_dir = get_parameter("debug_session_dir").as_string();
    namespace_prefix_ = get_parameter("namespace_prefix").as_string();
    bizon_behaviors::DebugSession::instance().configure(debug_session_dir);

    if (debug_session_dir.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "debug_session_dir is empty; rosout_logger is running but will not write anything");
    }

    // Matches the rosout aggregator's own publisher QoS
    // (rmw_qos_profile_rosout_default: KEEP_LAST 1000, RELIABLE,
    // TRANSIENT_LOCAL) so this subscription is compatible with it rather
    // than silently mismatching durability and seeing nothing.
    const rclcpp::QoS rosout_qos =
      rclcpp::QoS(rclcpp::KeepLast(1000)).reliable().transient_local();

    subscription_ = create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rosout_qos,
      std::bind(&RosoutLoggerNode::onLog, this, std::placeholders::_1));
  }

private:
  void onLog(const rcl_interfaces::msg::Log::SharedPtr msg)
  {
    if (msg->level < rcl_interfaces::msg::Log::WARN) {
      return;
    }
    // Logger names are the node's fully-qualified name with '/' replaced by
    // '.', so a node pushed into namespace "/bizon2" logs as
    // "bizon2.<node_name>". Matching on that prefix (or an exact match, for
    // a logger not tied to a node name) is the "belongs to this robot's
    // namespace" filter -- there is no first-class ROS concept of "which
    // namespace produced this log line" beyond the name string itself.
    if (!namespace_prefix_.empty()) {
      const bool belongs_to_namespace =
        msg->name == namespace_prefix_ || msg->name.rfind(namespace_prefix_ + ".", 0) == 0;
      if (!belongs_to_namespace) {
        return;
      }
    }

    const std::string line = bizon_behaviors::debug_format::formatLogLine(
      toTimePoint(msg->stamp), levelName(msg->level), msg->name + ": " + msg->msg);
    bizon_behaviors::DebugSession::instance().logRosout(line);
  }

  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
  std::string namespace_prefix_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosoutLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
